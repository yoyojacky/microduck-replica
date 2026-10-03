"""SwdReader（pyOCD：ST-Link / DAPLink）测试。

用假的 pyocd 模块，不打开真探针、不接板子。
Run: python -m pytest -q test_imu_swd.py
"""
import contextlib
import io
import struct
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import imu_bridge as bridge
import server
from test_imu_bridge import snapshot

IMAGE = bytes(range(64))            # 假固件：64 字节，按字对齐
SAMPLE, TICK = 0x20000064, 0x20000BE0


class FakeState:
    HALTED = "halted"
    RUNNING = "running"


class FakeProbe:
    def __init__(self, uid, description="STM32 STLink"):
        self.unique_id, self.description = uid, description


class JLinkProbe(FakeProbe):
    """类名跟 pyOCD 的 J-Link 探针一样，SwdReader 按类名拒绝。"""


class FakeTarget:
    def __init__(self, memory, state=FakeState.RUNNING, events=None):
        self.memory, self.state = memory, state
        self.resumed = 0
        self.reads = []
        self.events = events if events is not None else []
        self.state_error = None

    def get_state(self):
        if self.state_error:
            raise self.state_error
        return self.state

    def resume(self):
        self.resumed += 1
        self.state = FakeState.RUNNING

    def _bytes(self, address, size):
        self.events.append(("read", address))
        for base, data in self.memory.items():
            if base <= address and address + size <= base + len(data):
                return data[address - base:address - base + size]
        raise RuntimeError(f"fault at 0x{address:08X}")

    def read_memory_block32(self, address, words):
        self.reads.append(("32", address, words * 4))
        return list(struct.unpack(f"<{words}I", self._bytes(address, words * 4)))

    def read_memory_block8(self, address, size):
        self.reads.append(("8", address, size))
        return list(self._bytes(address, size))


class FakeLink:
    """pyOCD 里只有 ST-Link 的 _link 能测目标电压。"""
    def __init__(self, volts, events=None, error=None):
        self.volts, self.target_voltage, self.error = volts, 0, error
        self.events = events if events is not None else []

    def get_target_voltage(self):
        self.events.append(("voltage",))
        if self.error:
            raise self.error
        self.target_voltage = self.volts


class Board:
    """一块假板：所有 Session 共用同一个目标和同一根探针链路。"""
    def __init__(self, memory=None, volts=3.29, state=FakeState.RUNNING, voltage_api=True,
                 probe_open_error=None, target_open_error=None, voltage_error=None):
        self.events = []
        self.target = FakeTarget(memory if memory is not None else board_memory(), state, self.events)
        self.link = FakeLink(volts, self.events, voltage_error) if voltage_api else None
        self.probe_open_error, self.target_open_error = probe_open_error, target_open_error
        self.sessions = []

    def session_class(self):
        board = self

        class Session:
            def __init__(self, probe, options=None):
                self.chosen, self.options = probe, options
                self.probe = types.SimpleNamespace(is_open=False)
                if board.link is not None:
                    self.probe._link = board.link
                self.target = None
                self.closed = 0
                board.sessions.append(self)

            def open(self, init_board=True):
                board.events.append(("open", init_board))
                if board.probe_open_error:
                    raise board.probe_open_error
                self.probe.is_open = True
                if init_board:
                    if board.target_open_error:
                        raise board.target_open_error
                    self.target = board.target

            def close(self):
                board.events.append(("close",))
                self.closed += 1
                self.probe.is_open = False
        return Session


def board_memory(image=IMAGE, sample=None, tick=1005):
    return {0x08000000: image, SAMPLE: sample if sample is not None else snapshot(),
            TICK: struct.pack("<I", tick)}


class SwdTestCase(unittest.TestCase):
    def fake_pyocd(self, probes, board):
        """把假 pyocd 塞进 sys.modules。"""
        class ConnectHelper:
            @staticmethod
            def get_all_connected_probes(blocking=False):
                return probes

        helpers = types.ModuleType("pyocd.core.helpers")
        helpers.ConnectHelper = ConnectHelper
        session_mod = types.ModuleType("pyocd.core.session")
        session_mod.Session = board.session_class()
        target_mod = types.ModuleType("pyocd.core.target")
        target_mod.Target = types.SimpleNamespace(State=FakeState)
        modules = {"pyocd": types.ModuleType("pyocd"), "pyocd.core": types.ModuleType("pyocd.core"),
                   "pyocd.core.helpers": helpers, "pyocd.core.session": session_mod,
                   "pyocd.core.target": target_mod}
        stack = contextlib.ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.dict(sys.modules, modules))
        return board

    def reader(self, probe_id=None, resume=False, image=IMAGE):
        layout = bridge.FirmwareLayout(image, "test", SAMPLE, TICK)
        return bridge.SwdReader(layout, probe_id, resume)


class SwdConnectTests(SwdTestCase):
    def test_single_probe_auto_attach_options(self):
        board = self.fake_pyocd([FakeProbe("ABC123")], Board(volts=3.15))
        reader = self.reader()
        reader.connect()
        options = board.sessions[-1].options
        self.assertEqual(options["connect_mode"], "attach")        # 不复位、不暂停
        self.assertFalse(options["resume_on_disconnect"])          # 断开不隐式 Go
        self.assertTrue(options["no_config"])                      # 不加载 pyocd.yaml / 用户脚本
        self.assertEqual(options["target_override"], "cortex_m")
        self.assertEqual(options["frequency"], bridge.SWD_FREQUENCY_HZ)
        self.assertFalse(options["auto_unlock"])
        self.assertEqual(reader.voltage_mv, 3150)
        self.assertEqual((reader.probe_name, reader.probe_uid), ("STM32 STLink", "ABC123"))
        self.assertEqual(board.target.resumed, 0)
        self.assertIn(("32", 0x08000000, len(IMAGE)), board.target.reads)  # 比对 Flash 按字读

    def test_voltage_is_checked_before_touching_the_chip(self):
        board = self.fake_pyocd([FakeProbe("A1")], Board())
        self.reader().connect()
        kinds = [e[0] if e[0] != "open" else f"open{int(e[1])}" for e in board.events]
        # 先只开探针（open0）→ 量电压 → 关 → 再 attach（open1）→ 才读内存
        self.assertEqual(kinds[:4], ["open0", "voltage", "close", "open1"])
        self.assertLess(kinds.index("voltage"), kinds.index("read"))

    def test_low_voltage_refuses_without_attaching(self):
        for volts, hint in ((0.0, "没上电"), (1.8, "3.3 V")):
            with self.subTest(volts=volts):
                board = self.fake_pyocd([FakeProbe("A1")], Board(volts=volts))
                with self.assertRaises(bridge.BridgeError) as ctx:
                    self.reader().connect()
                self.assertIn(hint, str(ctx.exception))
                self.assertNotIn(("open", True), board.events)
                self.assertEqual(board.target.reads, [])

    def test_voltage_read_failure_or_none_is_an_error_but_no_api_is_allowed(self):
        self.fake_pyocd([FakeProbe("A1")], Board(voltage_error=OSError("usb gone")))
        with self.assertRaises(bridge.BridgeError):
            self.reader().connect()
        self.fake_pyocd([FakeProbe("A1")], Board(volts=None))
        with self.assertRaises(bridge.BridgeError):
            self.reader().connect()
        # DAPLink 经 pyOCD 没有测电压接口：不显示，也不拦
        self.fake_pyocd([FakeProbe("A1", "DAPLink")], Board(voltage_api=False))
        reader = self.reader()
        reader.connect()
        self.assertIsNone(reader.voltage_mv)

    def test_jlink_is_refused_and_ignored_by_auto(self):
        self.fake_pyocd([JLinkProbe("J1", "Segger J-Link")], Board())
        with self.assertRaises(bridge.BridgeError) as ctx:
            self.reader().connect()
        self.assertIn("--imu-jlink", str(ctx.exception))
        with self.assertRaises(bridge.BridgeError):
            self.reader("J1").connect()
        # J-Link 和 ST-Link 同时插着：自动模式只看 ST-Link
        board = self.fake_pyocd([JLinkProbe("J1", "Segger J-Link"), FakeProbe("S1")], Board())
        reader = self.reader()
        reader.connect()
        self.assertEqual(reader.probe_uid, "S1")
        self.assertTrue(all(s.chosen.unique_id == "S1" for s in board.sessions))

    def test_multiple_probes_need_uid_before_opening(self):
        board = self.fake_pyocd([FakeProbe("A1"), FakeProbe("B2", "DAPLink")], Board())
        with self.assertRaises(bridge.BridgeError) as ctx:
            self.reader().connect()
        self.assertIn("A1", str(ctx.exception))
        self.assertIn("B2", str(ctx.exception))
        self.assertEqual(board.sessions, [])

    def test_uid_picks_matching_probe_and_unknown_uid_fails(self):
        self.fake_pyocd([FakeProbe("A1"), FakeProbe("B2", "DAPLink")], Board())
        reader = self.reader("B2")
        reader.connect()
        self.assertEqual(reader.probe_name, "DAPLink")
        with self.assertRaises(bridge.BridgeError):
            self.reader("ZZ").connect()

    def test_no_probe_reports_clearly(self):
        self.fake_pyocd([], Board())
        with self.assertRaises(bridge.BridgeError) as ctx:
            self.reader().connect()
        self.assertIn("没找到调试器", str(ctx.exception))

    def test_probe_busy_and_wiring_failures_have_different_hints(self):
        board = self.fake_pyocd([FakeProbe("A1")], Board(probe_open_error=OSError("busy")))
        with self.assertRaises(bridge.BridgeError) as ctx:
            self.reader().connect()
        self.assertIn("占用", str(ctx.exception))
        self.assertTrue(all(s.closed for s in board.sessions))
        self.fake_pyocd([FakeProbe("A1")], Board(target_open_error=OSError("no ack")))
        with self.assertRaises(bridge.BridgeError) as ctx:
            self.reader().connect()
        self.assertIn("SWDIO", str(ctx.exception))

    def test_flash_mismatch_refuses_before_reading_ram(self):
        board = self.fake_pyocd([FakeProbe("A1")], Board(memory=board_memory(image=bytes(64))))
        with self.assertRaises(bridge.BridgeError):
            self.reader().connect()
        self.assertFalse(any(address == SAMPLE for _, address, _ in board.target.reads))

    def test_halted_cpu_is_not_resumed_unless_opted_in(self):
        board = self.fake_pyocd([FakeProbe("A1")], Board(state=FakeState.HALTED))
        with self.assertRaises(bridge.BridgeError):
            self.reader().connect()
        self.assertEqual(board.target.resumed, 0)
        self.reader(resume=True).connect()
        self.assertEqual(board.target.resumed, 1)

    def test_missing_pyocd_gives_install_hint(self):
        with patch.dict(sys.modules, {"pyocd": None, "pyocd.core": None, "pyocd.core.helpers": None,
                                      "pyocd.core.session": None, "pyocd.core.target": None}):
            with self.assertRaises(bridge.BridgeError) as ctx:
                self.reader().connect()
        self.assertIn("pip install pyocd", str(ctx.exception))


class SwdReadTests(SwdTestCase):
    def connected(self, **memory):
        board = self.fake_pyocd([FakeProbe("A1")], Board(memory=board_memory(**memory)))
        reader = self.reader()
        reader.connect()
        return reader, board

    def test_snapshot_decodes_same_frame_as_jlink_path(self):
        reader, _ = self.connected()
        frame = reader.snapshot()
        self.assertTrue(frame["live"])
        self.assertEqual(frame["age_ms"], 5)
        self.assertEqual(frame["coherence"], "counter_stable")
        self.assertEqual(frame["voltage_mv"], 3290)
        self.assertEqual(frame["quaternion_xyzw"], [0, 0, 0, 1])

    def test_halted_or_unreachable_cpu_stops_with_chinese_error(self):
        reader, board = self.connected()
        board.target.state = FakeState.HALTED
        with self.assertRaises(bridge.BridgeError):
            reader.snapshot()
        board.target.state_error = RuntimeError("Memory transfer fault @0xE000EDF0")
        with self.assertRaises(bridge.BridgeError) as ctx:
            reader.snapshot()
        self.assertIn("CPU 运行状态", str(ctx.exception))

    def test_unaligned_uses_bytes_and_errors_are_wrapped(self):
        reader, board = self.connected()
        self.assertEqual(reader.read(0x08000001, 3), IMAGE[1:4])
        self.assertEqual(board.target.reads[-1][0], "8")
        with self.assertRaises(bridge.BridgeError):
            reader.read(0x30000000, 4)

    def test_close_is_idempotent_and_releases_half_open_probe(self):
        reader, board = self.connected()
        session = reader.session
        reader.close()
        reader.close()
        self.assertEqual(session.closed, 1)
        self.assertIsNone(reader.session)
        # session.close() 没把 USB 放掉（打开到一半失败）时，再兜底关一次探针
        stuck = types.SimpleNamespace(close=lambda: None,
                                      probe=types.SimpleNamespace(is_open=True, closed=0))
        stuck.probe.close = lambda: setattr(stuck.probe, "closed", stuck.probe.closed + 1)
        bridge.SwdReader.close_session(stuck)
        self.assertEqual(stuck.probe.closed, 1)


class BracketedSnapshotTests(unittest.TestCase):
    def test_torn_read_retries_once(self):
        layout = bridge.FirmwareLayout(b"", "test", SAMPLE, TICK)
        counter = struct.pack("<I", 10)
        reads = iter([counter, snapshot(last=106488), struct.pack("<I", 106259), counter,
                      counter, snapshot(last=106490), struct.pack("<I", 106495), counter])
        frame = bridge.bracketed_snapshot(lambda a, n: next(reads), layout, 3300)
        self.assertEqual(frame["read_attempts"], 2)
        self.assertTrue(frame["live"])
        self.assertEqual(frame["voltage_mv"], 3300)


class FakeReader:
    """替换 BridgeService 里的 SwdReader，看重试、关闭和帧字段。"""
    instances = []
    fail_first = 0

    def __init__(self, layout, probe_id, resume):
        self.layout, self.probe_id = layout, probe_id
        self.probe_name, self.probe_uid, self.voltage_mv = "STM32 STLink", "S1", 3300
        self.closed = 0
        FakeReader.instances.append(self)

    def connect(self):
        if len(FakeReader.instances) <= FakeReader.fail_first:
            raise bridge.BridgeError("假的连接失败")

    def read_voltage(self):
        return 3300

    def snapshot(self):
        return bridge.decode_snapshot(snapshot(count=len(FakeReader.instances) * 100 + int(time.monotonic() * 1000) % 97),
                                      1005, voltage_mv=3300)

    def close(self):
        self.closed += 1


class SwdServiceTests(unittest.TestCase):
    def setUp(self):
        FakeReader.instances = []
        layout = bridge.FirmwareLayout(IMAGE, "test", SAMPLE, TICK)
        stack = contextlib.ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(bridge, "SwdReader", FakeReader))
        stack.enter_context(patch.object(bridge, "validate_firmware_layout", return_value=layout))

    def wait_for(self, service, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = service.get_status()
            if predicate(status):
                return status
            time.sleep(0.02)
        self.fail(f"等不到期望的状态：{service.get_status()}")

    def test_failed_connect_closes_reader_and_reports_error(self):
        FakeReader.fail_first = 99
        service = bridge.BridgeService(backend="swd", probe_id="auto", connect_attempts=1)
        service.start()
        self.addCleanup(service.stop)
        status = self.wait_for(service, lambda s: s["status"] == "error")
        self.assertIn("假的连接失败", status["message"])
        self.assertEqual(status["backend"], "swd")
        # _run 在 except 里关一次、finally 里再关一次；close 是幂等的，至少关过就行
        self.assertEqual(len(FakeReader.instances), 1)
        self.assertGreaterEqual(FakeReader.instances[0].closed, 1)

    def test_live_frames_carry_backend_and_probe(self):
        FakeReader.fail_first = 0
        service = bridge.BridgeService(backend="swd", probe_id="auto")
        service.start()
        self.addCleanup(service.stop)
        status = self.wait_for(service, lambda s: s["live"])
        self.assertEqual((status["backend"], status["probe_name"], status["probe_uid"]),
                         ("swd", "STM32 STLink", "S1"))
        self.assertNotIn("probe_serial", status)
        service.stop()
        self.assertEqual(FakeReader.instances[-1].closed, 1)

    def test_swd_service_needs_no_dll_or_serial(self):
        service = bridge.BridgeService(backend="swd", probe_id="auto")
        self.assertEqual(service.backend, "swd")
        self.assertIsNone(service.dll_path)
        self.assertIn("pyOCD", service.frame["message"])

    def test_unknown_backend_rejected(self):
        with self.assertRaises(bridge.BridgeError):
            bridge.BridgeService(backend="uart")


class SwdCliTests(unittest.TestCase):
    def test_flag_forms(self):
        self.assertEqual(server.parse_args(["--imu-swd"]).imu_swd, "auto")
        self.assertEqual(server.parse_args(["--imu-swd", "301612179216303030303032"]).imu_swd,
                         "301612179216303030303032")
        self.assertIsNone(server.parse_args(["--fake"]).imu_swd)

    def test_mutually_exclusive_with_other_imu_sources(self):
        for other in (["--imu-demo"], ["--imu-jlink", "imu.json"]):
            with self.subTest(other=other), contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                server.parse_args(["--imu-swd", *other])

    def test_create_service_checks_pyocd_uid_and_firmware(self):
        args = server.parse_args(["--fake", "--imu-swd"])
        with patch("importlib.util.find_spec", return_value=object()):
            service = server.create_imu_service(args)
            self.assertEqual((service.backend, service.probe_id), ("swd", "auto"))
            self.assertIsNone(service.thread)
            with self.assertRaises(ValueError):          # --imu-swd "" 不能悄悄关掉 IMU
                server.create_imu_service(server.parse_args(["--fake", "--imu-swd", " "]))
            with patch.object(Path, "is_file", return_value=False), self.assertRaises(ValueError):
                server.create_imu_service(args)
        with patch("importlib.util.find_spec", return_value=None), self.assertRaises(ValueError):
            server.create_imu_service(args)

    def test_error_frame_keeps_backend_label(self):
        self.assertEqual(server.imu_error_frame("x", "swd")["backend"], "swd")


if __name__ == "__main__":
    unittest.main()
