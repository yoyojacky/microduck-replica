"""Pure decoding/guard tests. These never import or open the J-Link DLL."""
import ctypes as ct
import hashlib
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

import imu_bridge as bridge


def snapshot(*, ready=1, configured=1, error=0, quat=(0x8000, 0, 0), count=10, last=1000):
    data = bytearray(60)
    struct.pack_into("<4B", data, 0, ready, 0x70, configured, error)
    struct.pack_into("<3h", data, 4, 100, -100, 0)
    struct.pack_into("<3h", data, 10, 0, 0, 8197)
    struct.pack_into("<3H", data, 16, *quat)
    struct.pack_into("<9I", data, 24, count, count, count, last, last, 0, 0, 0, 0)
    return bytes(data)


class QuaternionTests(unittest.TestCase):
    def test_negative_zero_identity_is_live(self):
        q = bridge.decode_quaternion((0x8000, 0, 0))
        self.assertEqual(q, [0, 0, 0, 1])
        self.assertLess(math.copysign(1, q[0]), 0)

    def test_positive_zero_marker_is_not_identity(self):
        with self.assertRaises(bridge.BridgeError):
            bridge.decode_quaternion((0, 0, 0))

    def test_nan_infinity_and_invalid_norm_rejected(self):
        for q in ((0x7C00, 0, 0), (0x7E00, 0, 0), (0x3C00, 0x3C00, 0)):
            with self.subTest(q=q), self.assertRaises(bridge.BridgeError):
                bridge.decode_quaternion(q)

    def test_rounding_overshoot_is_normalized(self):
        packed = struct.unpack("<3H", struct.pack("<3e", 0.71, 0.71, 0))
        q = bridge.decode_quaternion(packed)
        self.assertAlmostEqual(sum(v*v for v in q), 1)
        self.assertEqual(q[3], 0)

    def test_half_turn_component_order_xyzw(self):
        self.assertEqual(bridge.decode_quaternion((0x3C00, 0, 0)), [1, 0, 0, 0])


class SnapshotTests(unittest.TestCase):
    def test_units_status_and_counters(self):
        frame = bridge.decode_snapshot(snapshot(), 1005, voltage_mv=3293)
        self.assertTrue(frame["live"])
        self.assertEqual(frame["quaternion_xyzw"], [0, 0, 0, 1])
        self.assertAlmostEqual(frame["gyro_dps"][0], 1.75)
        self.assertAlmostEqual(frame["accel_g"][2], 1.000034)
        self.assertEqual(frame["age_ms"], 5)
        self.assertEqual(frame["sample_count"], 10)
        self.assertEqual(frame["errors"], {"imu": 0, "spi": 0, "fifo": 0, "quaternion": 0, "recovery": 0})

    def test_unready_fault_and_stale_clear_motion_but_keep_diagnostics(self):
        cases = [(snapshot(ready=0), 1001), (snapshot(configured=0), 1001),
                 (snapshot(error=2), 1001), (snapshot(), 1100), (snapshot(quat=(0, 0, 0)), 1001)]
        for raw, tick in cases:
            with self.subTest(tick=tick, raw=raw):
                frame = bridge.decode_snapshot(raw, tick)
                self.assertFalse(frame["live"])
                self.assertIsNone(frame["quaternion_xyzw"])
                self.assertIsNone(frame["accel_g"])
                self.assertEqual(frame["raw_accel"][2], 8197)
                self.assertEqual(frame["sample_count"], 10)

    def test_uint32_tick_wrap(self):
        frame = bridge.decode_snapshot(snapshot(last=0xFFFFFFF8), 3)
        self.assertTrue(frame["live"])
        self.assertEqual(frame["age_ms"], 11)

    def test_future_timestamp_is_inconsistent_not_49_days_old(self):
        # Reproduce the observed byte-torn tick from the hardware log.
        frame = bridge.decode_snapshot(snapshot(last=106488), 106259)
        self.assertFalse(frame["live"])
        self.assertIsNone(frame["quaternion_xyzw"])
        self.assertIsNone(frame["age_ms"])
        self.assertEqual(frame["coherence"], "inconsistent")

    def test_frozen_count_becomes_stale_and_recovers(self):
        tracker = bridge.FreshnessTracker()
        self.assertTrue(tracker.apply(bridge.decode_snapshot(snapshot(), 1001), 10.0)["live"])
        stale = tracker.apply(bridge.decode_snapshot(snapshot(), 1001), 10.5)
        self.assertFalse(stale["live"])
        self.assertIsNone(stale["quaternion_xyzw"])
        self.assertTrue(tracker.apply(bridge.decode_snapshot(snapshot(count=11), 1001), 10.6)["live"])

    def test_stopped_worker_frame_expires(self):
        service = bridge.BridgeService(demo=True)
        service.publish(bridge.decode_snapshot(snapshot(), 1001))
        service.last_publish -= 1
        status = service.get_status()
        self.assertEqual(status["status"], "stale")
        self.assertIsNone(status["quaternion_xyzw"])


class FirmwareGuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.binary = self.root / "test.bin"
        self.map = self.root / "test.map"
        self.binary.write_bytes(b"test-image")
        self.map.write_text(f" sample 0x{bridge.SAMPLE_ADDRESS:08x} Data 60 imu.o(.bss.data)\n"
                            f" tick_ms 0x{bridge.TICK_ADDRESS:08x} Data 4 board.o(.bss.tick_ms)\n")

    def tearDown(self):
        self.temp.cleanup()

    def test_unknown_binary_always_rejected(self):
        with self.assertRaises(bridge.BridgeError):
            bridge.validate_firmware_layout(self.binary, self.map)

    def test_map_guards_address_size_owner_and_unique(self):
        digest = hashlib.sha256(b"test-image").hexdigest()
        original = self.map.read_text()
        with patch.object(bridge, "VALIDATED_SHA256", digest):
            good = bridge.validate_firmware_layout(self.binary, self.map)
            self.assertEqual(good.sample_address, bridge.SAMPLE_ADDRESS)
            moved = f"0x{bridge.SAMPLE_ADDRESS + 4:08x}"
            for bad in (original.replace("Data 60", "Data 64"),
                        original.replace(f"0x{bridge.SAMPLE_ADDRESS:08x}", moved),
                        original.replace("imu.o", "other.o"), original + original, "no symbols"):
                self.map.write_text(bad)
                with self.subTest(map=bad), self.assertRaises(bridge.BridgeError):
                    bridge.validate_firmware_layout(self.binary, self.map)

    def test_current_firmware_validates_without_loading_dll(self):
        layout = bridge.validate_firmware_layout(bridge.DEFAULT_FIRMWARE, bridge.DEFAULT_MAP)
        self.assertEqual(layout.sha256, bridge.VALIDATED_SHA256)
        self.assertEqual(layout.tick_address, 0x20000A34)
        self.assertEqual(layout.sample_address, 0x2000006C)
        self.assertEqual(len(layout.image), 10724)

    @staticmethod
    def record(kind, data=b"", address=0):
        content = bytes([len(data)]) + address.to_bytes(2, "big") + bytes([kind]) + data
        return ":" + (content + bytes([-sum(content) & 0xFF])).hex().upper()

    def test_hex_decodes_exact_flash_bytes_and_checks_digest(self):
        hex_file = self.root / "test.hex"
        records = [self.record(4, b"\x08\x00"), self.record(0, b"test-image"), self.record(1)]
        hex_file.write_text("\n".join(records))
        self.assertEqual(bridge.read_firmware_image(hex_file), b"test-image")
        with self.assertRaises(bridge.BridgeError):
            bridge.validate_firmware_layout(hex_file)
        with patch.object(bridge, "VALIDATED_SHA256", hashlib.sha256(b"test-image").hexdigest()):
            layout = bridge.validate_firmware_layout(hex_file)
            self.assertEqual(layout.image, b"test-image")
            self.assertEqual(layout.sample_address, bridge.SAMPLE_ADDRESS)

    def test_corrupt_or_ambiguous_hex_records_rejected(self):
        extended = self.record(4, b"\x08\x00")
        data = self.record(0, b"abcd")
        end = self.record(1)
        cases = {
            "checksum": [extended, data[:-2] + "00", end],
            "truncated": [extended, data[:-2], end],
            "missing eof": [extended, data],
            "after eof": [extended, data, end, data],
            "overlap": [extended, data, data, end],
            "gap": [extended, data, self.record(0, b"x", 5), end],
            "offset start": [extended, self.record(0, b"x", 1), end],
            "outside flash": [self.record(4, b"\x20\x00"), data, end],
            "unsupported type": [extended, self.record(2, b"\x08\x00"), data, end],
            "empty image": [end],
            "wrong eof shape": [extended, data, self.record(1, b"x")],
            "entry outside flash": [extended, data, self.record(5, b"\x20\x00\x00\x00"), end],
            "nonhex": [extended, ":GG", end],
        }
        hex_file = self.root / "bad.hex"
        for name, records in cases.items():
            hex_file.write_text("\n".join(records))
            with self.subTest(name=name), self.assertRaises(bridge.BridgeError):
                bridge.read_firmware_image(hex_file)


class JLinkConnectionGuardTests(unittest.TestCase):
    """Exercise the actual connect path against a fake DLL, not a real probe."""
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.dll_path = Path(self.temp.name) / "fake-JLink_x64.dll"
        self.dll_path.write_bytes(b"test fixture, never loaded")
        self.events = []
        self.image = b"verified-test-image"
        self.returned_image = self.image
        self.restart_on_close = True
        self.work_ram_init = True
        self.reject_command = None
        self.lib = Mock()
        self.lib.JLINKARM_ExecCommand.side_effect = self.command
        self.lib.JLINKARM_EMU_SelectByUSBSN.return_value = 0
        self.lib.JLINKARM_OpenEx.side_effect = self.open_probe
        self.lib.JLINKARM_GetHWStatus.side_effect = self.voltage
        self.lib.JLINKARM_TIF_Select.return_value = 0
        self.lib.JLINKARM_IsConnected.side_effect = [0, 1]
        self.lib.JLINKARM_Connect.side_effect = self.connect_target
        self.lib.JLINKARM_ReadMemEx.side_effect = self.read_memory
        self.lib.JLINKARM_IsHalted.return_value = 0
        self.lib.JLINKARM_Close.side_effect = self.close_probe
        layout = bridge.FirmwareLayout(self.image, "test", 0x20000064, 0x20000BE0)
        self.reader = bridge.JLinkReader(layout, self.dll_path, 12345678, resume=True)

    def command(self, raw, error, capacity):
        command = raw.decode("ascii")
        self.events.append(command)
        if command == self.reject_command:
            return -1
        if command == "Device = STM32G031F8":
            # Simulate device selection installing its work-RAM defaults.
            self.work_ram_init = True
        elif command == "SetInitWorkRAMOnConnect = 0":
            self.work_ram_init = False
        elif command == "SetRestartOnClose = 0":
            self.restart_on_close = False
        return 0

    def open_probe(self, log, error):
        self.events.append("Open")
        # Simulate opening a fresh DLL session restoring a default option.
        self.restart_on_close = True
        return None

    def voltage(self, ptr):
        ct.cast(ptr, ct.POINTER(bridge.HardwareStatus)).contents.VTarget = 3293
        return 0

    def connect_target(self):
        self.events.append("Connect")
        self.assertFalse(self.work_ram_init)
        self.assertFalse(self.restart_on_close)
        return 0

    def read_memory(self, address, size, buffer, width):
        self.assertEqual(address, 0x08000000)
        self.assertEqual(width, 4 if size % 4 == 0 else 1)
        ct.memmove(buffer, self.returned_image, size)
        return size

    def close_probe(self):
        self.events.append("Close")
        self.assertFalse(self.restart_on_close, "SDK Close would resume an unverified target")

    def connect(self):
        with patch.object(bridge.ct, "CDLL", return_value=self.lib), \
                patch.object(bridge.os, "add_dll_directory", return_value=Mock(), create=True), \
                patch.object(bridge.os, "name", "nt"):
            self.reader.connect()

    def test_controls_precede_target_connect_and_survive_session_defaults(self):
        self.connect()
        self.reader.close()
        self.assertLess(self.events.index("SetRestartOnClose = 0"), self.events.index("Open"))
        self.assertLess(self.events.index("Device = STM32G031F8"), self.events.index("SetInitWorkRAMOnConnect = 0"))
        self.assertLess(self.events.index("SetInitWorkRAMOnConnect = 0"), self.events.index("Connect"))
        self.lib.JLINKARM_Go.assert_not_called()

    def test_rejected_work_ram_guard_aborts_before_target_connect(self):
        self.reject_command = "SetInitWorkRAMOnConnect = 0"
        with self.assertRaises(bridge.BridgeError):
            self.connect()
        self.reader.close()
        self.lib.JLINKARM_Connect.assert_not_called()
        self.lib.JLINKARM_ReadMemEx.assert_not_called()
        self.lib.JLINKARM_Go.assert_not_called()

    def test_rejected_close_guard_aborts_before_open(self):
        self.reject_command = "SetRestartOnClose = 0"
        with self.assertRaises(bridge.BridgeError):
            self.connect()
        self.reader.close()
        self.lib.JLINKARM_OpenEx.assert_not_called()
        self.lib.JLINKARM_Connect.assert_not_called()
        self.lib.JLINKARM_Close.assert_not_called()

    def test_mismatched_firmware_does_not_resume_explicitly_or_on_close(self):
        self.returned_image = b"X" * len(self.image)
        self.lib.JLINKARM_IsHalted.return_value = 1
        with self.assertRaises(bridge.BridgeError):
            self.connect()
        self.reader.close()
        self.lib.JLINKARM_Go.assert_not_called()
        self.assertEqual(self.events[-1], "Close")

    def test_only_verified_halted_cpu_with_resume_can_go(self):
        self.lib.JLINKARM_IsHalted.side_effect = [1, 0]
        self.connect()
        self.reader.close()
        self.lib.JLINKARM_Go.assert_called_once()


class JLinkMemoryReadTests(unittest.TestCase):
    def setUp(self):
        layout = bridge.FirmwareLayout(b"test", "test", 0x20000064, 0x20000BE0)
        self.reader = bridge.JLinkReader(layout, Path("unused-test-JLink.dll"), 12345678)
        self.reader.lib = Mock()
        self.reader.lib.JLINKARM_IsHalted.return_value = 0

    def test_aligned_reads_use_words_preserve_byte_counts_and_data(self):
        for address, size in ((0x08000000, 11612), (0x20000064, 60), (0x20000BE0, 4)):
            expected = bytes(index % 251 for index in range(size))

            def read(address_arg, byte_count, buffer, width):
                self.assertEqual((address_arg, byte_count, width), (address, size, 4))
                self.assertEqual(ct.addressof(buffer) % 4, 0)
                ct.memmove(buffer, expected, size)
                return size

            self.reader.lib.JLINKARM_ReadMemEx.side_effect = read
            with self.subTest(address=address, size=size):
                self.assertEqual(self.reader.read(address, size), expected)

    def test_unaligned_address_or_length_uses_bytes(self):
        for address, size in ((0x20000065, 4), (0x20000064, 3), (0x20000066, 2)):
            def read(address_arg, byte_count, buffer, width):
                self.assertEqual((address_arg, byte_count, width), (address, size, 1))
                ct.memmove(buffer, b"abcd", size)
                return size

            self.reader.lib.JLINKARM_ReadMemEx.side_effect = read
            with self.subTest(address=address, size=size):
                self.assertEqual(self.reader.read(address, size), b"abcd"[:size])

    def test_partial_memory_read_is_not_accepted(self):
        self.reader.lib.JLINKARM_ReadMemEx.return_value = 3
        with self.assertRaises(bridge.BridgeError):
            self.reader.read(0x20000BE0, 4)

    def test_inconsistent_snapshot_retries_once_and_recovers(self):
        counter = struct.pack("<I", 10)
        self.reader.read = Mock(side_effect=[counter, snapshot(last=106488), struct.pack("<I", 106259), counter,
                                            counter, snapshot(last=106490), struct.pack("<I", 106495), counter])
        frame = self.reader.snapshot()
        self.assertTrue(frame["live"])
        self.assertEqual(frame["age_ms"], 5)
        self.assertEqual(frame["read_attempts"], 2)
        self.assertEqual(self.reader.read.call_count, 8)

    def test_persistent_inconsistency_is_bounded_and_never_live(self):
        counter = struct.pack("<I", 10)
        self.reader.read = Mock(side_effect=[counter, snapshot(last=106488), struct.pack("<I", 106259), counter] * 2)
        frame = self.reader.snapshot()
        self.assertFalse(frame["live"])
        self.assertIsNone(frame["age_ms"])
        self.assertEqual(frame["coherence"], "inconsistent")
        self.assertEqual(self.reader.read.call_count, 8)


class ExplicitProbeConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.dll_path = Path(self.temp.name) / "fake-JLink_x64.dll"
        self.dll_path.write_bytes(b"test fixture, never loaded")

    def test_live_service_requires_explicit_serial_and_existing_dll(self):
        for serial, dll_path in ((None, None), (0, self.dll_path), (-1, self.dll_path),
                                 (True, self.dll_path), (2**32, self.dll_path),
                                 ("12345678", self.dll_path), (12345678, None),
                                 (12345678, self.dll_path.parent / "missing.dll")):
            with self.subTest(serial=serial, dll_path=dll_path), patch.object(bridge.ct, "CDLL") as load:
                with self.assertRaises(bridge.BridgeError):
                    bridge.BridgeService(serial=serial, dll_path=dll_path)
                load.assert_not_called()

    def test_reader_rejects_zero_before_opening_dll(self):
        layout = bridge.FirmwareLayout(b"test", "test", 0x20000064, 0x20000BE0)
        reader = bridge.JLinkReader(layout, self.dll_path, 0)
        with patch.object(bridge.ct, "CDLL") as load, self.assertRaises(bridge.BridgeError):
            reader.connect()
        load.assert_not_called()

    def test_demo_needs_no_probe_configuration_or_dll_loading(self):
        with patch.object(bridge.ct, "CDLL") as load:
            service = bridge.BridgeService(demo=True)
            self.assertIsNone(service.serial)
            self.assertIsNone(service.dll_path)
            load.assert_not_called()



if __name__ == "__main__":
    unittest.main()
