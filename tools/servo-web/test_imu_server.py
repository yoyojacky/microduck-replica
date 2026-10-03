"""Optional IMU integration tests; fake servos and synthetic IMU only.

Run: python -m unittest -v test_imu_server
Requires httpx for Starlette's TestClient; never connects to J-Link or serial.
"""
import contextlib
import io
import json
import logging
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from starlette.testclient import TestClient

import imu_bridge
import server


def stub_service():
    service = Mock()
    service.demo = True
    service.get_status.return_value = {
        "type": "imu", "mode": "demo", "status": "live", "live": True,
        "connected": False, "message": "synthetic sample", "sample_count": 7,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    return service


class ServerEnvironment(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.tmp = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.stack.enter_context(patch.multiple(
            server, LOG=[], LOG_N=0, LOG_DIR=str(self.tmp / "logs"),
            CLIENTS=set(), BUS=server.FakeBus([20]), IDS=[20], PRESENT=[20],
            IMU_SERVICE=None, PORT=None, STREAM_HZ=20,
        ))
        self.stack.enter_context(patch.object(server, "list_ports", return_value=[]))

    def collect(self, ws, types, limit=30):
        found = {}
        for _ in range(limit):
            message = ws.receive_json()
            found.setdefault(message["type"], message)
            if types.issubset(found):
                return found
        self.fail(f"Missing messages: {types - found.keys()}")


class WebSocketTests(ServerEnvironment):
    def test_existing_socket_delivers_servo_and_real_demo_bridge_frames(self):
        service = imu_bridge.BridgeService(demo=True)
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client:
                with client.websocket_connect("/ws") as ws:
                    hello = ws.receive_json()
                    self.assertEqual(hello["type"], "hello")
                    self.assertTrue(hello["imu_enabled"])
                    self.assertIn("IMU", hello["cats"])
                    messages = self.collect(ws, {"state", "imu"})
                    self.assertIn("20", messages["state"]["states"])
                    self.assertEqual(messages["imu"]["mode"], "demo")
                    self.assertTrue(messages["imu"]["live"])
                    self.assertEqual(len(messages["imu"]["quaternion_xyzw"]), 4)
                self.assertTrue(service.thread.is_alive())
            self.assertFalse(service.thread.is_alive())
        self.assertFalse(server.CLIENTS)

    def test_disabled_retains_servo_stream_and_does_not_start_bridge(self):
        with patch.object(imu_bridge.BridgeService, "start") as start:
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                hello = ws.receive_json()
                self.assertFalse(hello["imu_enabled"])
                for _ in range(4):
                    self.assertEqual(ws.receive_json()["type"], "state")
            start.assert_not_called()

    def test_multiple_clients_share_one_bridge_and_clean_up_logging(self):
        service = stub_service()
        logger = logging.getLogger("imu_bridge")
        before = (logger.level, logger.propagate, list(logger.handlers))
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client:
                with client.websocket_connect("/ws") as first, client.websocket_connect("/ws") as second:
                    self.assertEqual(first.receive_json()["type"], "hello")
                    self.assertEqual(second.receive_json()["type"], "hello")
                    self.collect(first, {"imu", "state"})
                    self.collect(second, {"imu", "state"})
                    service.start.assert_called_once_with()
                    service.stop.assert_not_called()
                service.stop.assert_not_called()
            service.stop.assert_called_once_with()
        self.assertEqual((logger.level, logger.propagate, list(logger.handlers)), before)

    def test_start_failure_is_an_imu_error_while_servos_keep_streaming(self):
        service = stub_service()
        service.start.side_effect = RuntimeError("synthetic startup failure")
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                messages = self.collect(ws, {"hello", "state", "imu"})
                self.assertEqual(messages["imu"]["status"], "error")
                self.assertFalse(messages["imu"]["live"])
                self.assertIn("synthetic startup failure", messages["imu"]["message"])
                service.get_status.assert_not_called()
            service.stop.assert_called_once_with()

    def test_status_failure_is_isolated_and_can_recover(self):
        service = stub_service()
        frame = service.get_status.return_value
        service.get_status.side_effect = [RuntimeError("synthetic read failure")] * 3 + [frame] * 40
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                messages = self.collect(ws, {"hello", "state", "imu"})
                self.assertEqual(messages["imu"]["status"], "error")
                for _ in range(30):
                    message = ws.receive_json()
                    if message["type"] == "imu" and message["live"]:
                        break
                else:
                    self.fail("IMU did not recover after transient status error")

    def test_servo_timeout_does_not_hold_up_imu_frames(self):
        service = stub_service()
        bus = server.BUS
        states = bus.states

        def slow_states(ids):
            time.sleep(0.3)
            return states(ids)

        with patch.object(server, "IMU_SERVICE", service), patch.object(bus, "states", slow_states):
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                self.assertEqual(ws.receive_json()["type"], "hello")
                before_state = 0
                for _ in range(20):
                    message = ws.receive_json()
                    if message["type"] == "state":
                        break
                    before_state += message["type"] == "imu"
                self.assertGreaterEqual(before_state, 3)

    def test_bridge_and_state_logs_use_existing_imu_category_and_file(self):
        service = stub_service()
        logger = logging.getLogger("imu_bridge")
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                self.collect(ws, {"hello", "imu", "state"})
                logger.warning("synthetic SDK diagnostic")
                for _ in range(8):
                    ws.receive_json()
        text = Path(server.log_path()).read_text(encoding="utf-8")
        self.assertIn("[IMU][警告] synthetic SDK diagnostic", text)
        self.assertEqual(text.count("状态 demo/live"), 1)
        self.assertTrue(any(entry["cat"] == "IMU" and entry["level"] == "warn" for entry in server.LOG))
        self.assertEqual(len(list((self.tmp / "logs").glob("*.log"))), 1)

    def test_no_remote_imu_control_routes_or_handlers(self):
        service = stub_service()
        with patch.object(server, "IMU_SERVICE", service):
            with TestClient(server.app) as client, client.websocket_connect("/ws") as ws:
                ws.receive_json()
                for path in ("/api/reconnect", "/api/disconnect", "/ws/imu"):
                    self.assertEqual(client.post(path).status_code, 404)
                ws.send_json({"op": "imu_reconnect"})
                self.collect(ws, {"state", "imu"})
                service.reconnect.assert_not_called()
                service.disconnect.assert_not_called()


class ConfigTests(ServerEnvironment):
    def config(self, **overrides):
        (self.tmp / "JLink_x64.dll").write_bytes(b"test placeholder, never loaded")
        (self.tmp / "test.hex").write_bytes(b"test placeholder, never flashed")
        config = {"dll": "JLink_x64.dll", "probe_serial": 12345678, "firmware": "test.hex"}
        config.update(overrides)
        path = self.tmp / "imu.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        return server.parse_args(["--imu-jlink", str(path)])

    def test_disabled_needs_no_configuration(self):
        self.assertIsNone(server.create_imu_service(server.parse_args(["--fake"])))

    def test_demo_needs_no_probe_or_dll(self):
        service = server.create_imu_service(server.parse_args(["--fake", "--imu-demo"]))
        self.assertTrue(service.demo)
        self.assertIsNone(service.serial)
        self.assertIsNone(service.dll_path)
        self.assertIsNone(service.thread)

    def test_config_paths_relative_to_config_and_resume_is_opt_in(self):
        service = server.create_imu_service(self.config(hz=12.5))
        self.assertEqual(service.dll_path, self.tmp / "JLink_x64.dll")
        self.assertEqual(service.firmware, self.tmp / "test.hex")
        self.assertEqual(service.serial, 12345678)
        self.assertEqual(service.hz, 12.5)
        self.assertFalse(service.resume)
        self.assertIsNone(service.thread)
        self.assertTrue(server.create_imu_service(self.config(resume=True)).resume)

    def test_default_firmware_is_repository_hex(self):
        args = self.config()
        config = json.loads(args.imu_jlink.read_text(encoding="utf-8"))
        del config["firmware"]
        args.imu_jlink.write_text(json.dumps(config), encoding="utf-8")
        with patch.object(Path, "is_file", return_value=True):
            service = server.create_imu_service(args)
        self.assertEqual(service.firmware, imu_bridge.DEFAULT_FIRMWARE)
        self.assertEqual(service.firmware.suffix, ".hex")

    def test_invalid_fields_rejected_before_probe_start(self):
        cases = [
            {"probe_serial": None}, {"probe_serial": 0}, {"probe_serial": True},
            {"probe_serial": "12345678"}, {"probe_serial": 2**32},
            {"dll": ""}, {"dll": "missing.dll"}, {"firmware": "missing.hex"},
            {"firmware": None}, {"hz": float("nan")}, {"hz": 0}, {"hz": 21},
            {"hz": True}, {"resume": "false"}, {"resume": 1}, {"unexpected": True},
        ]
        with patch.object(imu_bridge.BridgeService, "start") as start:
            for values in cases:
                with self.subTest(values=values), self.assertRaises((ValueError, RuntimeError)):
                    server.create_imu_service(self.config(**values))
            start.assert_not_called()

    def test_unreadable_and_nonobject_config_rejected(self):
        for content in (None, "not json", "[]"):
            path = self.tmp / "bad.json"
            if content is not None:
                path.write_text(content, encoding="utf-8")
            with self.subTest(content=content), self.assertRaises(ValueError):
                server.create_imu_service(server.parse_args(["--imu-jlink", str(path)]))

    def test_demo_and_real_config_are_mutually_exclusive(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            server.parse_args(["--imu-demo", "--imu-jlink", "imu.json"])

    def test_bad_configuration_prevents_server_and_bus_start(self):
        with patch.object(server, "open_bus") as bus, patch.object(server.uvicorn, "run") as run:
            with self.assertRaises(SystemExit):
                server.main(["--port", "COM99", "--imu-jlink", str(self.tmp / "missing.json")])
            bus.assert_not_called()
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
