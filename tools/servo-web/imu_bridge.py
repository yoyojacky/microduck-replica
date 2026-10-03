"""STM32 IMU observation through the locally installed J-Link DLL.

The application makes no flash/program, Reset, Halt, memory-write, or probe
update API calls. It requests that the SDK disable work-RAM initialization on
connect and implicit Go on close. This is not a guarantee of zero target side effects:
device-specific SDK connection/debug initialization may still alter state.
Validate behavior against the actual DLL/target logs. The config's resume flag opts
in to Go() only after firmware verification if the CPU is halted. All DLL calls
belong to one worker thread.

SwdReader is the same observation through pyOCD, for ST-Link or CMSIS-DAP
(DAPLink) probes: check target voltage, attach without reset/halt, verify the
board Flash against the HEX, then read the same RAM snapshot. J-Link probes are
refused there (use JLinkReader), see the SwdReader docstring.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import hashlib
import logging
import math
import os
from pathlib import Path
import re
import struct
import threading
import time
from typing import Any

LOGGER = logging.getLogger("imu_bridge")
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FIRMWARE = PROJECT_ROOT / "hardware/imu_to_dxl/firmware/Build/imu_to_dxl.hex"
DEFAULT_MAP = None  # Optional locally generated map, never required by the checkout.
VALIDATED_SHA256 = "66bc7c532f5a1ad87e3dfff71a4b51f8c4f20840a8f62436874fa5b7c665938e"
# These symbols were reviewed against the map for the exact image above:
# firmware 0.2.0 (Feetech protocol), Keil MDK 5.41 / Arm Compiler 6.22, 10724 bytes.
# (0.1.0, Dynamixel 2.0: 69bdaea7..., sample 0x20000064, tick 0x20000BE0.)
# A different build must be reviewed before adding a new digest/layout pair.
SAMPLE_ADDRESS = 0x2000006C
TICK_ADDRESS = 0x20000A34
SNAPSHOT_SIZE = 60
SENSOR_STALE_MS = 100
HOST_STALE_MS = 350
SWD_FREQUENCY_HZ = 100_000  # 与 J-Link 路径相同：首板 4 MHz 下载出过传输失败


class BridgeError(RuntimeError):
    pass


def validate_probe_settings(dll_path: Path | None, serial: int | None) -> tuple[Path, int]:
    """Require an explicit probe identity and installed DLL before hardware access."""
    if isinstance(serial, bool) or not isinstance(serial, int) or not 1 <= serial <= 0xFFFFFFFF:
        raise BridgeError("真板模式必须填写有效的 --probe-serial / probe_serial（J-Link 正整数序列号，不能为 0）。")
    if dll_path is None:
        raise BridgeError("真板模式必须填写 --dll / dll，指向本机安装的 JLink_x64.dll。")
    path = Path(dll_path).expanduser().resolve()
    if not path.is_file():
        raise BridgeError(f"J-Link DLL 文件不存在：{path}；请填写实际安装路径。")
    return path, serial


@dataclass(frozen=True)
class FirmwareLayout:
    image: bytes
    sha256: str
    sample_address: int
    tick_address: int


def read_firmware_image(firmware: Path) -> bytes:
    """Decode the committed Intel HEX without a BIN/MAP or extra dependency.

    Accept only contiguous STM32G031 main-flash data with checked record
    checksums, lengths, addresses and EOF. Never silently fill holes or accept
    overlapping records. The SHA check below still pins every decoded byte.
    """
    firmware = Path(firmware)
    if firmware.suffix.lower() != ".hex":
        return firmware.read_bytes()  # Optional local BIN, still digest-checked.
    cells = {}
    upper = 0
    eof = False
    try:
        lines = firmware.read_text(encoding="ascii").splitlines()
        for number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            if eof or not line.startswith(":"):
                raise ValueError(f"unexpected record at line {number}")
            record = bytes.fromhex(line[1:])
            if len(record) < 5 or len(record) != record[0] + 5 or sum(record) & 0xFF:
                raise ValueError(f"record length/checksum at line {number}")
            size, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
            payload = record[4:-1]
            if kind == 0:
                start = upper + address
                if not size or address + size > 0x10000 or not 0x08000000 <= start < start + size <= 0x08010000:
                    raise ValueError(f"data outside main flash at line {number}")
                for offset, value in enumerate(payload):
                    if start + offset in cells:
                        raise ValueError(f"overlapping data at line {number}")
                    cells[start + offset] = value
            elif kind == 1 and size == 0 and address == 0:
                eof = True
            elif kind == 4 and size == 2 and address == 0:
                upper = int.from_bytes(payload, "big") << 16
            elif kind == 5 and size == 4 and address == 0:
                if not 0x08000000 <= int.from_bytes(payload, "big") < 0x08010000:
                    raise ValueError("entry point outside main flash")
            else:
                raise ValueError(f"unsupported HEX record at line {number}")
        if not eof or not cells or min(cells) != 0x08000000 or len(cells) != max(cells) - 0x08000000 + 1:
            raise ValueError("missing EOF or non-contiguous flash image")
        return bytes(cells[address] for address in range(0x08000000, max(cells) + 1))
    except (ValueError, UnicodeError) as exc:
        raise BridgeError(f"固件 HEX 无效：{exc}") from exc


def validate_firmware_layout(binary: Path, map_file: Path | None = None) -> FirmwareLayout:
    """Tie the RAM schema to the exact validated executable, never just a name."""
    image = read_firmware_image(binary)
    digest = hashlib.sha256(image).hexdigest()
    if digest != VALIDATED_SHA256:
        raise BridgeError("固件文件与已验证版本不同：需先核对 RAM 结构及地址，禁止猜测读取。")
    if map_file is None:
        return FirmwareLayout(image, digest, SAMPLE_ADDRESS, TICK_ADDRESS)
    text = map_file.read_text(encoding="utf-8", errors="replace")

    def symbol(name: str, size: int, address: int, object_name: str) -> int:
        matches = re.findall(
            rf"^\s*{re.escape(name)}\s+0x([0-9a-fA-F]+)\s+Data\s+(\d+)\s+([^\r\n]+)",
            text, re.MULTILINE,
        )
        if len(matches) != 1:
            raise BridgeError(f"链接映射中的 {name} 缺失或不唯一。")
        found, found_size, owner = matches[0]
        value = int(found, 16)
        if int(found_size) != size or value != address or not owner.startswith(object_name + "("):
            raise BridgeError(f"{name} 的地址、大小或所属模块与已验证固件不匹配。")
        if not 0x20000000 <= value < value + size <= 0x20002000:
            raise BridgeError(f"{name} 超出 STM32G031 的 SRAM 范围。")
        return value

    sample = symbol("sample", SNAPSHOT_SIZE, SAMPLE_ADDRESS, "imu.o")
    tick = symbol("tick_ms", 4, TICK_ADDRESS, "board.o")
    return FirmwareLayout(image, digest, sample, tick)


def decode_quaternion(packed: tuple[int, int, int]) -> list[float]:
    """ST SFLP stores binary16 xyz; w is reconstructed nonnegative.

    0000/0000/0000 is the firmware's unavailable marker. 8000/0000/0000
    (IEEE negative zero) is a valid identity rotation, not the marker.
    """
    if not any(packed):
        raise BridgeError("四元数为未就绪标记。")
    xyz = struct.unpack("<3e", struct.pack("<3H", *packed))
    if not all(math.isfinite(value) for value in xyz):
        raise BridgeError("四元数包含 NaN 或无穷值。")
    squared = sum(value * value for value in xyz)
    if squared > 1.02:
        raise BridgeError("四元数超出 SFLP 有效范围。")
    values = [*xyz, math.sqrt(max(0.0, 1.0 - squared))]
    norm = math.sqrt(sum(value * value for value in values))
    return [value / norm for value in values]


def empty_frame(mode: str, status: str, message: str) -> dict[str, Any]:
    return {
        "type": "imu", "service": "microduck-imu-viewer", "mode": mode, "status": status,
        "connected": False, "live": False, "quaternion_xyzw": None,
        "accel_g": None, "gyro_dps": None, "sample_count": None,
        "gyro_count": None, "accel_count": None, "ready": 0,
        "configured": 0, "who_am_i": None, "error": None,
        "errors": {key: None for key in ("imu", "spi", "fifo", "quaternion", "recovery")},
        "voltage_mv": None, "age_ms": None, "hz": 0.0,
        "coherence": "best_effort", "message": message,
        "timestamp_ms": int(time.time() * 1000),
    }


def decode_snapshot(raw: bytes, tick_ms: int, *, voltage_mv: int | None = None,
                    coherence: str = "best_effort") -> dict[str, Any]:
    if len(raw) != SNAPSHOT_SIZE:
        raise BridgeError(f"快照长度错误：{len(raw)}，预期 {SNAPSHOT_SIZE}。")
    ready, who_am_i, configured, error = raw[:4]
    gyro = struct.unpack_from("<3h", raw, 4)
    accel = struct.unpack_from("<3h", raw, 10)
    packed = struct.unpack_from("<3H", raw, 16)
    sample_count, gyro_count, accel_count, last_gyro, last_quat, spi, fifo, invalid, recovery = struct.unpack_from("<9I", raw, 24)
    gyro_age = (tick_ms - last_gyro) & 0xFFFFFFFF
    quat_age = (tick_ms - last_quat) & 0xFFFFFFFF
    # A modular difference in the upper half of uint32 means the captured
    # sensor timestamp is ahead of tick, not that this sample is 49 days old.
    # Normal timer wrap (e.g. last=0xfffffff8, tick=3) still yields +11 ms.
    inconsistent_time = gyro_age >= 0x80000000 or quat_age >= 0x80000000
    age = None if inconsistent_time else max(gyro_age, quat_age)
    frame = empty_frame("live", "stale", "等待 IMU 有效采样。")
    frame.update({
        "connected": True, "ready": ready, "who_am_i": who_am_i,
        "configured": configured, "error": error,
        "sample_count": sample_count, "gyro_count": gyro_count, "accel_count": accel_count,
        "errors": {"imu": error, "spi": spi, "fifo": fifo, "quaternion": invalid, "recovery": recovery},
        "age_ms": age, "voltage_mv": voltage_mv, "coherence": coherence,
        "tick_ms": tick_ms, "last_gyro_ms": last_gyro, "last_quat_ms": last_quat,
        "raw_accel": list(accel), "raw_gyro": list(gyro), "quat_fp16": list(packed),
    })
    if inconsistent_time:
        frame.update(coherence="inconsistent", message="快照时间戳不一致，暂停姿态并等待重新读取。")
    elif who_am_i != 0x70 or ready not in (0, 1) or configured not in (0, 1):
        frame.update(status="error", message="IMU 标识或状态异常，停止显示姿态。")
    elif error:
        frame.update(status="error", message=f"IMU 报告错误码 {error}。")
    elif not ready or not configured or not sample_count:
        frame["message"] = "IMU 尚未就绪，等待初始化和有效采样。"
    elif age >= SENSOR_STALE_MS:
        frame["message"] = f"传感器采样已过期（{age} ms），姿态已暂停。"
    else:
        try:
            quaternion = decode_quaternion(packed)
        except BridgeError as exc:
            frame.update(status="error", message=str(exc))
        else:
            frame.update(
                status="live", live=True, quaternion_xyzw=quaternion,
                accel_g=[value * 0.000122 for value in accel],
                gyro_dps=[value * 0.0175 for value in gyro],
                message="正在读取真实板卡；姿态可跟随板子运动。",
            )
    return frame


class FreshnessTracker:
    """Detect a frozen CPU even if both RAM timestamps freeze together."""
    def __init__(self) -> None:
        self.count: int | None = None
        self.changed_at: float | None = None

    def apply(self, frame: dict[str, Any], now: float) -> dict[str, Any]:
        if frame["sample_count"] != self.count:
            self.count = frame["sample_count"]
            self.changed_at = now
        elif self.changed_at is not None and (now - self.changed_at) * 1000 >= HOST_STALE_MS:
            frame.update(status="stale", live=False, quaternion_xyzw=None, accel_g=None,
                         gyro_dps=None, message="采样计数停止增长，姿态已暂停。")
        return frame


class HardwareStatus(ct.Structure):
    _fields_ = [("VTarget", ct.c_uint16)] + [(name, ct.c_uint8) for name in ("tck", "tdi", "tdo", "tms", "tres", "trst")]


class JLinkReader:
    def __init__(self, layout: FirmwareLayout, dll_path: Path,
                 serial: int, resume: bool = False) -> None:
        self.layout, self.dll_path, self.serial, self.resume = layout, dll_path, serial, resume
        self.lib: Any = None
        self.opened = False
        self.dll_directory: Any = None
        self.voltage_mv: int | None = None
        self.callback: Any = None

    def command(self, command: str) -> None:
        error = ct.create_string_buffer(1024)
        rc = self.lib.JLINKARM_ExecCommand(command.encode("ascii"), error, len(error))
        if rc < 0 or error.value:
            raise BridgeError(f"J-Link 设置失败 {command}: {rc} {error.value.decode(errors='replace')}")

    def connect(self) -> None:
        self.dll_path, self.serial = validate_probe_settings(self.dll_path, self.serial)
        if os.name != "nt" or ct.sizeof(ct.c_void_p) != 8:
            raise BridgeError("实时模式需要 Windows 64 位 Python 和本机 J-Link 驱动。")
        self.dll_directory = os.add_dll_directory(str(self.dll_path.parent))
        self.lib = ct.CDLL(str(self.dll_path))
        callback_type = ct.CFUNCTYPE(None, ct.c_char_p)
        self.callback = callback_type(lambda msg: LOGGER.info("J-Link: %s", msg.decode(errors="replace")) if msg else None)
        declarations = {
            "ExecCommand": ([ct.c_char_p, ct.POINTER(ct.c_char), ct.c_int], ct.c_int),
            "EMU_SelectByUSBSN": ([ct.c_uint32], ct.c_int),
            "OpenEx": ([callback_type, callback_type], ct.c_char_p),
            "GetHWStatus": ([ct.POINTER(HardwareStatus)], ct.c_int),
            "TIF_Select": ([ct.c_int], ct.c_int), "SetSpeed": ([ct.c_uint32], None),
            "Connect": ([], ct.c_int), "IsConnected": ([], ct.c_int),
            "IsHalted": ([], ct.c_int), "Go": ([], None), "Close": ([], None),
            "ReadMemEx": ([ct.c_uint32, ct.c_uint32, ct.c_void_p, ct.c_uint32], ct.c_int),
        }
        for name, (args, result) in declarations.items():
            method = getattr(self.lib, "JLINKARM_" + name)
            method.argtypes, method.restype = args, result
        for command in ("DisableAutoUpdateFW", "SuppressInfoUpdateFW", "SuppressGUI = 1",
                        "SetRestartOnClose = 0"):
            self.command(command)
        if self.lib.JLINKARM_EMU_SelectByUSBSN(self.serial) < 0:
            raise BridgeError("未找到指定 J-Link；检查 USB，并让 Keil 退出调试。")
        error = self.lib.JLINKARM_OpenEx(self.callback, self.callback)
        if error:
            raise BridgeError("J-Link 打开失败；请让 Keil 退出调试、关闭其他探针工具。" + error.decode(errors="replace"))
        self.opened = True
        # SDK Close defaults to a Go-equivalent resume. Configure before Open
        # (so unsupported options fail before acquiring a probe) and repeat
        # after Open in case it restored session defaults. This also protects
        # cleanup when subsequent connection/firmware verification fails.
        # https://kb.segger.com/J-Link_Command_Strings#SetRestartOnClose
        self.command("SetRestartOnClose = 0")
        self.read_voltage()
        for command in ("DisableFlashDL", "DisableFlashBPs", "InhibitConnectRetries = 1"):
            self.command(command)
        if self.lib.JLINKARM_TIF_Select(1) != 0:
            raise BridgeError("无法选择 SWD 接口。")
        self.lib.JLINKARM_SetSpeed(100)
        self.command("Device = STM32G031F8")
        # Apply after device selection, before target connect. Firmware
        # verification happens later and cannot undo SDK RAM initialization.
        # Both settings are mandatory: command() aborts on rejection.
        # https://kb.segger.com/J-Link_Command_Strings#SetInitWorkRAMOnConnect
        self.command("SetInitWorkRAMOnConnect = 0")
        self.command("SetRestartOnClose = 0")
        connected = self.lib.JLINKARM_IsConnected()
        if connected < 0:
            raise BridgeError("无法读取目标连接状态。")
        if connected == 0 and self.lib.JLINKARM_Connect() < 0:
            raise BridgeError("SWD 连接失败，检查 SWDIO、SWCLK、GND、VTref 接线。")
        if self.lib.JLINKARM_IsConnected() != 1:
            raise BridgeError("J-Link 未连接到目标芯片。")
        # Read from the actual target; this never programs the supplied image.
        self.command("InvalidateCache")
        actual = self.read(0x08000000, len(self.layout.image))
        if actual != self.layout.image:
            raise BridgeError("板上固件与已验证文件不一致，拒绝继续读取 RAM 或恢复执行。")
        halted = self.lib.JLINKARM_IsHalted()
        if halted not in (0, 1):
            raise BridgeError("无法读取 CPU 运行状态，未尝试恢复执行。")
        if halted == 1:
            if not self.resume:
                raise BridgeError("目标 CPU 已暂停。固件已验证；如需恢复运行，在 IMU 配置中设置 resume: true 后重启服务。")
            self.lib.JLINKARM_Go()
            if self.lib.JLINKARM_IsHalted() != 0:
                raise BridgeError("恢复运行失败；检查 Keil 残留断点。")

    def read_voltage(self) -> int:
        status = HardwareStatus()
        if self.lib.JLINKARM_GetHWStatus(ct.byref(status)) != 0:
            raise BridgeError("无法读取探针参考电压，连接可能已断开。")
        self.voltage_mv = int(status.VTarget)
        if not 3000 <= self.voltage_mv <= 3500:
            raise BridgeError(f"目标参考电压为 {self.voltage_mv} mV，不在已验证的 3.3 V 范围内。")
        return self.voltage_mv

    def read(self, address: int, size: int) -> bytes:
        if address < 0 or size <= 0 or address + size > 0x100000000:
            raise BridgeError("读取地址或长度超出有效范围。")
        # SEGGER UM08002, JLINKARM_ReadMemEx: AccessWidth=4 forces U32
        # accesses, 1 forces U8; NumBytes remains a byte count. Use aligned
        # words so an updating 32-bit tick/counter is not assembled from four
        # separate byte reads. This does NOT make the entire snapshot atomic.
        width = 4 if address % 4 == 0 and size % 4 == 0 else 1
        buf = (ct.c_uint32 * (size // 4))() if width == 4 else (ct.c_uint8 * size)()
        count = self.lib.JLINKARM_ReadMemEx(address, size, buf, width)
        if count != size:
            raise BridgeError(f"读取失败 0x{address:08X}：{count}/{size} 字节。请检查接线并重新启动连接。")
        return bytes(buf)

    def snapshot(self) -> dict[str, Any]:
        if self.lib.JLINKARM_IsHalted() != 0:
            raise BridgeError("目标 CPU 已暂停；未自动恢复，姿态显示已停止。")
        # No explicit halt per frame. Equal bracketing counters improve detection
        # of torn reads, but are not a seqlock: telemetry explicitly says so.
        return bracketed_snapshot(self.read, self.layout, self.voltage_mv)

    def close(self) -> None:
        if self.opened:
            self.lib.JLINKARM_Close()
            self.opened = False
        if self.dll_directory is not None:
            self.dll_directory.close()
            self.dll_directory = None


def bracketed_snapshot(read, layout: FirmwareLayout, voltage_mv: int | None) -> dict[str, Any]:
    """与 JLinkReader.snapshot 同一套读法：快照前后各读一次采样计数，相等才算没撕裂。"""
    address = layout.sample_address
    for attempt in range(2):
        before = struct.unpack("<I", read(address + 24, 4))[0]
        raw = read(address, SNAPSHOT_SIZE)
        tick = struct.unpack("<I", read(layout.tick_address, 4))[0]
        after = struct.unpack("<I", read(address + 24, 4))[0]
        inner = struct.unpack_from("<I", raw, 24)[0]
        coherence = "counter_stable" if before == inner == after else "best_effort"
        frame = decode_snapshot(raw, tick, voltage_mv=voltage_mv, coherence=coherence)
        frame["read_attempts"] = attempt + 1
        if frame["coherence"] != "inconsistent":
            break
    return frame


class SwdReader:
    """pyOCD 观察：ST-Link / CMSIS-DAP(DAPLink)。J-Link 不走这里，用 --imu-jlink。

    不烧写、不写 RAM、不复位、不暂停：connect_mode=attach，resume_on_disconnect=False
    （断开时不隐式 Go），no_config=True（不加载当前目录的 pyocd.yaml / 用户脚本）。
    target_override=cortex_m：按通用 Cortex-M 连，只读内存，不需要芯片包和 Flash 算法。
    跟 JLinkReader 一样先逐字节比对板上 Flash 与 HEX，不一致就不读 RAM。

    这不等于对目标零影响：attach 仍会写调试寄存器（DHCSR.C_DEBUGEN、DEMCR.TRCENA、
    FPB/DWT 使能），断开后 C_DEBUGEN 保持置位 —— 这期间固件里的 BKPT 会停住 CPU 而不是进 HardFault。
    实测（ST-LINK/V2，100 kHz）连续连接不会让板子重启。

    J-Link 经 pyOCD 会默认从 19 脚供 5 V（jlink.power）、可能静默升级探针固件、
    关闭时不保证不 Go，绕开了 JLinkReader 的保护，所以这里直接拒绝。
    """

    def __init__(self, layout: FirmwareLayout, probe_id: str | None = None, resume: bool = False) -> None:
        self.layout, self.probe_id, self.resume = layout, probe_id, resume
        self.session: Any = None
        self.target: Any = None
        self.halted_state: Any = None
        self.voltage_mv: int | None = None
        self.probe_name: str | None = None
        self.probe_uid: str | None = None

    @staticmethod
    def options() -> dict[str, Any]:
        return {"target_override": "cortex_m", "connect_mode": "attach", "frequency": SWD_FREQUENCY_HZ,
                "resume_on_disconnect": False, "auto_unlock": False, "no_config": True}

    def choose_probe(self, probes: list[Any]) -> Any:
        def is_jlink(p: Any) -> bool:
            return type(p).__name__ == "JLinkProbe"
        found = "；".join(f"{p.description} {p.unique_id}" for p in probes)
        jlink_hint = "J-Link 请改用 --imu-jlink（走 SEGGER 官方 DLL，带不供电、不升级固件、不隐式恢复运行的保护）。"
        if not probes:
            raise BridgeError("没找到调试器（ST-Link / DAPLink）；检查 USB 和驱动，"
                              "并关掉 Keil 调试、CubeProgrammer 等占用调试器的程序。")
        if self.probe_id is None or self.probe_id == "auto":
            usable = [p for p in probes if not is_jlink(p)]
            if not usable:
                raise BridgeError("只找到 J-Link。" + jlink_hint)
            if len(usable) > 1:
                raise BridgeError(f"插了 {len(usable)} 个调试器，用 --imu-swd <UID> 指定一个：{found}")
            return usable[0]
        matches = [p for p in probes if p.unique_id == self.probe_id]
        if not matches:
            raise BridgeError(f"没找到 UID 为 {self.probe_id} 的调试器；现在插着的：{found}")
        if is_jlink(matches[0]):
            raise BridgeError("指定的是 J-Link。" + jlink_hint)
        return matches[0]

    def connect(self) -> None:
        try:
            from pyocd.core.helpers import ConnectHelper
            from pyocd.core.session import Session
            from pyocd.core.target import Target
        except ImportError as exc:
            raise BridgeError("SWD 模式需要 pyOCD：pip install pyocd") from exc
        self.halted_state = Target.State.HALTED
        chosen = self.choose_probe(list(ConnectHelper.get_all_connected_probes(blocking=False)))
        self.probe_name, self.probe_uid = chosen.description, chosen.unique_id
        LOGGER.info("SWD 探针：%s %s（%s）", self.probe_name, self.probe_uid, type(chosen).__name__)

        # 第 1 步：只打开调试器、不碰芯片，先看目标电压（JLinkReader 也是连芯片前查电压）。
        probe_only = Session(chosen, options=self.options())
        try:
            probe_only.open(init_board=False)
        except Exception as exc:
            self.close_session(probe_only)
            raise BridgeError(f"打开调试器失败（{type(exc).__name__}: {exc}）：可能被 Keil 调试、"
                              "CubeProgrammer 占用，或 ST-Link 固件太旧（用 CubeProgrammer 升级）。") from exc
        try:
            self.check_voltage(probe_only.probe)
        finally:
            self.close_session(probe_only)

        # 第 2 步：attach 到芯片。
        self.session = Session(chosen, options=self.options())
        try:
            self.session.open()
        except Exception as exc:
            raise BridgeError(f"SWD 连接芯片失败（{type(exc).__name__}: {exc}）："
                              "检查 SWDIO、SWCLK、GND、VTref 接线和板子供电。") from exc
        self.target = self.session.target
        # 第 3 步：只读板上 Flash 做比对，从不烧写。
        actual = self.read(0x08000000, len(self.layout.image))
        if actual != self.layout.image:
            raise BridgeError("板上固件与已验证文件不一致，拒绝继续读取 RAM 或恢复执行。")
        if self.is_halted():
            if not self.resume:
                raise BridgeError("目标 CPU 已暂停。固件已验证；如需恢复运行，退出 Keil 调试后重新上电。")
            try:
                self.target.resume()
            except Exception as exc:
                raise BridgeError(f"恢复运行失败（{type(exc).__name__}: {exc}）。") from exc
            if self.is_halted():
                raise BridgeError("恢复运行失败；检查 Keil 残留断点。")

    def check_voltage(self, probe: Any) -> int | None:
        # pyOCD 里只有 ST-Link 能测目标电压（私有接口 _link）。没有这个接口（DAPLink）就不显示、不拦；
        # 有接口但读失败或读不到，按连接异常处理，不悄悄放过。
        link = getattr(probe, "_link", None)
        getter = getattr(link, "get_target_voltage", None)
        if getter is None:
            self.voltage_mv = None
            return None
        try:
            getter()
            volts = link.target_voltage
        except Exception as exc:
            raise BridgeError(f"读目标电压失败（{type(exc).__name__}: {exc}），调试器可能已断开。") from exc
        if not isinstance(volts, (int, float)) or not math.isfinite(volts):
            raise BridgeError("读不到目标电压：检查 J3.6 是否接到调试器的 VTref/TVCC。")
        self.voltage_mv = int(round(volts * 1000))
        if self.voltage_mv < 500:
            raise BridgeError(f"目标电压只有 {self.voltage_mv} mV：J3.6 没接到 VTref/TVCC，或者板子没上电。")
        if not 3000 <= self.voltage_mv <= 3500:
            raise BridgeError(f"目标参考电压为 {self.voltage_mv} mV，不在已验证的 3.3 V 范围内。")
        return self.voltage_mv

    def read_voltage(self) -> int | None:
        return self.check_voltage(getattr(self.session, "probe", None))

    def is_halted(self) -> bool:
        try:
            return self.target.get_state() == self.halted_state
        except Exception as exc:
            raise BridgeError(f"读 CPU 运行状态失败（{type(exc).__name__}: {exc}）："
                              "板子可能掉电了，或 SWD 线松了。") from exc

    def read(self, address: int, size: int) -> bytes:
        if address < 0 or size <= 0 or address + size > 0x100000000:
            raise BridgeError("读取地址或长度超出有效范围。")
        try:
            # 对齐时按 32 位字读，免得正在更新的计数被拆成 4 次字节读拼出来。
            if address % 4 == 0 and size % 4 == 0:
                words = self.target.read_memory_block32(address, size // 4)
                data = struct.pack(f"<{len(words)}I", *words)
            else:
                data = bytes(self.target.read_memory_block8(address, size))
        except Exception as exc:
            raise BridgeError(f"读取失败 0x{address:08X}（{type(exc).__name__}: {exc}）。"
                              "请检查接线并重新启动连接。") from exc
        if len(data) != size:
            raise BridgeError(f"读取失败 0x{address:08X}：{len(data)}/{size} 字节。")
        return data

    def snapshot(self) -> dict[str, Any]:
        if self.is_halted():
            raise BridgeError("目标 CPU 已暂停；未自动恢复，姿态显示已停止。")
        return bracketed_snapshot(self.read, self.layout, self.voltage_mv)

    @staticmethod
    def close_session(session: Any) -> None:
        try:
            session.close()
        except Exception as exc:
            LOGGER.warning("关闭调试器连接出错：%s: %s", type(exc).__name__, exc)
        # 打开到一半失败时 session.close() 可能认为探针没开，USB 却已被占住，这里再兜底关一次。
        probe = getattr(session, "probe", None)
        if probe is not None and getattr(probe, "is_open", False):
            try:
                probe.close()
            except Exception as exc:
                LOGGER.warning("关闭调试器 USB 出错：%s: %s", type(exc).__name__, exc)

    def close(self) -> None:
        session, self.session, self.target = self.session, None, None
        if session is not None:
            self.close_session(session)


class BridgeService:
    def __init__(self, *, demo: bool = False, resume: bool = False,
                 firmware: Path = DEFAULT_FIRMWARE, map_file: Path | None = DEFAULT_MAP,
                 dll_path: Path | None = None, serial: int | None = None,
                 hz: float = 20.0, connect_attempts: int = 3,
                 backend: str = "jlink", probe_id: str | None = None) -> None:
        if backend not in ("jlink", "swd"):
            raise BridgeError(f"未知探针后端：{backend}")
        if not demo and backend == "jlink":
            dll_path, serial = validate_probe_settings(dll_path, serial)
        self.demo, self.resume, self.backend, self.probe_id = demo, resume, backend, probe_id
        self.firmware, self.map_file, self.dll_path = firmware, map_file, dll_path
        self.serial, self.hz, self.connect_attempts = serial, max(1.0, min(hz, 20.0)), connect_attempts
        self.lock = threading.Lock()
        self.lifecycle_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.last_publish = time.monotonic()
        connecting = "正在连接 J-Link 并校验固件。" if backend == "jlink" else "正在连接调试器（pyOCD）并校验固件。"
        self.frame = empty_frame("demo" if demo else "live", "disconnected", "正在启动演示。" if demo else connecting)

    def publish(self, frame: dict[str, Any]) -> None:
        frame["timestamp_ms"] = int(time.time() * 1000)
        frame.setdefault("backend", self.backend)  # 连接中的帧也带上，网页标签不会先显示成 J-Link
        with self.lock:
            self.frame = frame
            self.last_publish = time.monotonic()

    def get_status(self) -> dict[str, Any]:
        with self.lock:
            result = dict(self.frame)
            elapsed_ms = (time.monotonic() - self.last_publish) * 1000
        if result["live"] and elapsed_ms >= HOST_STALE_MS:
            result.update(status="stale", live=False, quaternion_xyzw=None, accel_g=None,
                          gyro_dps=None, hz=0.0, message="数据流已超时，姿态显示暂停。")
        return result

    def start(self) -> None:
        with self.lifecycle_lock:
            if self.thread is not None and self.thread.is_alive():
                raise BridgeError("一个服务只允许一个探针读取线程。")
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._run, name="imu-probe-owner", daemon=True)
            self.thread.start()

    def stop(self) -> None:
        with self.lifecycle_lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                raise BridgeError("探针调用仍在退出，禁止启动第二个连接；请稍后重试。")

    def disconnect(self) -> None:
        self.stop()
        self.publish(empty_frame("demo" if self.demo else "live", "disconnected", "连接已释放；可以返回 Keil 调试，或重新启动服务连接。"))

    def reconnect(self) -> None:
        with self.lifecycle_lock:
            self._stop_locked()
            self.stop_event.clear()
            self.publish(empty_frame("demo" if self.demo else "live", "disconnected", "正在重新连接。"))
            self.thread = threading.Thread(target=self._run, name="imu-probe-owner", daemon=True)
            self.thread.start()

    def _run_demo(self) -> None:
        started = time.monotonic()
        while not self.stop_event.is_set():
            elapsed = time.monotonic() - started
            x, y, z = (math.sin(elapsed * factor) * amplitude for factor, amplitude in ((0.7, 0.23), (0.9, 0.3), (0.4, 0.25)))
            frame = empty_frame("demo", "live", "演示数据：没有连接或读取真实板卡。")
            frame.update(connected=False, live=True, ready=1, configured=1, error=0,
                         quaternion_xyzw=[x, y, z, math.sqrt(1 - x*x - y*y - z*z)],
                         accel_g=[0.0, 0.0, 1.0], gyro_dps=[0.0, 0.0, 0.0],
                         sample_count=int(elapsed * 120), gyro_count=int(elapsed * 120),
                         accel_count=int(elapsed * 120), age_ms=0, hz=self.hz, coherence="demo",
                         errors={key: 0 for key in ("imu", "spi", "fifo", "quaternion", "recovery")})
            self.publish(frame)
            self.stop_event.wait(1 / self.hz)

    def _run(self) -> None:
        if self.demo:
            self._run_demo()
            return
        reader: JLinkReader | SwdReader | None = None
        try:
            layout = validate_firmware_layout(self.firmware, self.map_file)
            for attempt in range(self.connect_attempts):
                reader = (SwdReader(layout, self.probe_id, self.resume) if self.backend == "swd"
                          else JLinkReader(layout, self.dll_path, self.serial, self.resume))
                try:
                    reader.connect()
                    break
                except Exception as exc:
                    reader.close()
                    if attempt + 1 >= self.connect_attempts:
                        raise
                    frame = empty_frame("live", "disconnected", f"连接尝试 {attempt + 1}/{self.connect_attempts}：{exc}")
                    self.publish(frame)
                    if self.stop_event.wait(3):
                        return
            tracker = FreshnessTracker()
            previous: float | None = None
            smooth_hz = 0.0
            next_voltage_read = time.monotonic() + 2
            while not self.stop_event.is_set():
                started = time.monotonic()
                if started >= next_voltage_read:
                    reader.read_voltage()
                    next_voltage_read = started + 2
                frame = tracker.apply(reader.snapshot(), time.monotonic())
                now = time.monotonic()
                if previous is not None:
                    instant = 1.0 / max(0.0001, now - previous)
                    smooth_hz = instant if smooth_hz == 0 else smooth_hz * 0.8 + instant * 0.2
                previous = now
                frame.update(hz=round(smooth_hz, 1), firmware_sha256=layout.sha256,
                             sample_address=f"0x{layout.sample_address:08X}", backend=self.backend)
                if isinstance(reader, SwdReader):
                    frame.update(probe_name=reader.probe_name, probe_uid=reader.probe_uid)
                else:
                    frame.update(probe_serial=self.serial)
                self.publish(frame)
                self.stop_event.wait(max(0.0, 1 / self.hz - (time.monotonic() - started)))
        except Exception as exc:
            LOGGER.exception("IMU connection stopped")
            frame = empty_frame("live", "error", str(exc) + " 确认接线和 Keil 已退出调试后，重新启动本工具。")
            self.publish(frame)
        finally:
            if reader is not None:
                reader.close()
