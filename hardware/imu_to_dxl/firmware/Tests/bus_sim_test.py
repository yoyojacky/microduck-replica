# -*- coding: utf-8 -*-
"""真固件协议代码上模拟总线：把 protocol.c + control_table.c 编成 DLL（Tests/bus_sim_shim.c），
挂到协议级舵机模拟器上，用主控侧的真实工具去测它：

- tools/imu200/imu200.py 的 check()：总线协议.md §11 的验收，判据照主控源码
- 调试台 servo-web 的 --imu-bus 轮询：跟主控同一条 sync_read，200 排第一
- 排队规则：200 排在中间、前面有设备不在线（§5）

总线上的时间是虚拟微秒：主控的包和别的舵机的应答都按 1 Mbps（10 µs/字节）喂给固件，
固件答不答、什么时候答，全由 C 代码自己决定。舵机仍用 sim_bus 的模拟。不接硬件。

  python Tests/bus_sim_test.py            # cl.exe 在 PATH 里，或自动用 vswhere 找 Visual Studio
"""
import ctypes
import math
import os
import shutil
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(FW, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "servo-web"))
sys.path.insert(0, os.path.join(REPO, "tools", "imu200"))
import feetech          # noqa: E402
import imu200           # noqa: E402
from sim_bus import SimSerial   # noqa: E402

JOINTS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]
BYTE_US = 10            # 1 Mbps
SERVO_TRESP_US = 30     # 模拟舵机的应答延时（真值待量，§12）
SOURCES = ["Tests/bus_sim_shim.c", "Core/Src/protocol.c", "Core/Src/control_table.c"]


def build_dll():
    out_dir = os.path.join(FW, "Build", "host-tests")
    os.makedirs(out_dir, exist_ok=True)
    dll = os.path.join(out_dir, "bus_sim_shim.dll")
    srcs = [os.path.join(FW, s) for s in SOURCES] + [os.path.join(FW, "Core", "Inc", h)
                                                     for h in ("protocol.h", "control_table.h", "imu.h")]
    if os.path.exists(dll) and os.path.getmtime(dll) > max(os.path.getmtime(s) for s in srcs):
        return dll
    cmd = (f'cl /nologo /LD /std:c11 /W4 /WX /Od /I Core\\Inc /FoBuild\\host-tests\\ '
           f'/Fe:Build\\host-tests\\bus_sim_shim.dll ' + " ".join(s.replace("/", "\\") for s in SOURCES))
    if not shutil.which("cl"):
        vswhere = r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
        root = subprocess.run([vswhere, "-latest", "-products", "*", "-requires",
                               "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property",
                               "installationPath"], capture_output=True, text=True).stdout.strip()
        vcvars = os.path.join(root, "VC", "Auxiliary", "Build", "vcvars64.bat")
        cmd = f'call "{vcvars}" >nul 2>nul && {cmd}'
    # 整条命令作为一个字符串交给 cmd /s /c：列表形式会被 Python 按 C 运行时规则转义引号，cmd 看不懂
    r = subprocess.run(f'cmd /d /s /c "{cmd}"', cwd=FW, capture_output=True, text=True,
                       encoding="mbcs", errors="replace")
    if r.returncode:
        raise SystemExit("编译 DLL 失败：\n" + r.stdout + r.stderr)
    return dll


def f16(x):
    return struct.unpack("<H", struct.pack("<e", x))[0]


class FirmwareOnBus(SimSerial):
    """sim_bus 的假串口，但 ID 200 由真固件协议代码来答。"""

    def __init__(self, dll, joints, ready_ms=100):
        super().__init__(joints)
        self.fw = dll
        self.fw.shim_init()
        self.us = 1000
        self.last_sync_us = -20000
        self.ready_ms = ready_ms
        self.fw_delays = []          # 固件应答相对"轮到它"的延时（µs）
        self.collisions = []         # 不该它说话的时候它发了：(时刻, 字节)
        self.sync_bus_us = []        # 每条 sync_read 在虚拟总线上占了多久（指令开始 → 最后一个应答结束）

    # ---- 固件这一侧的时间和 IMU 数据 ----
    def _imu_now(self):
        t = self.us / 1e6
        ms = self.us // 1000
        self.fw.shim_set_millis(ms)
        samples = int(t * 120)
        if ms < self.ready_ms:
            self.fw.shim_set_snapshot(0, 1, 0, 0, 0, 0, 0, 0, samples)
            return
        jitter = samples % 7 - 3            # 真芯片静置时低位也在跳
        yaw = 0.3 * math.sin(t)
        self.fw.shim_set_snapshot(1, 0, int(100 * math.sin(t)) + jitter, -50 + jitter, 3 - jitter,
                                  f16(0.05), f16(0.0), f16(math.sin(yaw / 2)), samples)

    def _idle(self, duration_us):
        """总线归别人（或空着）的这段时间也逐微秒 poll 固件：这时候它敢发就是抢话 / 撞包。"""
        buf = (ctypes.c_uint8 * 300)()
        for _ in range(duration_us):
            self.us += 1
            self.fw.shim_poll(self.us)
            n = self.fw.shim_take_tx(buf, 300)
            if n > 0:
                self.collisions.append((self.us, bytes(buf[:n]).hex(" ")))

    def _heard(self, data, gap_us):
        """总线上出现一串字节（主控的包或别的舵机的应答），固件都听得见；字节之间照样 poll。"""
        self._idle(gap_us)
        for i, b in enumerate(data):
            if i:
                self._idle(BYTE_US - 1)
                self.us += 1
            self.fw.shim_feed(b, self.us)

    def _fw_turn(self, limit_us=6000):
        """轮到固件（或它该不答）：逐微秒 poll，看它在什么时候发、发了什么。"""
        start = self.us
        buf = (ctypes.c_uint8 * 300)()
        while self.us - start < limit_us:
            self.us += 1
            self.fw.shim_poll(self.us)
            n = self.fw.shim_take_tx(buf, 300)
            if n > 0:
                self.fw_delays.append(self.us - start)
                data = bytes(buf[:n])
                self.us += BYTE_US * n        # 自己发的这一串占着总线，但自己听不见（RX_EN 关了回显）
                return data
            if self.fw.shim_pending() == 0 and self.us - start > 200:
                return None                   # 没打算答
        return None

    def _servo_reply(self, s, params):
        s.tick(self.clock)
        body = bytes([s.id, len(params) + 2, s.r[65]]) + bytes(params)
        return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])

    def write(self, data):
        data = bytes(data)
        while len(data) >= 6:
            if data[0] != 0xFF or data[1] != 0xFF:
                data = data[1:]
                continue
            length = data[3]
            pkt, data = data[:4 + length], data[4 + length:]
            self._packet(pkt)
        return len(data)

    def _packet(self, pkt):
        sid, instr, params = pkt[2], pkt[4], pkt[5:-1]
        self._advance()
        # 主控的节奏：sync_read 每 20 ms 一条（50 Hz），别的包之间 2 ms。不跟墙钟走 ——
        # Windows 的 sleep 粒度 15.6 ms，墙钟会让虚拟时间忽长忽短，计数步进就不像真机了。
        if instr == feetech.SYNC_READ:
            self.us = max(self.us + 2000, self.last_sync_us + 20000)
            self.last_sync_us = self.us
        else:
            self.us += 2000
        self._imu_now()
        self._heard(pkt, 0)
        if instr == feetech.SYNC_READ and sid == feetech.BROADCAST:
            t0 = self.us - BYTE_US * (len(pkt) - 1)
            addr, ln, ids = params[0], params[1], params[2:]
            for i in ids:
                if i == 200:
                    r = self._fw_turn()
                    if r:
                        self.rx += r
                    continue
                s = self._by_id(i)
                if s and addr not in getattr(s, "muted", ()):
                    reply = self._servo_reply(s, s.read(addr, ln, self.clock))
                    self.rx += reply
                    self._heard(reply, SERVO_TRESP_US)
                else:
                    self._idle(300)               # 不在线的舵机：总线空着
            self.sync_bus_us.append(self.us - t0)
            if 200 not in ids:
                self._idle(3000)                  # 没问它：这段时间它也不许开口
            return
        if sid == 200 or sid == feetech.BROADCAST:
            r = self._fw_turn()
            if r:
                self.rx += r
            if sid == 200:
                return
        before = len(self.rx)
        self._handle(sid, instr, params)          # 发给舵机的：sim_bus 照常处理
        reply = bytes(self.rx[before:])
        if reply:
            self._heard(reply, SERVO_TRESP_US)

    def counters(self):
        out = (ctypes.c_uint32 * 7)()
        self.fw.shim_counters(out)
        return dict(zip(("rx_packets", "tx_packets", "crc", "malformed", "timeouts", "cancelled", "skipped"), out))


def load():
    lib = ctypes.CDLL(build_dll())
    lib.shim_feed.argtypes = [ctypes.c_uint8, ctypes.c_uint32]
    lib.shim_poll.argtypes = [ctypes.c_uint32]
    lib.shim_set_millis.argtypes = [ctypes.c_uint32]
    lib.shim_take_tx.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.shim_set_snapshot.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int16, ctypes.c_int16, ctypes.c_int16,
                                      ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint32]
    lib.shim_counters.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
    return lib


def main():
    lib = load()
    results = []

    def ok(cond, what):
        results.append((bool(cond), what))
        print(("  ✓ " if cond else "  ✗ ") + what)

    print("== 1. imu200.py check：主控判据的验收，对象是真固件协议代码 ==")
    port = FirmwareOnBus(lib, JOINTS)
    bus = feetech.FeetechBus(None, ser=port)
    imu200.log = lambda *a, **k: None          # 只看结果，别刷屏
    for good, what in imu200.check(bus, frames=80, hz=50):
        if what.startswith("事务耗时"):
            # check 量的是 Python 墙钟；这里每微秒都要调一次 DLL，墙钟不代表总线。改用虚拟总线时间判
            print(f"  · check: {what}（模拟里是 Python 墙钟，不计）")
            continue
        ok(good, "check: " + what)
    c = port.counters()
    ok(c["crc"] == 0 and c["malformed"] == 0 and c["timeouts"] == 0, f"固件侧没有校验错/坏包/残帧：{c}")
    delays = sorted(port.fw_delays)
    ok(delays and delays[0] >= 50 and delays[-1] <= 52,
       f"T_resp（协议逻辑）：轮到固件后 {delays[0]}–{delays[-1]} µs 开始发（规格 50 µs；CPU 耗时要逻辑分析仪实测）")
    ok(not port.collisions, f"别的设备说话、总线空着时固件从不开口（抢话 {len(port.collisions)} 次：{port.collisions[:2]}）")
    full = sorted(t for t in port.sync_bus_us if t > 2000)   # 16 个设备的那条
    p99 = full[max(0, int(len(full) * 0.99) - 1)] / 1000
    ok(full and p99 < 6.0, f"16 个设备的 sync_read 在虚拟总线上 p99 {p99:.2f} ms（预算 6 ms；模拟舵机 T_resp 取 {SERVO_TRESP_US} µs）")

    print("== 2. 版本区（§9）==")
    _, head = bus.read(200, 0, 9)
    ok(list(head) == [0, 2, 0, 0x44, 0x4D, 200, 0, 0, 1], f"0–8 = {list(head)}（版本 0.2、END 0、型号、ID 200、1 Mbps、级别 1）")

    print("== 3. 排队（§5）：200 排在中间 / 前面的设备不在线 ==")
    port = FirmwareOnBus(lib, JOINTS, ready_ms=0)
    bus = feetech.FeetechBus(None, ser=port)
    got = bus.sync_read([20, 21, 200, 22], 56, 15)
    ok(all(got.get(i) for i in (20, 21, 200, 22)), "200 排第 3：四个都答了，没撞包、没错位")
    port.servos[21].muted.add(56)                 # 21 对这个地址不应答 = 前面有设备不在线
    # 直接看总线上的原始帧：feetech.py 按顺序读，缺一个就「错位后面全废」（规格 §2），那是主控侧的读法
    port.reset_input_buffer()
    bus._send(feetech.BROADCAST, feetech.SYNC_READ, bytes([56, 15, 20, 21, 200, 22]))
    raw, frames = bytes(port.rx), []
    while len(raw) >= 6 and raw[:2] == b"\xff\xff":
        frames.append(raw[2])
        raw = raw[4 + raw[3]:]
    ok(frames == [20, 200, 22], f"21 不在线：总线上的应答顺序是 {frames}（200 等一个空档后照样答，22 也答了）")
    ok(port.counters()["skipped"] == 1, f"固件记了 1 次跳过不在线的设备（{port.counters()['skipped']}）")
    got = bus.sync_read([20, 21, 22], 56, 15)
    ok(200 not in got and not port.collisions,
       f"列表里没有 200：舵机应答期间和之后 3 ms 固件都没开口（抢话 {len(port.collisions)} 次）")

    print("== 4. 调试台 --imu-bus：跟主控同一条 sync_read ==")
    import imu_bus
    import server
    port = FirmwareOnBus(lib, JOINTS, ready_ms=0)
    svc = imu_bus.BusImuService()
    server.BUS = feetech.FeetechBus(None, ser=port)
    server.log = lambda *a, **k: None
    for _ in range(5):
        st = server.read_states_with_imu(JOINTS, svc)
        time.sleep(0.02)
    status = svc.get_status()
    ok(status["live"] and all(st[i] for i in JOINTS), f"姿态 live，15 颗舵机状态都在：{status['message']}")
    ok(not port.collisions, f"调试台轮询期间固件没抢话（{len(port.collisions)} 次）")

    bad = [w for g, w in results if not g]
    print(f"\n{len(results) - len(bad)}/{len(results)} 通过" + ("" if not bad else "；失败：\n  " + "\n  ".join(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
