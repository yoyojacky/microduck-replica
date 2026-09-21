# -*- coding: utf-8 -*-
"""ID 200 IMU 块：编解码 + 假小板（模拟器）+ 真板子一致性检查。

`imu_to_dxl` 小板要在舵机总线上冒充一颗 ID 200 的舵机，主控每 tick 的那条 sync_read
（地址 56、长度 15）把它跟 15 颗舵机一次读回来。协议全文见
[`hardware/imu_to_dxl/总线协议.md`](../../hardware/imu_to_dxl/总线协议.md)，这个文件是它的**可执行版本**：

  python imu200.py demo                  # 假小板 + 真主控代码，在内存里跑一遍
  python imu200.py vectors               # 打印测试向量（十六进制），写固件时对着抄
  python imu200.py check --port COM6     # 拿真板子跑一致性检查，逐条 PASS/FAIL

判据全部照着主控的真实代码写（fork 的 `duck-control/src/imu.rs` 和 `bus.rs`），不是照着协议文字写，
所以这里过了 = 主控会认。日志跟仓库约定一致：`logs/imu200-日期.log`（含原始收发十六进制），
每帧另落一行 JSONL，出问题直接读文件。
"""
import argparse
import json
import math
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "servo-web"))
import feetech  # noqa: E402  仓库里的飞特协议实现，不重复造

IMU_ID = 200
BLOCK_ADDR = 56           # 跟舵机同一个地址段，ID 200 才能挤进同一条 sync_read
BLOCK_LEN = 15            # 主控实际用前 12 字节（imu.rs: IMU_BLOCK_LEN = 12），后 3 字节凑满舵机的块长
IMU_WINDOW = 12           # 主控看的窗口，也是 StaleImuTracker 逐字节比较的范围
GYRO_LSB_DPS = 0.0175     # ±500 dps 量程，17.5 mdps/LSB（imu.rs: GYRO_RAD_PER_LSB）
GYRO_MAX_LSB = 28571      # ±500 dps 满量程对应的 LSB，超了说明固件量程配错
READY_FRAMES = 25         # 主控累计（不是连续）25 帧有效四元数才算 ready（imu.rs: quat_samples）
NORM_SQ_MAX = 1.02        # imu.rs 的判据：半精度在满量程的舍入余量
STALE_RUN_MAX = 25        # bus.rs: StaleImuTracker 连续 25 帧 12 字节完全相同 → 报 frozen
TICK_BUDGET_MS = 6.0      # 架构文档 §5 第 4 项：一条 sync_read 读 16 个设备的预算
MOUNT = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)   # imu.rs: DEFAULT_MOUNT，绕 Y +90°

JOINT_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]
FULL_IDS = [IMU_ID] + JOINT_IDS        # 主控每 tick 发的就是这个列表，200 在最前


# ---- 日志：格式跟 servo-web 一致，debug 只进文件 ----
LOG_DIR = os.path.join(HERE, "logs")
_log_file = None


def log(msg, cat="总线", level="info"):
    global _log_file
    tag = {"info": "", "warn": "[警告]", "error": "[错误]", "debug": "[调试]"}[level]
    line = f"{time.strftime('%H:%M:%S')} [{cat}]{tag} {msg}"
    try:
        if _log_file is None:
            os.makedirs(LOG_DIR, exist_ok=True)
            _log_file = open(os.path.join(LOG_DIR, f"imu200-{time.strftime('%Y-%m-%d')}.log"),
                             "a", encoding="utf-8")
        _log_file.write(line + "\n")
        _log_file.flush()
    except OSError:
        pass
    if level != "debug":
        print(line)


def attach_trace(bus):
    """把原始收发接进日志。注意要清空 trace_skip：默认它把 SYNC_READ 滤掉了，
    而我们要调的正是那一条。"""
    bus.trace_skip = set()
    bus.trace = lambda s: log(s, "总线", "debug")


# ---- 半精度：固件侧多半没有现成的，按 IEEE 754 binary16 走，方便对着抄 ----
def f32_to_f16(x):
    return struct.unpack("<H", struct.pack("<e", x))[0]


def f16_to_f32(h):
    return struct.unpack("<e", struct.pack("<H", h))[0]


def encode_block(gyro_dps=(0.0, 0.0, 0.0), quat_xyz=(0.0, 0.0, 0.0), counter=0, status=0):
    """15 字节：gyro x/y/z（i16 小端，17.5 mdps/LSB）+ quat x/y/z（half）+ 采样计数 + 状态 + 保留。

    陀螺和四元数都按主控的判据校验：生成一个主控必然拒绝的块却不报错，是这种工具最坏的失败方式。
    """
    raw = [int(round(v / GYRO_LSB_DPS)) for v in gyro_dps]
    if any(abs(v) > GYRO_MAX_LSB for v in raw):
        raise ValueError(f"陀螺超量程：{gyro_dps} dps，±500 dps 对应 ±{GYRO_MAX_LSB} LSB")
    norm_sq = sum(v * v for v in quat_xyz)
    if not all(math.isfinite(v) for v in quat_xyz) or norm_sq > NORM_SQ_MAX:
        raise ValueError(f"四元数主控会拒绝：{quat_xyz}，norm² = {norm_sq:.3f} > {NORM_SQ_MAX}")
    return (struct.pack("<hhh", *raw)
            + struct.pack("<HHH", *(f32_to_f16(v) for v in quat_xyz))
            + bytes([counter & 0xFF, status & 0xFF, 0]))


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def decode_block(b):
    """解 15 字节，判据逐条照抄 imu.rs：

    - 四元数三分量全 0 → 融合还没出数，主控沿用上一帧、不计入 ready
    - 任一分量不是有限数，或 norm² > 1.02 → 主控**拒绝**这一帧（工具必须一样严，
      不然坏固件在这里全绿、装上真机 imu_ready() 永远 false）
    - w = √(1 − norm²)，再乘 mount 的共轭，norm ≤ 0.5 也拒绝
    """
    if len(b) != BLOCK_LEN:
        raise ValueError(f"要 {BLOCK_LEN} 字节，给了 {len(b)}")
    gx, gy, gz = struct.unpack_from("<hhh", b, 0)
    packed = struct.unpack_from("<HHH", b, 6)
    qx, qy, qz = (f16_to_f32(h) for h in packed)
    finite = all(math.isfinite(v) for v in (qx, qy, qz))
    norm_sq = (qx * qx + qy * qy + qz * qz) if finite else float("inf")
    accepted, quat, gravity = False, None, None
    if any(packed) and finite and norm_sq <= NORM_SQ_MAX:
        w = math.sqrt(max(0.0, 1.0 - norm_sq))
        q = _qmul((w, qx, qy, qz), (MOUNT[0], -MOUNT[1], -MOUNT[2], -MOUNT[3]))
        n = math.sqrt(sum(v * v for v in q))
        if n > 0.5:
            accepted = True
            quat = tuple(v / n for v in q)
            # gravity = 躯干系下的重力方向，直立时 [0, 0, -1]（imu.rs 同款）
            w0, x0, y0, z0 = quat
            gravity = (-2 * (x0 * z0 - w0 * y0), -2 * (y0 * z0 + w0 * x0),
                       -(1 - 2 * (x0 * x0 + y0 * y0)))
    return {
        "gyro_raw": (gx, gy, gz),
        "gyro_dps": (gx * GYRO_LSB_DPS, gy * GYRO_LSB_DPS, gz * GYRO_LSB_DPS),
        "quat_packed": packed,
        "quat_xyz": (qx, qy, qz),
        "quat_trunk": quat,
        "gravity": gravity,
        "norm_sq": norm_sq,
        "finite": finite,
        "counter": b[12],
        "status": b[13],
        "reserved": b[14],
        "accepted": accepted,                  # 主控会不会把这一帧计进 ready
        "window_hex": b[:IMU_WINDOW].hex(),     # StaleImuTracker 比的就是这 12 字节
    }


class SimImu200:
    """假小板：塞进 sim_bus.SimSerial 就能被真的主控代码读到。

    按协议文件的行为写：
    - 块由 IMU 的数据就绪驱动（默认 120 Hz），**不是被读一次才更新一次** —— 计数器是采样计数，
      主控 50 Hz 读的话步进是 2~3。做成"读一次加一"会掩盖"IMU 死了但总线处理器还活着"这种故障
    - 总线处理器比 IMU 先起来：bus_up_ms 之后能应答，imu_ready_ms 之后才出四元数，中间回全 0
    - 陀螺带噪声：真芯片静置时低位也在跳，块逐字节都在变。固件要是做了平滑导致 12 字节不变，
      主控 0.5 秒就报 frozen
    - answer=False 装成不应答的板子（真机后果：整条 sync_read 失败，15 颗舵机数据一起丢）
    """

    def __init__(self, sid=IMU_ID, sample_hz=120, bus_up_ms=0, imu_ready_ms=500,
                 answer=True, noise=True):
        self.r = bytearray(feetech.DUMP_END)
        self.r[0], self.r[1] = 0, 1           # 固件版本 0.1，别冒充舵机的 3.46
        self.r[2], self.r[5], self.r[6], self.r[8] = 0, sid, 0, 1   # END 小端、ID、1 Mbps、应答级别 1
        self.sample_hz, self.bus_up_ms, self.imu_ready_ms = sample_hz, bus_up_ms, imu_ready_ms
        self.answer, self.noise = answer, noise
        self.counter, self.samples, self.last_ms = 0, 0, 0.0
        self.muted = set()
        self.block = encode_block(counter=0, status=0x01)

    @property
    def id(self):
        return self.r[5]

    def tick(self, clock_ms=0.0):
        """按虚拟时钟推进采样：每 1/sample_hz 秒换一块新数据，跟被读几次无关。"""
        while clock_ms - self.last_ms >= 1000.0 / self.sample_hz:
            self.last_ms += 1000.0 / self.sample_hz
            self.samples += 1
            self.counter = (self.counter + 1) & 0xFF
            t = self.last_ms / 1000.0
            n = (self.samples % 7 - 3) * 0.02 if self.noise else 0.0   # 低位抖动，块才会逐字节变
            if self.last_ms < self.imu_ready_ms:
                self.block = encode_block(gyro_dps=(0, 0, 0), quat_xyz=(0, 0, 0),
                                          counter=self.counter, status=0x01)
            else:
                yaw = 0.3 * math.sin(t)
                self.block = encode_block(
                    gyro_dps=(5.0 * math.sin(t) + n, -2.0 + n, 0.5 + n),
                    quat_xyz=(0.0, 0.0, math.sin(yaw / 2)),
                    counter=self.counter, status=0x00)

    def alive(self, clock_ms):
        return self.answer and clock_ms >= self.bus_up_ms

    def read(self, addr, n, clock_ms=0.0):
        self.tick(clock_ms)
        if addr == BLOCK_ADDR and n == BLOCK_LEN:
            return self.block
        return bytes(self.r[addr:min(addr + n, feetech.DUMP_END)])

    def write(self, addr, data):
        pass          # 协议：主控从不写这块板；写了照样 ACK，但什么都不改


# ---- 主控侧：一致性检查 ----
def check(bus, sid=IMU_ID, frames=200, hz=50, full_bus=True, frame_log=None):
    """拿主控会发的包去问板子，逐条判。返回 [(通过?, 说明), ...]。"""
    out = []

    def ok(cond, what, level=None):
        out.append((bool(cond), what))
        log(("✓ " if cond else "✗ ") + what, "总线", level or ("info" if cond else "warn"))
        return cond

    # 1. 上电：ping 最多等 5 秒，记下第一次应答的时刻
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        if bus.ping(sid) is not None:
            break
        time.sleep(0.05)
    boot_ms = (time.monotonic() - t0) * 1000
    if not ok(bus.ping(sid) is not None, f"PING ID {sid} 有应答（上电后 {boot_ms:.0f} ms）"):
        return out

    # 2. 单播 READ 能读到块（调试台要用）
    try:
        _, blk = bus.read(sid, BLOCK_ADDR, BLOCK_LEN)
        ok(len(blk) == BLOCK_LEN, f"READ 地址 {BLOCK_ADDR} 长 {BLOCK_LEN} 回了 {len(blk)} 字节")
        d = decode_block(blk)
        ok(d["reserved"] == 0, f"保留字节（byte 14）是 0（实际 {d['reserved']}）")
    except Exception as e:
        ok(False, f"READ 地址 {BLOCK_ADDR}：{type(e).__name__}: {e}")
        return out

    # 3. 版本/型号寄存器（0–4）能读，END 是小端
    try:
        _, head = bus.read(sid, 0, 5)
        ok(head[2] == 0, f"END 字节序 = 0 小端（实际 {head[2]}）")
        log(f"版本区 0–4 = {list(head)}", "总线")
    except Exception as e:
        ok(False, f"READ 地址 0 长 5：{type(e).__name__}: {e}")

    # 4. 主控真正发的那条：16 个 ID 一起读
    ids = FULL_IDS if full_bus else [sid]
    stats0 = dict(bus.stats)
    counters, accepted, windows, times, misses, partial = [], 0, [], [], 0, 0
    first_quat_ms = ready_ms = None
    t_start = time.monotonic()
    for k in range(frames):
        t = time.perf_counter()
        got = bus.sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
        times.append((time.perf_counter() - t) * 1000)      # 只包这一句，不含我们自己的 sleep
        v = got.get(sid)
        if not v:
            misses += 1
        else:
            d = decode_block(v[1])
            counters.append(d["counter"])
            windows.append(d["window_hex"])
            if d["accepted"]:
                accepted += 1
                if first_quat_ms is None:
                    first_quat_ms = (time.monotonic() - t_start) * 1000
                if accepted == READY_FRAMES and ready_ms is None:
                    ready_ms = (time.monotonic() - t_start) * 1000
            if frame_log:
                frame_log.write(json.dumps({
                    "frame": k, "ms": round(times[-1], 3), "counter": d["counter"],
                    "gyro_raw": d["gyro_raw"], "quat_packed": d["quat_packed"],
                    "status": d["status"], "accepted": d["accepted"], "window": d["window_hex"],
                }, ensure_ascii=False) + "\n")
        if full_bus and got:
            partial += sum(1 for i in ids if got.get(i) is None and i != sid)
        time.sleep(max(0.0, 1.0 / hz - (time.perf_counter() - t)))
    dstats = {k: bus.stats[k] - stats0.get(k, 0) for k in bus.stats}

    ok(misses == 0, f"{frames} 帧里 IMU 一帧不漏（漏了 {misses}）")
    if full_bus:
        ok(partial == 0, f"同一条 sync_read 里 15 颗舵机也都答了（缺 {partial} 个）")
    ok(dstats.get("bad_id", 0) == 0,
       f"应答没错位（bad_id {dstats.get('bad_id', 0)}）—— 错位一次后面 15 个全废")
    ok(dstats.get("bad_checksum", 0) == 0, f"没有校验错（{dstats.get('bad_checksum', 0)}）")

    # 5. 计数器：采样计数，50 Hz 读 120 Hz 采样 → 每帧 +1~4；恒 0 说明板子没在采样
    steps = [(b - a) % 256 for a, b in zip(counters, counters[1:])]
    ok(steps and all(1 <= s <= 4 for s in steps),
       f"采样计数每帧 +1~4（实际 {sorted(set(steps))[:5]}）—— 恒 0 说明 IMU 没在出数，总线处理器却还活着")

    # 6. 冻结：主控比的是前 12 字节，连续 25 帧相同就报 frozen
    run = best = 0
    for a, b in zip(windows, windows[1:]):
        run = run + 1 if a == b else 0
        best = max(best, run)
    ok(best < STALE_RUN_MAX,
       f"12 字节窗口最长连续相同 {best} 帧（主控 ≥{STALE_RUN_MAX} 帧就报 frozen）")

    # 7. 主控会认的帧够不够，以及多久到 ready
    ok(accepted >= READY_FRAMES,
       f"{frames} 帧里主控会接受 {accepted} 帧（累计 {READY_FRAMES} 帧才 ready）"
       + (f"，{ready_ms:.0f} ms 到 ready" if ready_ms else "，没到 ready"))
    if first_quat_ms is not None:
        log(f"第一个有效四元数在开跑后 {first_quat_ms:.0f} ms", "总线")

    # 8. 时序：只算总线事务本身，判据是 tick 预算不是超时
    times.sort()
    p99 = times[max(0, int(len(times) * 0.99) - 1)]
    ok(p99 < TICK_BUDGET_MS,
       f"事务耗时 min {times[0]:.2f} / 平均 {sum(times)/len(times):.2f} / p99 {p99:.2f} / max {times[-1]:.2f} ms"
       f"（预算 {TICK_BUDGET_MS} ms，不是超时 60 ms）")

    # 9. 广播 PING 不该应答（手册 §4.1：总线上多个设备时不能用广播 PING）
    try:
        bus._send(feetech.BROADCAST, feetech.PING)
        time.sleep(0.01)
        noise = bus.ser.read(64)
        ok(not noise, f"广播 PING 不应答（收到 {len(noise)} 字节）")
    except Exception as e:
        log(f"广播 PING 测试跳过：{type(e).__name__}: {e}", "总线", "warn")

    # 10. 写一个寄存器：应该 ACK 且不影响数据（主控从不写它，但调试台可能手滑）
    try:
        before = bus.read(sid, BLOCK_ADDR, BLOCK_LEN)[1][:IMU_WINDOW]
        bus.write_u8(sid, 40, 0)
        after = bus.read(sid, BLOCK_ADDR, BLOCK_LEN)[1][:IMU_WINDOW]
        ok(True, "写寄存器不会把板子写坏（写完还能正常读块）")
        log(f"写前后 12 字节：{before.hex()} → {after.hex()}", "总线", "debug")
    except Exception as e:
        ok(False, f"写寄存器以后读不到块了：{type(e).__name__}: {e}")

    log(f"总线统计增量：{dstats}", "总线")
    return out


def run_demo():
    from sim_bus import SimSerial
    port = SimSerial(JOINT_IDS)
    port.latency_ms = 0.25                       # 每包 0.25 ms，接近 1 Mbps 的线上时间
    port.servos[IMU_ID] = SimImu200(imu_ready_ms=200)
    bus = feetech.FeetechBus(None, ser=port)
    attach_trace(bus)

    print("== 一条 sync_read 同时读 IMU 和 15 颗舵机（主控每 tick 就是这么干的）==")
    for n in range(6):
        got = bus.sync_read(FULL_IDS, BLOCK_ADDR, BLOCK_LEN)
        d = decode_block(got[IMU_ID][1])
        miss = [i for i in JOINT_IDS if got.get(i) is None]
        print(f"  第{n}帧 计数={d['counter']:>3} 陀螺 {d['gyro_dps'][0]:+6.2f},{d['gyro_dps'][1]:+6.2f},"
              f"{d['gyro_dps'][2]:+6.2f} dps  {'主控接受' if d['accepted'] else '融合还没出数'}"
              f"  舵机缺 {len(miss)} 个")
        time.sleep(0.02)

    print("\n== 一致性检查（同一套判据，对真板子就是 check --port）==")
    bad = sum(not g for g, _ in check(bus, frames=60, frame_log=None))

    print("\n== 板子不应答：整条 sync_read 跟着废，15 颗舵机数据一起丢 ==")
    port.servos[IMU_ID].answer = False
    got = bus.sync_read(FULL_IDS, BLOCK_ADDR, BLOCK_LEN)
    lost = [i for i in FULL_IDS if got.get(i) is None]
    print(f"  IMU 不答这一帧，{len(lost)}/{len(FULL_IDS)} 个设备的数据没了：{lost[:6]}…")
    port.servos[IMU_ID].answer = True

    print("\n== 坏固件示例：四元数 norm² = 2（主控会拒绝，工具必须也拒绝）==")
    bad_blk = struct.pack("<hhh", 0, 0, 0) + struct.pack("<HHH", f32_to_f16(1.0), f32_to_f16(1.0),
                                                         f32_to_f16(0.0)) + bytes([9, 0, 0])
    d = decode_block(bad_blk)
    print(f"  norm²={d['norm_sq']:.2f} → 主控接受？{d['accepted']}（应该是 False）")
    print(f"\n日志在 {LOG_DIR}")
    return bad


def run_vectors():
    def frame(sid, instr, params):
        ln = len(params) + 2
        body = [sid, ln, instr] + list(params)
        return "ff ff " + " ".join(f"{b:02x}" for b in body + [(~sum(body)) & 0xFF])

    print("主控每 tick 发的完整 sync_read（16 个 ID，200 在最前）：")
    print("  " + frame(0xFE, feetech.SYNC_READ, [BLOCK_ADDR, BLOCK_LEN] + FULL_IDS))
    print("\n只问 200 的简化版（调试用）：")
    print("  " + frame(0xFE, feetech.SYNC_READ, [BLOCK_ADDR, BLOCK_LEN, IMU_ID]))
    print("\nPING 200：")
    print("  " + frame(IMU_ID, feetech.PING, []))
    print("\n应答 · 融合还没出数（四元数全 0，状态 BIT0=1，计数 0）：")
    blk = encode_block(counter=0, status=0x01)
    print("  " + frame(IMU_ID, 0x00, list(blk)) + "\n  数据段 " + " ".join(f"{b:02x}" for b in blk))
    print("\n应答 · 有数（陀螺 +1.75/−0.875/+0.0525 dps，四元数 z=0.3827，计数 7）：")
    blk = encode_block(gyro_dps=(1.75, -0.875, 0.0525), quat_xyz=(0.0, 0.0, 0.3827), counter=7)
    print("  " + frame(IMU_ID, 0x00, list(blk)) + "\n  数据段 " + " ".join(f"{b:02x}" for b in blk))
    d = decode_block(blk)
    print(f"\n解回来：陀螺 {d['gyro_raw']} LSB，四元数 {d['quat_packed']}（half），"
          f"主控接受={d['accepted']}，躯干系重力 {tuple(round(v, 3) for v in d['gravity'])}")


def main():
    ap = argparse.ArgumentParser(description="ID 200 IMU 块：模拟、测试向量、真板子检查")
    ap.add_argument("cmd", choices=["demo", "vectors", "check"])
    ap.add_argument("--port", help="真板子在哪个串口（check 用）")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--only-imu", action="store_true", help="只问 200，不带 15 颗舵机")
    a = ap.parse_args()
    if a.cmd == "demo":
        sys.exit(run_demo() and 1)
    if a.cmd == "vectors":
        return run_vectors()
    if not a.port:
        sys.exit("check 要 --port，例如 --port COM6")
    bus = feetech.FeetechBus(a.port, a.baud)
    attach_trace(bus)
    os.makedirs(LOG_DIR, exist_ok=True)
    fl = open(os.path.join(LOG_DIR, f"imu200-frames-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"),
              "w", encoding="utf-8")
    try:
        log(f"对着 {a.port} @ {a.baud} 检查 ID {IMU_ID}，{a.frames} 帧", "总线")
        bad = sum(not g for g, _ in check(bus, frames=a.frames,
                                          full_bus=not a.only_imu, frame_log=fl))
        print("\n全部通过" if not bad else f"\n{bad} 项没过")
        print(f"日志：{LOG_DIR}")
        sys.exit(1 if bad else 0)
    finally:
        fl.close()
        bus.close()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
