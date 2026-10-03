# -*- coding: utf-8 -*-
"""舵机体检：一条命令读出每颗舵机的保护设置和当前状态，存成报告。**只读，不写舵机任何寄存器。**

  python servo_report.py --port COM5                 # 默认 15 颗：20-24,30-34,10-14
  python servo_report.py --port COM5 --ids 21        # 总线上只接 21 号时
  python servo_report.py --port COM5 --scan-all      # 不知道 ID（新舵机出厂是 1）：扫 0–253

报告写在 logs/体检-日期-时间.md（给人看）和同名 .json（每颗 0–86 寄存器原值）。
串口不能同时被调试台（server.py）或 FD 占着，先关掉它们。
判据按《调试记录》第 5 步：过热（4）、过流（8）、过压（1）三个保护都要开，没开标红。
"""
import argparse
import json
import os
import sys
import time

import feetech

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IDS = "20-24,30-34,10-14"
NAMES = {20: "左髋 yaw", 21: "左髋 roll", 22: "左髋 pitch", 23: "左膝", 24: "左踝",
         30: "颈 pitch", 31: "头 pitch", 32: "头 yaw", 33: "头 roll", 34: "嘴",
         10: "右髋 yaw", 11: "右髋 roll", 12: "右髋 pitch", 13: "右膝", 14: "右踝"}
# 卸载条件（19）/ 状态位（65）每一位的意思，两个寄存器同一套定义
BITS = {1: "电压", 2: "磁编码", 4: "过热", 8: "过流", 32: "过载"}
MUST = (4, 8, 1)          # 调试记录第 5 步：过热出厂开着，过流、过压要自己开


def parse_ids(text):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        elif part:
            ids.append(int(part))
    return ids


def bits_text(v):
    on = [n for b, n in BITS.items() if v & b]
    return "、".join(on) if on else "无"


def examine(bus, sid):
    """读一颗：PING → 导出 0–86 → 解码 → 判红黄。返回一行结果；哪一步读不到写在 step 里。"""
    row = {"id": sid, "name": NAMES.get(sid, ""), "online": False, "red": [], "yellow": [], "info": []}
    step = "PING"
    try:
        if bus.ping(sid) is None:
            row["red"].append("PING 不通：没供电、线没插好、ID 不对，或者驱动烧了")
            return row
        row["online"] = True
        step = "导出 0–86 寄存器"
        d = bus.dump(sid)
    except Exception as e:
        row["red"].append(f"卡在「{step}」：{type(e).__name__}: {e}")
        return row
    v = {r["addr"]: r["value"] for r in d["rows"]}
    row["raw"] = d["raw"]
    s15 = feetech.sign15
    row.update({
        "firmware": f"{v[0]}.{v[1]}", "servo_ver": f"{v[3]}.{v[4]}",
        "unload": v[19], "status": v[65], "temp_c": v[63], "volt": v[62] / 10,
        "volt_range": (v[15] / 10, v[14] / 10), "temp_limit": v[13],
        # 38 / 35 的单位按 10 ms 换算，报告里原值也写上（单位没在真机上核过）
        "protect_ma": round(v[28] * 6.5), "overcurrent_raw": v[38], "protect_time_raw": v[35],
        "overload_pct": v[36], "max_torque": v[16], "torque_limit": v[48],
        "angle_limit": (v[9], v[11]), "offset": s15(v[31]), "pos": s15(v[56]), "goal": s15(v[42]),
        "current_ma": round(s15(v[69]) * 6.5), "load_pct": feetech.sign10(v[60]) / 10,
        "torque_on": v[40], "lock": v[55], "mode": v[33], "baud": v[6], "reply_level": v[8],
    })
    missing = [BITS[b] for b in MUST if not v[19] & b]
    if missing:
        row["red"].append(f"卸载条件 19 = {v[19]}（开着：{bits_text(v[19])}），没开：{'、'.join(missing)}")
    if not v[19] & 32:
        row["yellow"].append("过载保护（32）没开：负载超过「过载扭矩」持续「保护时间」才断，堵转时多一道保险")
    if v[65]:
        row["red"].append(f"状态位 65 = {v[65]}：{bits_text(v[65])} 正在报警")
    lo, hi = row["volt_range"]
    if not lo <= row["volt"] <= hi:
        row["red"].append(f"电压 {row['volt']:.1f} V 不在舵机自己设的 {lo:.1f}–{hi:.1f} V 里")
    if v[63] >= 60:
        row["yellow"].append(f"温度 {v[63]} °C，偏高（上限 {v[13]} °C 断扭矩）")
    if abs(row["offset"]) > 200:
        row["yellow"].append(f"偏移 {row['offset']} 步（{row['offset'] * 360 / 4096:+.1f}°），很大：装舵盘时差了好几个齿，或者转了半圈")
    if v[33] != 4:
        row["yellow"].append(f"运行模式 33 = {v[33]}，调试记录要求 4（纯位置 PD）")
    if (v[9], v[11]) == (0, 4095):
        row["info"].append("没设角度限位（0–4095）：命令到哪都会去，包括撞机械限位")
    return row


def report_md(rows, port, t):
    L = [f"# 舵机体检 {t}", "", f"串口 {port}，只读，没写任何寄存器。判据见 servo_report.py 开头。", ""]
    online = [r for r in rows if r["online"]]
    reds = [r for r in rows if r["red"]]
    L.append(f"**在线 {len(online)}/{len(rows)} 颗；有红项的 {len(reds)} 颗**" + (f"：{' '.join(str(r['id']) for r in reds)}" if reds else ""))
    L += ["", "| ID | 关节 | 在线 | 卸载条件 19 | 状态 65 | 温度 | 电压 | 电流 | 读数 | 偏移 | 扭矩 | 固件 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if not r["online"]:
            L.append(f"| {r['id']} | {r['name']} | ✗ | | | | | | | | | |")
            continue
        L.append(f"| {r['id']} | {r['name']} | ✓ | {r['unload']}（{bits_text(r['unload'])}） | {r['status']}（{bits_text(r['status'])}） "
                 f"| {r['temp_c']} °C | {r['volt']:.1f} V | {r['current_ma']} mA | {r['pos']} | {r['offset']:+d} "
                 f"| {'开' if r['torque_on'] else '关'} | {r['firmware']} |")
    L += ["", "## 每颗的问题", ""]
    for r in rows:
        items = [f"- 🔴 {x}" for x in r["red"]] + [f"- 🟡 {x}" for x in r["yellow"]] + [f"- {x}" for x in r["info"]]
        if items:
            L += [f"**#{r['id']} {r['name']}**", *items, ""]
    L += ["## 保护参数（在线的）", "", "| ID | 保护电流 | 过流保护时间 38 | 过载扭矩 | 保护时间 35 | 温度上限 | 最大扭矩 16 | 转矩限制 48 | 角度限位 |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in online:
        L.append(f"| {r['id']} | {r['protect_ma']} mA | {r['overcurrent_raw']}（按 10 ms 算 {r['overcurrent_raw'] * 10} ms） | {r['overload_pct']} % "
                 f"| {r['protect_time_raw']}（按 10 ms 算 {r['protect_time_raw'] * 10} ms） "
                 f"| {r['temp_limit']} °C | {r['max_torque']} | {r['torque_limit']} | {r['angle_limit'][0]}–{r['angle_limit'][1]} |")
    return "\n".join(L) + "\n"


def run(bus, ids, port, out_dir):
    t = time.strftime("%Y-%m-%d %H:%M:%S")
    rows = []
    for sid in ids:
        r = examine(bus, sid)
        if r["online"] or len(ids) <= 32:          # 扫 0–253 时只留在线的
            rows.append(r)
        mark = "✗" if not r["online"] else ("🔴" if r["red"] else ("🟡" if r["yellow"] else "✓"))
        if r["online"] or len(ids) <= 32:
            print(f"#{sid:<3} {r['name']:<8} {mark}  " + "；".join(r["red"] + r["yellow"])[:160], flush=True)
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, "体检-" + time.strftime("%Y%m%d-%H%M%S"))
    with open(base + ".md", "w", encoding="utf-8") as f:
        f.write(report_md(rows, port, t))
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump({"time": t, "port": port, "servos": rows}, f, ensure_ascii=False, indent=1)
    return base, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description="舵机体检（只读）")
    ap.add_argument("--port", required=True, help="串口，如 COM5 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", default=DEFAULT_IDS)
    ap.add_argument("--scan-all", action="store_true", help="扫 0–253，只报在线的")
    ap.add_argument("--out", default=os.path.join(HERE, "logs"))
    a = ap.parse_args(argv)
    ids = list(range(0, 254)) if a.scan_all else parse_ids(a.ids)
    try:
        bus = feetech.FeetechBus(a.port, a.baud)
    except Exception as e:
        raise SystemExit(f"打不开串口 {a.port}：{type(e).__name__}: {e}（调试台或 FD 占着的话先关掉）")
    try:
        base, rows = run(bus, ids, a.port, a.out)
    finally:
        bus.close()
    reds = [r["id"] for r in rows if r["red"]]
    print(f"\n在线 {sum(r['online'] for r in rows)}/{len(rows)}；红项 {reds or '无'}")
    print(f"报告：{base}.md\n原始寄存器：{base}.json")
    return 1 if reds else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
