# -*- coding: utf-8 -*-
"""调试台后端自检：真实的 server.handle + FeetechBus，对着协议级模拟器（sim_bus）跑一遍会写舵机的功能。

  python selftest.py

改了 feetech.py 或 server.py 里跟舵机打交道的地方，先跑这个再上真机。
不碰真舵机，不碰仓库里的 poses/ 和 calib/（用临时目录）。
"""
import os
import shutil
import sys
import tempfile

sys.stdout.reconfigure(encoding="utf-8")
import feetech
import server
from sim_bus import SimSerial

IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]
FAILS = []


def check(cond, what):
    print(("  ✓ " if cond else "  ✗ ") + what)
    if not cond:
        FAILS.append(what)


def main():
    tmp = tempfile.mkdtemp(prefix="servo-web-selftest-")
    server.CALIB_DIR = os.path.join(tmp, "calib")
    server.POSES_DIR = os.path.join(tmp, "poses")
    server.LOG_DIR = os.path.join(tmp, "logs")
    os.makedirs(server.POSES_DIR)
    # 摆在一个离 2048 挺远的姿势上
    start = {i: 2048 + (k * 37 - 250) for k, i in enumerate(IDS)}
    port = SimSerial(IDS, phys=start)
    server.BUS = feetech.FeetechBus(None, ser=port)
    server.IDS = IDS
    server.PRESENT = []
    H = server.handle
    sv = {s.r[5]: s for s in port.servos.values()}

    print("扫描、读状态")
    H({"op": "scan"})
    check(server.PRESENT == IDS, f"15 颗都在线（{len(server.PRESENT)}）")
    st = server.BUS.states(IDS)
    check(all(st[i]["pos"] == start[i] for i in IDS), "同步读的位置对得上")

    print("存一个姿态，扭矩开，摆好不动")
    H({"op": "torque", "on": 1})
    H({"op": "save_pose", "name": "测试站姿"})
    before = {i: sv[i].phys for i in IDS}

    print("一键校准到零位（扭矩开着，0x0B）")
    r = H({"op": "calib_mid_all", "ids": IDS})
    check(len(r["ok"]) == 15 and not r["bad"], f"15 颗成功（成功 {len(r['ok'])}，失败 {r['bad']}）")
    check(all(sv[i].calibrations == 1 for i in IDS), "每颗都真的做了一次校准（不是被拉过去的）")
    check(all(sv[i].phys == before[i] for i in IDS), "物理位置一步没动")
    st = server.BUS.states(IDS)
    check(all(abs(st[i]["pos"] - 2048) <= 1 for i in IDS), "校准后都读 2048")
    check(all(sv[i].r[55] == 1 for i in IDS), "每颗都重新加锁了")
    check(all(sv[i].r[40] == 1 for i in IDS), "扭矩恢复成开着")
    poses = server.list_poses()
    p = [x for x in poses if x["name"] == "测试站姿"][0]
    check(all(abs(p["goals"][i] - 2048) <= 1 for i in IDS), "存的姿态换算到新零点（同一个姿势现在是 2048）")

    print("回零位不该动")
    H({"op": "goals", "goals": {str(i): 2048 for i in IDS}})
    server.BUS.states(IDS)
    check(all(sv[i].phys == before[i] for i in IDS), "发 2048 以后物理位置不变")

    print("撤销")
    r = H({"op": "calib_undo"})
    check(len(r["ok"]) == 15, f"15 颗偏移写回（{len(r['ok'])}）")
    st = server.BUS.states(IDS)
    check(all(st[i]["pos"] == start[i] for i in IDS), "读数回到校准前")
    check(all(sv[i].phys == before[i] for i in IDS), "撤销时物理位置也没动")
    check(all(sv[i].r[55] == 1 for i in IDS), "撤销后都加锁了")
    p = [x for x in server.list_poses() if x["name"] == "测试站姿"][0]
    check(all(p["goals"][i] == start[i] for i in IDS), "存的姿态换算回原值")

    print("校准后新存的姿态，撤销时也要跟着挪回来")
    H({"op": "calib_mid_all", "ids": IDS})
    H({"op": "save_pose", "name": "校准后存的"})
    phys_at_save = {i: sv[i].phys for i in IDS}     # 存它的时候，每个关节的物理位置
    H({"op": "calib_undo"})
    after = [x for x in server.list_poses() if x["name"] == "校准后存的"][0]["goals"]
    # 真正要保证的：撤销以后把这个姿态发出去，关节还是回到存它时那个物理姿势
    want = {i: round(phys_at_save[i] + sv[i].off) for i in IDS}
    worst = max(abs(after[i] - want[i]) for i in IDS)
    check(worst <= 2, f"撤销后姿态里的数字仍然指向存它时那个物理姿势（最大差 {worst} 步）")
    server.delete_pose([x for x in server.list_poses() if x["name"] == "校准后存的"][0]["file"])

    print("按姿态校准：鸭子缩着放好，读数校回「缩」里存的值")
    H({"op": "save_pose", "name": "缩"})
    H({"op": "save_pose", "name": "别的"})
    ref = [x for x in server.list_poses() if x["name"] == "缩"][0]
    other_before = [x for x in server.list_poses() if x["name"] == "别的"][0]["goals"]
    legs = [22, 23, 24, 12, 13, 14]
    H({"op": "torque", "ids": legs, "on": 0})          # 缩着放在地上，腿扭矩关着
    for i in legs:                                     # 模拟舵盘重装：同一个物理位置读数偏了 7 步
        sv[i].off += 7
    phys = {i: sv[i].phys for i in IDS}
    r = H({"op": "calib_mid_all", "ids": legs, "ref": ref["file"]})
    st = server.BUS.states(IDS)
    check(sorted(r["ok"]) == sorted(legs), f"6 颗腿成功（{r['ok']}，失败 {r['bad']}）")
    check(all(st[i]["pos"] == ref["goals"][i] for i in legs), "读数回到「缩」里存的值")
    check(all(sv[i].phys == phys[i] for i in IDS), "物理位置没动")
    now = {x["name"]: x["goals"] for x in server.list_poses()}
    check(now["缩"] == ref["goals"], "基准姿态本身没被挪")
    check(now["别的"] == other_before, "别的姿态也没挪（跟基准同一套读数）")
    H({"op": "calib_undo"})
    st = server.BUS.states(IDS)
    check(all(st[i]["pos"] == ref["goals"][i] + 7 for i in legs), "撤销回到偏了 7 步的样子")

    print("舵机不认 0x0B：自动换兜底（自己算偏移写寄存器 31）")
    sv[31].support_0b = False
    H({"op": "goals", "goals": {"31": 2300}})
    server.BUS.states([31])
    p31 = sv[31].phys
    r = H({"op": "calibrate", "id": 31})
    st = server.BUS.states([31])
    check(r["ok"] == [31], f"兜底把它校准了（失败 {r['bad']}）")
    check(abs(p31 + sv[31].off - 2048) <= 2, "兜底也是把摆好的位置校成 2048")
    check(abs(sv[31].phys - p31) <= 20 and sv[31].r[40] == 1 and sv[31].r[55] == 1, "关节没被拉走、扭矩恢复、加锁了")
    sv[31].support_0b = True

    print("0x0B 和写偏移都不认：偏移还原，不留半吊子")
    sv[31].support_0b = False
    sv[31].ignore_offset_writes = True
    H({"op": "goals", "goals": {"31": 2300}})     # 先挪开，不然"本来就在 2048"会直接算成功
    server.BUS.states([31])
    o31 = sv[31].off
    r = H({"op": "calibrate", "id": 31})
    check(not r["ok"] and "31" in r["bad"], f"报失败（{r['bad'].get('31', '')[:40]}）")
    check(sv[31].off == o31, "偏移原样还原")
    sv[31].support_0b = True
    sv[31].ignore_offset_writes = False

    print("写 128 在 HD-1910 上不生效（模拟器跟真机一致）")
    sv[32].calibrations = 0
    server.BUS.unlock(32)
    server.BUS.write_u8(32, 40, 0)
    server.BUS.write_u8(32, 40, 128)
    server.BUS.lock(32)
    check(sv[32].calibrations == 0, "解锁、扭矩关着写 128 也不校准")

    print("单颗中位校准按钮")
    r = H({"op": "calibrate", "id": 32})
    check(r["ok"] == [32] and sv[32].calibrations == 1, "单颗也走同一套流程")

    print("中途掉线：0x0B 以后读不到位置")
    sv[33].mute_after_calib = {56}
    r = H({"op": "calibrate", "id": 33})
    sv[33].mute_after_calib.clear(); sv[33].muted.clear()
    why = r["bad"].get("33", "")
    check("卡在「" in why, f"失败原因写明卡在哪一步（{why[:40]}…）")
    check(sv[33].r[55] == 1, "出错也重新加锁了（finally）")
    check(sv[33].r[40] == 0, "位置读不到就不恢复扭矩，免得按旧目标把关节拉走")
    line = [e for e in server.LOG if e["cat"] == "校准" and e["msg"].startswith("#33") and e["level"] == "warn"]
    check(bool(line) and "加锁✓" in line[-1]["msg"], "日志那行里有每一步的结果")
    with open(server.log_path(), encoding="utf-8") as f:
        text = f.read()
    check("Traceback" in text and "[校准][错误]" in text, "日志文件里有分类和 traceback")
    H({"op": "torque", "id": 33, "on": 1})

    print("串口原始收发进日志文件")
    server.BUS.trace = server.bus_trace
    H({"op": "calibrate", "id": 32})
    text = open(server.log_path(), encoding="utf-8").read()
    check("→ #32 位置校准" in text, "发出去的 0x0B 包记下来了（含十六进制）")
    check("地址40「扭矩开关」" in text and "地址31" in text, "读写寄存器记了地址和名字")
    check("← #32 状态0x00" in text, "舵机的应答也记了")
    check("SYNC_READ" not in text, "10 Hz 的状态轮询不记，不然日志全是它")
    server.BUS.trace = None

    print("日志分类")
    cats = {e["cat"] for e in server.LOG}
    check({"总线", "运动", "校准", "姿态"} <= cats, f"出现的分类 {sorted(cats)}")
    ok_line = [e for e in server.LOG if e["cat"] == "校准" and e["msg"].startswith("#20 ")]
    check(bool(ok_line) and "偏移" in ok_line[0]["msg"] and "0x0B→2048✓" in ok_line[0]["msg"], "成功的那颗也记了偏移前后、每一步")
    if ok_line:
        print("    例：" + ok_line[0]["msg"])

    print("会往下垂的关节（真机颈部 0.62 步/毫秒，总线每包 1.5 ms）")
    port.latency_ms = 1.5
    for i in IDS:
        sv[i].droop = 0.62
    H({"op": "torque", "on": 1})
    server.BUS.states(IDS)
    held = {i: sv[i].phys for i in IDS}          # 扭矩开着，舵机顶着不动
    r = H({"op": "calib_mid_all", "ids": IDS})
    st = server.BUS.states(IDS)
    check(len(r["ok"]) == 15, f"15 颗都成功（{len(r['ok'])} 成功，失败 {list(r['bad'])[:3]}）")
    # 真正的精度指标：把「你摆好的那个物理位置」代进新的坐标系，应该正好读 2048。
    # 关节被重力垂几步是物理现实，挡不住；能挡住的是"零位被垂坏"。
    worst = max(abs(held[i] + sv[i].off - 2048) for i in IDS)
    check(worst <= 2, f"摆好的位置校准后读 2048（最大误差 {worst:.0f} 步 = {worst * 360 / 4096:.2f}°）")
    yank = max(abs(sv[i].phys - held[i]) for i in IDS)
    check(yank <= 20, f"关节没被拉走（最多移动 {yank:.1f} 步，就是关扭矩那几毫秒垂的）")
    for i in IDS:
        sv[i].droop = 0.0
    port.latency_ms = 0.0
    H({"op": "calib_undo"})

    print("导出寄存器")
    r = H({"op": "dump", "id": 20})
    check(len(r["raw"]) == feetech.DUMP_END, f"读到 {len(r['raw'])} 字节，到地址 86 为止")

    print("解锁写寄存器、改 ID")
    H({"op": "write_reg", "id": 21, "addr": 21, "size": 1, "value": 40, "unlock": True})
    check(sv[21].r[21] == 40 and sv[21].r[55] == 1, "写 P=40，写完加锁")
    ok = server.BUS.set_id(34, 35)
    check(ok and sv[34].r[5] == 35 and sv[34].r[55] == 1, "改 ID 34 → 35 成功并加锁")

    shutil.rmtree(tmp)
    print()
    print("全部通过" if not FAILS else f"{len(FAILS)} 项没过：{FAILS}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
