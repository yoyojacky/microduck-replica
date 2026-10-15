"""站 HOME 时两只脚掌在横滚（左右）方向差多少、是哪个关节造成的、哪条边低（只读分析）。
脚掌法线 = 脚碰撞盒最短边方向；横滚 = 法线在躯干 y-z 平面里的倾角（+ = 法线往机身左边倒 = 脚掌左边低）。
两脚横滚差 = 左脚横滚 − 右脚横滚，**相对仿真 HOME 姿势**：仿真模型在 HOME 时两只脚底本身就差 5.97°（每只外侧低约 3°，
是模型几何，见开头自检行），下面都减掉这个基准。读数和 HOME 目标用同一个零点，所以零位偏差在这里看不出来。
关节贡献 = 只把该关节从 HOME 换成实测值时差值的变化（其余关节保持 HOME）。
内外侧边 = 躯干摆正时，脚碰撞盒底面靠身体中线的长边（内）和远离中线的长边（外）的平均高度；碰撞盒是简化的脚，真实情况要看实物。
用法：python home_roll.py MJCF.xml 名称=录像.jsonl ...   （取每段 policy=held 且未卸力的最后 150 帧，按段统计）
     python home_roll.py MJCF.xml 名称=home-capture.json ...（单帧，字段 joints15）
"""
import json
import math
import sys

import mujoco
import numpy as np

J14 = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle', 'neck_pitch', 'head_pitch',
       'head_yaw', 'head_roll', 'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle']
HOME = np.array([0.0, -0.0873, -0.4579, -0.0049, 0.453, 0.3491, 0.3491, 0.0, 0.0, 0.0, 0.0873, 0.4579, 0.0049, -0.453])
LEG = {'left': [0, 1, 2, 3, 4], 'right': [9, 10, 11, 12, 13]}
NAME = ['髋偏航', '髋横滚', '髋俯仰', '膝', '踝']
CN = {'left': '左', 'right': '右'}

m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
adr = [m.jnt_qposadr[m.joint(n).id] for n in J14]
gid = {s: m.geom(f'{s}_foot_collision').id for s in ('left', 'right')}
base = m.body('trunk_base').id


def fk(q):
    d.qpos[:] = m.qpos0
    for a, v in zip(adr, q): d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    return d.xmat[base].reshape(3, 3), d.xpos[base]


def rolls(q):
    Rb, _ = fk(q); out = {}
    for s, g in gid.items():
        R = d.geom_xmat[g].reshape(3, 3); n = Rb.T @ R[:, int(np.argmin(m.geom_size[g]))]
        if n[2] < 0: n = -n
        out[s] = math.degrees(math.atan2(n[1], n[2]))       # + = 法线往机身左边倒 = 脚掌左边低
    return out


def diff(q):
    r = rolls(q); return r['left'] - r['right']


def edges(q):
    """躯干坐标下每只脚底面内侧边、外侧边的平均高度（mm）。
    底面 = 碰撞盒最薄方向上朝下的那一面；内外 = 底面左右两条长边里离身体中线近 / 远的那条。"""
    Rb, pb = fk(q); out = {}
    for s, g in gid.items():
        R = Rb.T @ d.geom_xmat[g].reshape(3, 3); c = Rb.T @ (d.geom_xpos[g] - pb); h = m.geom_size[g]
        n = int(np.argmin(h)); down = -1.0 if R[2, n] > 0 else 1.0
        rest = [k for k in range(3) if k != n]
        lat = max(rest, key=lambda k: abs(R[1, k])); lon = [k for k in rest if k != lat][0]
        bottom = c + down * h[n] * R[:, n]
        side = {}
        for sg in (-1.0, 1.0):
            e = bottom + sg * h[lat] * R[:, lat]
            side[sg] = (abs(e[1]), np.mean([(e + sx * h[lon] * R[:, lon])[2] for sx in (-1.0, 1.0)]))
        inner, outer = sorted(side.values())                  # |y| 小的是内侧
        out[s] = (inner[1] * 1000, outer[1] * 1000)
    return out


def edge_text(q):
    parts = []
    for s, (zi, zo) in edges(q).items():
        parts.append(f"{CN[s]}脚{'外侧' if zo < zi else '内侧'}低 {abs(zo - zi):.1f} mm")
    return '，'.join(parts)


def report(name, Q):
    Q = np.asarray(Q); qm = np.median(Q, axis=0); base_d = diff(HOME)
    total = diff(qm) - base_d
    parts = []
    for s in ('left', 'right'):
        for k, i in enumerate(LEG[s]):
            q1 = HOME.copy(); q1[i] = qm[i]
            c = diff(q1) - base_d
            if abs(c) >= 0.05: parts.append(f"{CN[s]}{NAME[k]} {c:+.2f}°")
    devs = '  '.join(f"{CN[s]}{NAME[k]} {math.degrees(qm[i] - HOME[i]):+.1f}°"
                     for s in ('left', 'right') for k, i in enumerate(LEG[s]) if k in (0, 1))
    print(f"{name:28s} {len(Q):6d} 帧  两脚横滚差 {total:+.2f}°  = {'  '.join(parts)}\n"
          f"{'':36s}髋偏航/髋横滚 实测-HOME：{devs}\n{'':36s}躯干摆正时：{edge_text(qm)}")


def held_segments(path):
    segs, cur = [], []
    for line in open(path, encoding='utf-8', errors='replace'):
        try: r = json.loads(line)
        except ValueError: continue
        if not r.get('joints'):
            continue
        j = r['joints']; q = j[:9] + j[10:15]
        if r.get('policy') == 'held' and not (r.get('safety') or {}).get('limp'):
            cur.append(q)
        elif cur:
            if len(cur) >= 100: segs.append(cur)
            cur = []
    if len(cur) >= 100: segs.append(cur)
    return segs


print(f"仿真 HOME 自检：两脚横滚差 {diff(HOME):+.3f}°，{edge_text(HOME)}（下面的差值都减掉了这个基准）")
for spec in sys.argv[2:]:
    name, path = spec.split('=', 1)
    if path.endswith('.json'):
        j = json.load(open(path, encoding='utf-8'))['joints15']; report(name, [j[:9] + j[10:15]])
        continue
    for k, seg in enumerate(held_segments(path)):
        report(f'{name}#{k + 1}', seg[-150:])
