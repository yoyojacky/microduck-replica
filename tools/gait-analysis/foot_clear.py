"""迈步脚离地间隙（只读分析）：用 IMU 重力把机身转正，算「迈步脚最低点 − 撑地脚最低点」。
每帧：两只脚碰撞盒 8 个角做正运动学（躯干坐标）→ 按 safety.gravity 转到水平坐标 → 较低的脚算撑地，另一只算迈步。
同时用 targets（模型给舵机的目标）替换「迈步腿 / 撑地腿」，拆出间隙损失来自哪里：
  实际/实际：真实间隙；目标/实际：迈步腿完全跟上目标时的间隙；实际/目标：撑地腿一点不塌时的间隙；目标/目标：模型本意。
用法：python foot_clear.py MJCF.xml 名称=录像.jsonl[@起始wall[@结束wall]] ...
"""
import json
import math
import sys

import mujoco
import numpy as np

J14 = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle', 'neck_pitch', 'head_pitch',
       'head_yaw', 'head_roll', 'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle']
LEGIDX = {'left': [0, 1, 2, 3, 4], 'right': [9, 10, 11, 12, 13]}
LEGNAME = ['髋偏航', '髋横滚', '髋俯仰', '膝', '踝']
SIDES = ('left', 'right')
CN = {'left': '左脚', 'right': '右脚'}


def load(spec):
    parts = spec.split('@'); path = parts[0]
    t0 = float(parts[1]) if len(parts) > 1 and parts[1] else -1e18
    t1 = float(parts[2]) if len(parts) > 2 and parts[2] else 1e18
    held, walk = [], []
    for line in open(path, encoding='utf-8', errors='replace'):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if not r.get('joints') or not r.get('targets'):
            continue
        if not t0 <= r.get('wall', 0) < t1:
            continue
        g = (r.get('safety') or {}).get('gravity') or (r.get('imu') or {}).get('gravity')
        if not g:
            continue
        j = r['joints']; tg = r['targets']
        row = (np.array(j[:9] + j[10:15]), np.array(tg[:9] + tg[10:15]), np.array(g, float))
        req = (r.get('move') or {}).get('requested', [0, 0, 0])
        if r.get('policy') == 'held' and not (r.get('safety') or {}).get('limp'):
            held.append(row)
        elif r.get('policy') == 'walk' and math.hypot(req[0], req[1]) > 0.02:
            walk.append(row)
    return held, walk


def level(g):
    """把躯干坐标转到水平坐标的旋转（只转横滚/俯仰，把重力方向转成 -z）。"""
    v = g / np.linalg.norm(g); t = np.array([0.0, 0.0, -1.0])
    ax = np.cross(v, t); s = np.linalg.norm(ax); c = float(np.dot(v, t))
    if s < 1e-9:
        return np.eye(3)
    k = ax / s; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def main():
    m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
    adr = [m.jnt_qposadr[m.joint(n).id] for n in J14]
    gid = {s: m.geom(f'{s}_foot_collision').id for s in SIDES}
    base = m.body('trunk_base').id

    def corners(q):
        d.qpos[:] = m.qpos0
        for a, v in zip(adr, q): d.qpos[a] = v
        mujoco.mj_kinematics(m, d)
        Rb = d.xmat[base].reshape(3, 3); pb = d.xpos[base]
        out = {}
        for s in SIDES:
            g = gid[s]; c = d.geom_xpos[g]; R = d.geom_xmat[g].reshape(3, 3); sz = m.geom_size[g]
            pts = np.array([c + R @ (np.array([a, b, e]) * sz) for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)])
            out[s] = (pts - pb) @ Rb                                     # 躯干坐标：x 前 y 左 z 上
        return out

    def label(p, side):
        """最低角在脚掌哪个位置：前/后 + 内/外。"""
        fb = '脚尖' if p[0] > 0 else '脚跟'
        inner = (p[1] < 0) if side == 'left' else (p[1] > 0)
        return fb + ('内侧' if inner else '外侧')

    for spec in sys.argv[2:]:
        name, src = spec.split('=', 1)
        held, walk = load(src)
        print(f'\n## {name}：站 HOME {len(held)} 帧，走路 {len(walk) / 50:.1f} s')
        if len(held) >= 25:
            gap = []
            for q, _, g in held[-150:]:
                C = corners(q); L = level(g)
                zl = (C['left'] @ L.T)[:, 2]; zr = (C['right'] @ L.T)[:, 2]
                gap.append((zl.min() - zr.min()) * 1000)
            print(f'  自检：站 HOME 时左脚最低点 − 右脚最低点 = {np.mean(gap):+.1f} mm（应接近 0，两脚都踩地）')
        if len(walk) < 100:
            continue
        A = [corners(q) for q, _, _ in walk]; T = [corners(t) for _, t, _ in walk]
        Ls = [level(g) for _, _, g in walk]
        def zmin(C, L, s): return float((C[s] @ L.T)[:, 2].min())
        za = {s: np.array([zmin(C, L, s) for C, L in zip(A, Ls)]) for s in SIDES}
        zt = {s: np.array([zmin(C, L, s) for C, L in zip(T, Ls)]) for s in SIDES}
        stance = np.where(za['left'] <= za['right'], 'left', 'right')
        roll = np.degrees([math.asin(-g[1] / np.linalg.norm(g)) for _, _, g in walk])
        for s in SIDES:
            o = 'right' if s == 'left' else 'left'
            sw = stance == o                                              # 这只脚在迈步的帧
            idx = np.flatnonzero(sw)
            if len(idx) < 10:
                continue
            segs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            segs = [g for g in segs if len(g) >= 3]
            def peak(zs, zo):
                v = (zs - zo) * 1000
                return np.array([v[g].max() for g in segs]), v
            aa, va = peak(za[s], za[o]); ta, _ = peak(zt[s], za[o]); at, _ = peak(za[s], zt[o]); tt, _ = peak(zt[s], zt[o])
            drag = np.mean(va[idx] < 2.0) * 100
            low = [label((A[g[np.argmax(va[g])]][s] @ Ls[g[np.argmax(va[g])]].T)[np.argmin((A[g[np.argmax(va[g])]][s] @ Ls[g[np.argmax(va[g])]].T)[:, 2])], s) for g in segs]
            top = max(set(low), key=low.count)
            print(f'  {CN[s]} 迈步 {len(segs):3d} 次  每步最高间隙 中位 {np.median(aa):4.1f} mm  （迈步帧里 <2 mm 占 {drag:3.0f}%）  '
                  f'最高时最低点多在 {top}（{low.count(top)}/{len(low)}）')
            print(f'        拆分：模型本意 {np.median(tt):4.1f}  ｜ 迈步腿完全跟上 {np.median(ta):4.1f}  ｜ 撑地腿不塌 {np.median(at):4.1f}  ｜ 实际 {np.median(aa):4.1f} mm'
                  f'   迈步时机身横滚均值 {roll[sw].mean():+.1f}°')
        # 跟随：每条腿每个关节 实际 vs 目标 的摆幅比和平均偏差（撑地 / 迈步分开）
        Q = np.array([q for q, _, _ in walk]); Tg = np.array([t for _, t, _ in walk])
        for s in SIDES:
            o = 'right' if s == 'left' else 'left'
            row = []
            for k, i in enumerate(LEGIDX[s]):
                amp_q = np.percentile(Q[:, i], 95) - np.percentile(Q[:, i], 5)
                amp_t = np.percentile(Tg[:, i], 95) - np.percentile(Tg[:, i], 5)
                bias_st = np.degrees(np.mean((Q[:, i] - Tg[:, i])[stance == s]))
                row.append(f'{LEGNAME[k]} 目标摆幅 {math.degrees(amp_t):4.1f}° 实际/目标 {amp_q / amp_t if amp_t > 1e-6 else 0:4.2f} 撑地时偏差 {bias_st:+4.1f}°')
            print(f'  {CN[s]}腿跟随：' + '；'.join(row[1:]))


if __name__ == '__main__':
    main()
