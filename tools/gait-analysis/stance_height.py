"""撑地腿把机身撑多高：撑地时「躯干原点到撑地脚最低点」的竖直距离（IMU 转正），实际 vs 目标，左右分开。
撑得越矮（比目标塌得越多），对侧迈步脚就越早落地、越抬不起来。"""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_clear as fc
import mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
adr = [m.jnt_qposadr[m.joint(n).id] for n in fc.J14]
gid = {s: m.geom(f'{s}_foot_collision').id for s in fc.SIDES}; base = m.body('trunk_base').id
def zmin(q, L):
    d.qpos[:] = m.qpos0
    for a, v in zip(adr, q): d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    Rb = d.xmat[base].reshape(3, 3); pb = d.xpos[base]; out = {}
    for s in fc.SIDES:
        g = gid[s]; c = d.geom_xpos[g]; R = d.geom_xmat[g].reshape(3, 3); sz = m.geom_size[g]
        pts = np.array([c + R @ (np.array([a, b, e]) * sz) for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)])
        out[s] = float((((pts - pb) @ Rb) @ L.T)[:, 2].min()) * 1000
    return out
for spec in sys.argv[2:]:
    name, src = spec.split('=', 1)
    held, walk = fc.load(src)
    if held:
        h = [zmin(q, fc.level(g)) for q, _, g in held[-150:]]
        print(f'\n## {name}  站 HOME：左腿撑高 {-np.mean([x["left"] for x in h]):.1f} mm  右腿 {-np.mean([x["right"] for x in h]):.1f} mm')
    else:
        print(f'\n## {name}')
    A = []; T = []
    for q, t, g in walk:
        L = fc.level(g); A.append(zmin(q, L)); T.append(zmin(t, L))
    st = np.array(['left' if a['left'] <= a['right'] else 'right' for a in A])
    for s in fc.SIDES:
        i = np.flatnonzero(st == s)
        a = -np.array([A[k][s] for k in i]); t = -np.array([T[k][s] for k in i])
        print(f'  {fc.CN[s][0]}腿撑地 占走路时间 {len(i)/len(A)*100:3.0f}%   撑高 实际 中位 {np.median(a):5.1f} mm（最低 5% {np.percentile(a,5):5.1f}）  目标 {np.median(t):5.1f} mm   比目标塌 {np.median(t-a):4.1f} mm')
