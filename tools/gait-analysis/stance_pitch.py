"""撑地时机身前后倾从哪来：撑地脚踩平时，机身俯仰 ≈ 撑地腿各关节让脚掌相对躯干转的角度。
按关节拆：贡献 = 脚掌俯仰(HOME 只把该关节换成撑地时中位值) − 脚掌俯仰(HOME)；+ = 让机身往前栽。"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_clear as fc
import mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
adr = [m.jnt_qposadr[m.joint(n).id] for n in fc.J14]
gid = {s: m.geom(f'{s}_foot_collision').id for s in fc.SIDES}; base = m.body('trunk_base').id
HOME = np.array([0.0, -0.0873, -0.4579, -0.0049, 0.453, 0.3491, 0.3491, 0.0, 0.0, 0.0, 0.0873, 0.4579, 0.0049, -0.453])
def fp(q, s):
    d.qpos[:] = m.qpos0
    for a, v in zip(adr, q): d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    R = d.geom_xmat[gid[s]].reshape(3, 3); f = d.xmat[base].reshape(3, 3).T @ R[:, int(np.argmax(m.geom_size[gid[s]]))]
    f = f if f[0] > 0 else -f
    return math.degrees(math.atan2(-f[2], math.hypot(f[0], f[1])))
def zmin(q, L):
    d.qpos[:] = m.qpos0
    for a, v in zip(adr, q): d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    Rb = d.xmat[base].reshape(3, 3); pb = d.xpos[base]; out = {}
    for s in fc.SIDES:
        g = gid[s]; c = d.geom_xpos[g]; R = d.geom_xmat[g].reshape(3, 3); sz = m.geom_size[g]
        pts = np.array([c + R @ (np.array([a, b, e]) * sz) for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)])
        out[s] = float((((pts - pb) @ Rb) @ L.T)[:, 2].min())
    return out
for spec in sys.argv[2:]:
    name, src = spec.split('=', 1)
    _, walk = fc.load(src)
    Q = np.array([q for q, _, _ in walk]); T = np.array([t for _, t, _ in walk]); G = np.array([g for _, _, g in walk])
    Z = [zmin(q, fc.level(g)) for q, _, g in walk]
    st = np.array(['left' if z['left'] <= z['right'] else 'right' for z in Z])
    bp = np.degrees(np.arcsin(G[:, 0] / np.linalg.norm(G, axis=1)))
    print(f'\n## {name}')
    for s in fc.SIDES:
        i = np.flatnonzero(st == s); qm = np.median(Q[i], axis=0); tm = np.median(T[i], axis=0)
        parts = []
        for n, j in zip(('髋俯仰', '膝', '踝'), fc.LEGIDX[s][2:5]):
            q1 = HOME.copy(); q1[j] = qm[j]; t1 = HOME.copy(); t1[j] = tm[j]
            parts.append(f'{n} 实际 {fp(q1, s) - fp(HOME, s):+5.1f}（目标 {fp(t1, s) - fp(HOME, s):+5.1f}）')
        print(f'  {fc.CN[s][0]}腿撑地：机身前倾（IMU）中位 {np.median(bp[i]):+5.1f}°  撑地脚相对躯干俯仰 {fp(qm, s):+5.1f}°  = ' + '  '.join(parts))
