"""迈步最高点脚掌俯仰（相对本脚撑地时）按关节拆分：贡献 = ∂脚掌俯仰/∂关节 × (最高点关节角 − 撑地时关节角中位)。+ = 脚尖朝下。
另给出机身俯仰变化的贡献（IMU）。"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_clear as fc
import mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
adr = [m.jnt_qposadr[m.joint(n).id] for n in fc.J14]
gid = {s: m.geom(f'{s}_foot_collision').id for s in fc.SIDES}; base = m.body('trunk_base').id
def kin(q):
    d.qpos[:] = m.qpos0
    for a, v in zip(adr, q): d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    Rb = d.xmat[base].reshape(3, 3); pb = d.xpos[base]; out = {}
    for s in fc.SIDES:
        g = gid[s]; c = d.geom_xpos[g]; R = d.geom_xmat[g].reshape(3, 3); sz = m.geom_size[g]
        pts = np.array([c + R @ (np.array([a, b, e]) * sz) for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)])
        f = Rb.T @ R[:, int(np.argmax(sz))]; f = f if f[0] > 0 else -f
        out[s] = ((pts - pb) @ Rb, f)
    return out
def pitch(f): return math.degrees(math.atan2(-f[2], math.hypot(f[0], f[1])))
for spec in sys.argv[2:]:
    name, src = spec.split('=', 1)
    _, walk = fc.load(src)
    Q = np.array([q for q, _, _ in walk]); L = [fc.level(g) for _, _, g in walk]
    K = [kin(q) for q in Q]
    z = {s: np.array([float((k[s][0] @ l.T)[:, 2].min()) for k, l in zip(K, L)]) for s in fc.SIDES}
    st = np.where(z['left'] <= z['right'], 'left', 'right')
    print(f'\n## {name}   脚尖朝下(+)/朝上(-) 的来源（°）')
    for s in fc.SIDES:
        o = 'right' if s == 'left' else 'left'
        idx = np.flatnonzero(st == o); segs = [g for g in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1) if len(g) >= 3]
        pk = [g[np.argmax(z[s][g] - z[o][g])] for g in segs]
        q0 = np.median(Q[st == s], axis=0)
        tot = []; con = np.zeros(5); body = []
        p0 = pitch(kin(q0)[s][1])
        for j in pk:
            p_body = pitch(L[j] @ K[j][s][1]); p_rel = pitch(K[j][s][1])
            tot.append(p_body - np.median([pitch(L[k] @ K[k][s][1]) for k in np.flatnonzero(st == s)[::10]]))
            body.append(p_body - p_rel)
            for n, i in enumerate(fc.LEGIDX[s]):
                q1 = q0.copy(); q1[i] = Q[j, i]
                con[n] += pitch(kin(q1)[s][1]) - p0
        con /= max(len(pk), 1)
        bst = np.median([pitch(L[k] @ K[k][s][1]) - pitch(K[k][s][1]) for k in np.flatnonzero(st == s)[::10]])
        print(f'  {fc.CN[s]} 最高点比撑地时 {np.median(tot):+5.1f}°  =  ' + '  '.join(f'{fc.LEGNAME[n]} {con[n]:+5.1f}' for n in range(5))
              + f'  机身俯仰变化 {np.median(body) - bst:+5.1f}')
