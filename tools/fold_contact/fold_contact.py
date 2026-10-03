# -*- coding: utf-8 -*-
"""腿关节「转到零件碰上」的角度 + 官方姿势会不会让零件顶在一起。

为零位标定找机械参考点用：鸭子缩着的时候哪个关节是被什么顶住的、在多少度。
MuJoCo 只负责正运动学（每个零件在哪），碰没碰上用 fcl 按原始三角网格精确求交 ——
MuJoCo 自己的碰撞把网格当凸包，比实物"胖"，算出来的接触角会偏小。

  python fold_contact.py --mjcf <microduck_rl>/src/mjlab_microduck/robot/microduck/robot_walk.xml
  python fold_contact.py --mjcf ... --json out.json

依赖：mujoco、python-fcl、numpy（pip install mujoco python-fcl numpy）。
零位（腿伸直）时就互相穿插的几何体对当作装配关系（舵机装在支架里、轴承装在座里），扫描时排除。
官方模型是 XL330 原版的几何；飞特版改过 8 个配合件，数值要拿飞特版的模型再算一遍。
"""
import argparse
import json
import math
import sys
import time

import fcl
import mujoco
import numpy as np

# microduck_rl scene.xml 的 keyframe（d424a0c），14 个关节按 MJCF 顺序
JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
KEYFRAMES = {
    "STAND": [0, -0.08726646259971647, -0.457924, -0.004940, 0.452984,
              0.3490658503988659, 0.3490658503988659, 0, 0,
              0, 0.08726646259971647, 0.457924, 0.004940, -0.452984],
    "SIT": [0, 0, -0.5236, 1.0472, 0, 0.5, 1.6, 0, 0, 0, 0, 0.5236, -1.0472, 0],
    "FOLD": [0, 0, 1.57, 1.57, 0, 1, 1, 0, 0, 0, 0, -1.57, -1.57, 0],
}


class Model:
    def __init__(self, path):
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        m = self.m
        # 只用可视网格（group 2）：跟实物外形一致；group 3 是训练用的简化碰撞体
        self.geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                      and m.geom_group[g] == 2 and m.geom_dataid[g] >= 0]
        self.bvh = {}
        for g in self.geoms:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(np.float64)
            f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]].astype(np.int32)
            b = fcl.BVHModel()
            b.beginModel(len(v), len(f))
            b.addSubModel(v, f)
            b.endModel()
            self.bvh[g] = b
        self.q0 = m.qpos0.copy()
        self.baseline = {p for p in self.pairs(self.q0, self.geoms, self.geoms)}

    def name(self, obj, i):
        return mujoco.mj_id2name(self.m, obj, i)

    def label(self, g):
        m = self.m
        return f"{self.name(mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])}/{self.name(mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])}"

    def qadr(self, joint):
        return self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, joint)]

    def with_joints(self, values, base=None):
        q = (self.q0 if base is None else base).copy()
        for j, v in values.items():
            q[self.qadr(j)] = v
        return q

    def objects(self, q):
        self.d.qpos[:] = q
        mujoco.mj_kinematics(self.m, self.d)
        return {g: fcl.CollisionObject(self.bvh[g], fcl.Transform(self.d.geom_xmat[g].reshape(3, 3).copy(),
                                                                  self.d.geom_xpos[g].copy()))
                for g in self.geoms}

    def pairs(self, q, set_a, set_b, skip=()):
        """q 姿势下 set_a × set_b 里跨零件（不同 body）穿插的几何体对。"""
        objs = self.objects(q)
        out = set()
        for a in set_a:
            for b in set_b:
                p = (min(a, b), max(a, b))
                if a == b or p in out or p in skip or self.m.geom_bodyid[a] == self.m.geom_bodyid[b]:
                    continue
                if fcl.collide(objs[a], objs[b], fcl.CollisionRequest(), fcl.CollisionResult()) > 0:
                    out.add(p)
        return out

    def subtree_geoms(self, joint):
        m = self.m
        root = m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)]
        bodies = {root}
        grew = True
        while grew:
            grew = False
            for b in range(1, m.nbody):
                if m.body_parentid[b] in bodies and b not in bodies:
                    bodies.add(b)
                    grew = True
        return {g for g in self.geoms if m.geom_bodyid[g] in bodies}

    def sweep(self, joint, sign, base, max_deg=180.0, step=0.25, tol=0.05):
        """从 base 起把 joint 往 sign 方向转，返回 (第一次碰上的角度°, 碰上的零件)；转满 max_deg 没碰上返回 None。"""
        distal = self.subtree_geoms(joint)
        proximal = set(self.geoms) - distal
        adr = self.qadr(joint)
        start = base[adr]

        def hits(deg):
            q = base.copy()
            q[adr] = start + sign * math.radians(deg)
            return self.pairs(q, distal, proximal, self.baseline)

        prev, a = 0.0, 0.0
        while a <= max_deg:
            if hits(a):
                lo, hi = prev, a
                while hi - lo > tol:
                    mid = (lo + hi) / 2
                    if hits(mid):
                        hi = mid
                    else:
                        lo = mid
                who = sorted({(self.label(x), self.label(y)) for x, y in hits(hi)})
                return math.degrees(start) + sign * hi, who
            prev, a = a, a + step
        return None, []


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mjcf", required=True, help="robot_walk.xml（或同结构的模型）")
    ap.add_argument("--json", help="结果另存 JSON")
    a = ap.parse_args()
    t0 = time.time()
    mdl = Model(a.mjcf)
    out = {"mjcf": a.mjcf, "baseline_pairs": sorted((mdl.label(x), mdl.label(y)) for x, y in mdl.baseline),
           "sweeps": {}, "keyframes": {}}
    print(f"零位时就穿插的装配件对 {len(mdl.baseline)} 个（舵机在支架里、轴承在座里），扫描时排除")

    def record(key, res):
        ang, who = res
        out["sweeps"][key] = {"deg": None if ang is None else round(ang, 2), "contact": who[:3]}
        print(f"  {key}：" + ("转满 180° 没碰上" if ang is None else f"{ang:+.2f}°  碰上 {who[:2]}"))

    for side, s in (("left", +1), ("right", -1)):   # s = 官方 FOLD 的折叠方向
        print(f"== {side} 腿（折叠方向 {s:+d}）")
        knee = mdl.sweep(f"{side}_knee", s, mdl.q0)
        record(f"{side}.膝折叠（髋、踝 0°）", knee)
        base = mdl.with_joints({f"{side}_hip_pitch": s * 1.57})
        record(f"{side}.膝折叠（髋 {s * 90:+d}°、踝 0°，即官方 FOLD 的髋）", mdl.sweep(f"{side}_knee", s, base))
        if knee[0] is not None:
            qk = mdl.with_joints({f"{side}_knee": math.radians(knee[0] - s * 0.1)})
            hip = mdl.sweep(f"{side}_hip_pitch", s, qk)
            record(f"{side}.髋 pitch 折叠（膝已折到底）", hip)
            if hip[0] is not None:
                qh = qk.copy()
                qh[mdl.qadr(f"{side}_hip_pitch")] = math.radians(hip[0] - s * 0.1)
                record(f"{side}.踝 +（膝、髋都折到底）", mdl.sweep(f"{side}_ankle", +1, qh))
                record(f"{side}.踝 −（膝、髋都折到底）", mdl.sweep(f"{side}_ankle", -1, qh))

    # 颈：官方 FOLD 的颈 / 头 pitch 都是 1 rad（57°）。看它是不是接触点：从腿折叠、头颈 0° 起把颈往 FOLD 的方向（+）转
    print("== 颈（腿按官方 FOLD 折好）")
    fold = mdl.with_joints(dict(zip(JOINTS, KEYFRAMES["FOLD"])))
    for head, rad in (("0°", 0.0), ("57°，官方 FOLD 的头", 1.0)):
        q = fold.copy()
        q[mdl.qadr("neck_pitch")] = 0.0
        q[mdl.qadr("head_pitch")] = rad
        record(f"颈 pitch 往 FOLD 方向（头 pitch {head}）", mdl.sweep("neck_pitch", +1, q))

    print("== 官方姿势：零件会不会顶在一起")
    for key, vals in KEYFRAMES.items():
        q = mdl.with_joints(dict(zip(JOINTS, vals)))
        bad = mdl.pairs(q, mdl.geoms, mdl.geoms, mdl.baseline)
        who = sorted({(mdl.label(x), mdl.label(y)) for x, y in bad})
        out["keyframes"][key] = who
        print(f"  {key}：" + (f"穿插 {len(who)} 对 {who[:3]}" if who else "不穿插"))
    print(f"用时 {time.time() - t0:.0f} s")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
