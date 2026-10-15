"""电脑上运行（要 mujoco 和官方 microduck_rl 的 MJCF）。运动学法：以 MJCF 的 STAND 关键帧（仿真里站立、脚掌平贴地面）为“脚平”参考，
换成真机关节角后，令左右脚掌姿态保持与参考相同（仍平贴地面），反推机身姿态，读 imu 站点的机体系重力。
用法：python sim_home_tilt.py <microduck_rl>/src/mjlab_microduck/robot/microduck/scene_backlash.xml home-capture.json
输出「IMU 安装偏差（实测 − 应有）」，交给 imu_mount_fix.py。"""
import json, sys, math
import numpy as np, mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m); real = json.load(open(sys.argv[2]))
ORDER15 = ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle','neck_pitch','head_pitch','head_yaw','head_roll','mouth',
           'right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle']
HOME14 = dict(zip([n for n in ORDER15 if n!='mouth'], [0.0,-0.0873,-0.4579,-0.0049,0.453,0.3491,0.3491,0.0,0.0,0.0,0.0873,0.4579,0.0049,-0.453]))
bodies = [m.body(i).name for i in range(m.nbody)]
feet = [b for b in bodies if 'foot' in b.lower()] or [b for b in bodies if 'ankle' in b.lower()]
print('脚部 body：', feet)
def fk(qpos):
    d.qpos[:] = qpos; mujoco.mj_kinematics(m, d)
    return {b: d.body(b).xmat.reshape(3,3).copy() for b in feet + ['trunk_base' if 'trunk_base' in bodies else bodies[1]]}, d.site('imu').xmat.reshape(3,3).copy()
k = m.key('STAND'); q0 = k.qpos.copy()
print('STAND 关键帧机身四元数', np.round(q0[3:7],4), ' 各关节(°):', {m.joint(i).name: round(math.degrees(q0[m.jnt_qposadr[i]]),1) for i in range(m.njnt) if m.jnt_type[i]!=0 and 'passive' not in m.joint(i).name})
# 参考：STAND 时脚掌在世界系的姿态（假定此时平贴地面）
ref, imu_ref = fk(q0)
g = lambda Rimu: Rimu.T @ np.array([0,0,-1.0])
def ang(v): n=np.linalg.norm(v); return math.degrees(math.asin(v[0]/n)), math.degrees(math.asin(-v[1]/n))
print('STAND 关键帧本身：imu 前倾 %+.2f° 右倾 %+.2f°' % ang(g(imu_ref)))
def tilt_for(qmap):
    q = q0.copy(); q[3:7] = [1,0,0,0]
    for i in range(m.njnt):
        if m.jnt_type[i]==0: continue
        n = m.joint(i).name
        q[m.jnt_qposadr[i]] = qmap.get(n, 0.0) if not n.startswith('passive') else 0.0
    Rs, Rimu = fk(q)   # 机身水平时的各部件姿态
    out = []
    for f in feet:
        # 要求 R_world_base * R_base_foot = R_ref_foot  ⇒  R_world_base = R_ref_foot * R_base_foot^T（此处 base 水平所以 xmat 即 R_base_x）
        Rwb = ref[f] @ Rs[f].T
        out.append(ang(g(Rwb @ Rimu)))
    return out
for label, qmap in (('理想 HOME', HOME14), ('真机此刻关节角', {n: real['joints15'][i] for i,n in enumerate(ORDER15)})):
    res = tilt_for(qmap)
    print(f"{label}：按左脚平 前倾 {res[0][0]:+.2f}° 右倾 {res[0][1]:+.2f}° ｜ 按右脚平 前倾 {res[1][0]:+.2f}° 右倾 {res[1][1]:+.2f}°")
pr, rr = ang(np.array(real['gravity']))
res = tilt_for({n: real['joints15'][i] for i,n in enumerate(ORDER15)})
ps = sum(r[0] for r in res)/len(res); rs = sum(r[1] for r in res)/len(res)
print(f"真机 IMU 实测：前倾 {pr:+.2f}° 右倾 {rr:+.2f}°")
print(f"⇒ IMU 安装偏差（实测 − 应有）：前倾 {pr-ps:+.2f}°  右倾 {rr-rs:+.2f}°")
