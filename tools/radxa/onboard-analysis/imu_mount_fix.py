"""电脑上运行：按 sim_home_tilt.py 算出的「IMU 安装偏差」，给出新的安装四元数。
robotd 的约定（duck-control/src/imu.rs）：安装四元数 M 是 [w,x,y,z]，表示传感器→机身；机身重力 = M·g传感器·M⁻¹。
补偿就是在前面再乘一个小旋转：M' = C ⊗ M，C 把机身系读数绕 +Y 转「前倾偏差」、绕 +X 转「右倾偏差」。
用法：python imu_mount_fix.py <前倾偏差°> <右倾偏差°> [当前 M 的 w x y z]
例（2026-10-12 第一台）：python imu_mount_fix.py 8.54 1.78 0.5 0.5 0.5 0.5
  → [0.454164155, 0.544101455, 0.527409117, 0.468538313]"""
import math, sys
def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)
def rot(q, v):
    w, x, y, z = q; r = qmul(qmul(q, (0.0,) + tuple(v)), (w, -x, -y, -z)); return r[1:]
def ang(g):
    n = math.sqrt(sum(c * c for c in g)); return math.degrees(math.asin(g[0] / n)), math.degrees(math.asin(-g[1] / n))
dp, dr = float(sys.argv[1]), float(sys.argv[2])
M = tuple(float(x) for x in sys.argv[3:7]) if len(sys.argv) >= 7 else (0.5, 0.5, 0.5, 0.5)
C = qmul((math.cos(math.radians(dp) / 2), 0, math.sin(math.radians(dp) / 2), 0), (math.cos(math.radians(dr) / 2), math.sin(math.radians(dr) / 2), 0, 0))
N = qmul(C, M); n = math.sqrt(sum(c * c for c in N)); N = tuple(c / n for c in N)
print('新安装四元数 [w,x,y,z] = [%.9f, %.9f, %.9f, %.9f]' % N)
# 自检：取一个「旧参数下读到前倾 dp、右倾 dr」的重力，换新参数后应读到 0 / 0
gx = math.sin(math.radians(dp)); gyv = -math.sin(math.radians(dr)); g_old = (gx, gyv, -math.sqrt(1 - gx * gx - gyv * gyv))
g_sensor = rot((M[0], -M[1], -M[2], -M[3]), g_old)
print('自检：旧参数读 前倾 %+.2f° 右倾 %+.2f° → 新参数读 前倾 %+.2f° 右倾 %+.2f°（应接近 0）' % ((dp, dr) + ang(rot(N, g_sensor))))
print('写进 /etc/robot/feetech-ft5/hd1910-calibration.toml 的 ["imu_mounts"."bno08x"] → "quat"，先备份；robotd 要在卸力状态下重启才生效。')
