"""看最近一段 walk（模型接管）：每隔一段打印倾斜、角速度、腿目标偏离 HOME、跟踪误差、摇杆指令；最后给倾斜均值和倒地帧数。
用法：python3 walk_now.py [run-xxxx.jsonl]"""
import json, glob, math, sys
f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob('/home/duck/r17/rec/run-*.jsonl'))[-1]
R = []
for l in open(f, errors='replace'):
    try: R.append(json.loads(l))
    except ValueError: pass
W = [i for i, r in enumerate(R) if r.get('policy') == 'walk']
if not W: print('还没进入 walk'); sys.exit()
# 取最后一段连续 walk
i1 = W[-1]; i0 = i1
while i0 - 1 in set(W): i0 -= 1
seg = R[i0:i1 + 1]; HOME = R[i0 - 1]['targets']; LEG = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14]
def tilt(g):
    n = math.sqrt(sum(x * x for x in g)); return math.degrees(math.asin(g[0] / n)), math.degrees(math.asin(-g[1] / n))
t0 = seg[0]['t']
print('这段 walk 时长', round(seg[-1]['t'] - t0, 1), 's；之后状态', R[-1]['policy'])
print(' 时间   前倾   右倾  |gyro|  腿目标偏离HOME  腿跟踪误差  摇杆')
step = max(1, len(seg) // 40)
for k in range(0, len(seg), step):
    r = seg[k]; p, rr = tilt(r['safety']['gravity']); w = math.sqrt(sum(x * x for x in r['imu']['gyro']))
    dev = max(abs(r['targets'][j] - HOME[j]) for j in LEG); err = max(abs(r['targets'][j] - r['joints'][j]) for j in LEG)
    cmd = [round(x, 2) for x in r['move']['requested']]
    print('%5.1fs %+6.1f %+6.1f  %5.2f   %6.1f°       %6.1f°    %s' % (r['t'] - t0, p, rr, w, math.degrees(dev), math.degrees(err), cmd))
ps = [tilt(r['safety']['gravity'])[0] for r in seg]
print('前倾 均值 %+.1f° 范围 %+.1f ~ %+.1f°；倒地帧 %d' % (sum(ps) / len(ps), min(ps), max(ps), sum(1 for r in seg if r['safety'].get('fallen'))))
