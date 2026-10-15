"""最近 N 分钟：每次推摇杆的一段（指令、持续、是否交替迈步、前进距离 odom、倾斜、有没有倒），以及 IMU 断线。"""
import json, glob, math, sys, subprocess
mins = float(sys.argv[1]) if len(sys.argv) > 1 else 8
R = []
for f in sorted(glob.glob('/home/duck/r17/rec/run-*.jsonl'))[-3:]:
    for l in open(f, errors='replace'):
        try: R.append(json.loads(l))
        except ValueError: pass
R.sort(key=lambda r: r['wall']); tend = R[-1]['wall']; R = [r for r in R if r['wall'] > tend - mins * 60]
def tilt(g):
    n = math.sqrt(sum(x * x for x in g)); return math.degrees(math.asin(g[0] / n)), math.degrees(math.asin(-g[1] / n))
pol = {}
for r in R: pol[r['policy']] = pol.get(r['policy'], 0) + 1
print('最近 %.0f 分钟：%d 帧，状态分布 %s' % (mins, len(R), {k: round(v / 50, 1) for k, v in pol.items()}), '（秒）')
# 分段：连续有摇杆指令的区间
segs = []; cur = None
for i, r in enumerate(R):
    on = r['policy'] == 'walk' and any(abs(x) > 0.02 for x in r['move']['requested'])
    if on and cur is None: cur = [i, i]
    elif on: cur[1] = i
    elif cur is not None:
        if i - cur[1] > 25: segs.append(cur); cur = None
if cur: segs.append(cur)
print('推摇杆 %d 段' % len(segs))
for a, b in segs:
    S = R[a:b + 1]; dur = S[-1]['t'] - S[0]['t']
    req = [r['move']['requested'] for r in S]
    vx = max(req, key=lambda v: abs(v[0]))[0]; vy = max(req, key=lambda v: abs(v[1]))[1]; wz = max(req, key=lambda v: abs(v[2]))[2]
    o0 = S[0].get('odom', {}).get('position', [0, 0, 0]); o1 = S[-1].get('odom', {}).get('position', [0, 0, 0])
    dist = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    # 交替迈步：左右膝角(右侧取反)之差的过零次数
    d = [math.degrees(r['joints'][3] + r['joints'][13]) for r in S]; m = sum(d) / len(d)
    zc = sum(1 for x, y in zip(d, d[1:]) if (x - m) * (y - m) < 0)
    lift = (max(d) - min(d))
    ps = [tilt(r['safety']['gravity'])[0] for r in S]; fall = sum(1 for r in S if r['safety'].get('fallen'))
    print('  %s 持续 %4.1fs  最大指令 vx %+.2f vy %+.2f wz %+.2f  odom 走了 %.2f m  左右膝差摆幅 %.0f° 换腿约 %d 次  前倾 %+.1f~%+.1f°%s' % (
        __import__('datetime').datetime.fromtimestamp(S[0]['wall']).strftime('%H:%M:%S'), dur, vx, vy, wz, dist, lift, zc // 2, min(ps), max(ps), '  倒地!' if fall else ''))
out = subprocess.run(['journalctl', '-u', 'robotd', '--since', '-%d min' % int(mins + 1), '--no-pager'], capture_output=True, text=True).stdout
print('同期 IMU 断线 %d 次；倒地判定 %d 次' % (out.count('reader fault'), out.count('fallen=true')))
