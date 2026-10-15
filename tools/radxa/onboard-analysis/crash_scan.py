"""每段录像：状态切换时间线、摇杆指令、最后几帧安全状态；配合同一段的 health CSV 看电压/温度。只读。
时间用录像里的 wall（墙钟）：duck_rec 在 robotd 重启后会重连，robotd 自己的 t 会归零。
用法：python3 crash_scan.py run-xxx.jsonl [health-xxx.csv]"""
import csv, json, math, sys

R = []
for l in open(sys.argv[1], errors="replace"):
    try:
        d = json.loads(l)
    except ValueError:
        continue
    if 'rec_error' in d:
        print('  rec_error', d)
        continue
    if 'wall' in d and 't' in d:
        R.append(d)
if not R:
    sys.exit('empty')
w0 = R[0]['wall']
print(f"{sys.argv[1]}: {len(R)} 帧, {R[-1]['wall'] - w0:.1f} s, wall {w0:.0f}->{R[-1]['wall']:.0f}")


def req(r):
    return (r.get('move') or {}).get('requested', [0, 0, 0])


prev = None
for r in R:
    s = r.get('safety', {})
    key = (r.get('policy'), s.get('limp'), s.get('fallen'))
    if key != prev:
        print(f"  {r['wall'] - w0:7.2f}s policy={r.get('policy')} limp={s.get('limp')} fallen={s.get('fallen')} 摇杆={req(r)}")
        prev = key
moving = [r for r in R if any(abs(v) > 0.01 for v in req(r))]
if moving:
    print(f"  摇杆有指令的帧: {len(moving)}，第一次 {moving[0]['wall'] - w0:.2f}s 最后一次 {moving[-1]['wall'] - w0:.2f}s 样例 {req(moving[-1])}")
print('最后 3 帧:')
for r in R[-3:]:
    s = dict(r.get('safety', {}))
    g = s.pop('gravity', None) or [0, 0, -1]
    n = math.sqrt(sum(x * x for x in g)) or 1
    tilt = math.degrees(math.acos(max(-1, min(1, -g[2] / n))))     # 站直时重力朝下（g_z<0），倾角 0°
    print(f"  {r['wall'] - w0:7.2f}s policy={r.get('policy')} loop={r.get('loop')} 倾角={tilt:.1f}° 摇杆={req(r)} safety={s}")
if len(sys.argv) > 2:
    rows = list(csv.reader(open(sys.argv[2])))
    print('health:', rows[0])
    print(f"  {len(rows) - 1} 行；最后 6 行:")
    for row in rows[-6:]:
        print('  ', row)
    try:
        vi = rows[0].index(next(h for h in rows[0] if 'volt' in h.lower()))
        vs = [float(x[vi]) for x in rows[1:] if x[vi] not in ('', 'None')]
        print(f"  电压 最低 {min(vs):.2f} 最高 {max(vs):.2f}")
    except Exception as e:
        print('  电压列解析失败', e)
