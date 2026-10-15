"""从录像最后 N 秒找出手动倾斜的各段：每 0.5 s 打印机身前后/左右倾角，并自动挑出偏离站姿最大的几段。
约定（机体系 x 前、y 左、z 上；重力 g 在机体系）：前倾 → g_x 变正；往右倾 → g_y 变负。"""
import json, sys, math, glob
secs=float(sys.argv[1]) if len(sys.argv)>1 else 120
f=sorted(glob.glob('/home/duck/r17/rec/run-*.jsonl'))[-1]
R=[]
for l in open(f, errors='replace'):
    try: R.append(json.loads(l))
    except ValueError: pass
R=[r for r in R if r['t']>=R[-1]['t']-secs]
def ang(g):
    n=math.sqrt(sum(x*x for x in g)) or 1
    return math.degrees(math.asin(max(-1,min(1,g[0]/n)))), math.degrees(math.asin(max(-1,min(1,-g[1]/n))))  # (前倾+, 右倾+)
base=[sum(r['safety']['gravity'][k] for r in R[:100])/100 for k in range(3)]
b=ang(base); print(f"开头基准：前倾 {b[0]:+.1f}°  右倾 {b[1]:+.1f}°")
print(" 时间    前倾(+)/后仰(-)   右倾(+)/左倾(-)   gyro(x,y,z)")
for i in range(0,len(R),25):
    r=R[i]; a=ang(r['safety']['gravity']); w=r['imu']['gyro']
    mark='  ←' if abs(a[0]-b[0])>8 or abs(a[1]-b[1])>8 else ''
    print(f"{r['t']-R[0]['t']:6.1f}s   {a[0]-b[0]:+6.1f}°          {a[1]-b[1]:+6.1f}°        ({w[0]:+.2f},{w[1]:+.2f},{w[2]:+.2f}){mark}")
