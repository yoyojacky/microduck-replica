"""找模型接管后「从平稳到失控」的转折点：腿部目标首次偏离 HOME 超过 15° 的时刻，前后逐帧打印倾斜、重力、角速度和偏离最大的关节。
用法：python3 failure_onset.py run-xxxx.jsonl（取第一段 policy=walk）"""
import json, sys, math
def _load(f):
    out=[]
    for l in open(f, errors='replace'):
        try: out.append(json.loads(l))
        except ValueError: pass
    return out
R=_load(sys.argv[1]); print('读到', len(R), '帧（坏行已跳过）')
i0=next(i for i,r in enumerate(R) if r.get('policy')=='walk'); i1=max(i for i,r in enumerate(R) if r.get('policy')=='walk')
home=R[i0-1]['targets']; LEG=[0,1,2,3,4,10,11,12,13,14]
N=['左髋偏','左髋横','左髋俯','左膝','左踝','颈','头俯','头偏','头横','嘴','右髋偏','右髋横','右髋俯','右膝','右踝']
def tilt(g): n=math.sqrt(sum(x*x for x in g)); return math.degrees(math.acos(max(-1,min(1,-g[2]/n))))
dev=lambda r: max(abs(r['targets'][j]-home[j]) for j in LEG)
ib=next((i for i in range(i0,i1) if math.degrees(dev(R[i]))>15), i0)
print('接管于 0s，总时长', round(R[i1]['t']-R[i0]['t'],1),'s；腿目标首次偏离 HOME 超 15° 在', round(R[ib]['t']-R[i0]['t'],2),'s')
print(' 时间   偏离  倾斜  g(x,y,z)                 gyro(x,y,z)            偏离最大的关节')
for i in range(max(i0-10,ib-60), min(i1+5,ib+40), 2):
    r=R[i]; d=[abs(r['targets'][j]-home[j]) for j in range(15)]; jm=max(LEG,key=lambda j:d[j])
    g=r['safety']['gravity']; w=r['imu']['gyro']
    print(f"{r['t']-R[i0]['t']:5.2f}s {math.degrees(dev(r)):5.1f}° {tilt(g):5.1f}° ({g[0]:+.2f},{g[1]:+.2f},{g[2]:+.2f})  ({w[0]:+.2f},{w[1]:+.2f},{w[2]:+.2f})  {N[jm]} 目标{math.degrees(r['targets'][jm]-home[jm]):+.0f}° 实际{math.degrees(r['joints'][jm]-home[jm]):+.0f}°")
