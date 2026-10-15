"""每个关节：角度的差分（实际转速） vs robotd 报的速度。比值≈+1 正常，≈-1 号反了，偏离 1 很多说明比例错。"""
import json, sys, math
R=[]
for _l in open(sys.argv[1], errors='replace'):
    try: R.append(json.loads(_l))
    except ValueError: pass
NAMES=['左髋偏航','左髋横滚','左髋俯仰','左膝','左踝','颈俯仰','头俯仰','头偏航','头横滚','嘴','右髋偏航','右髋横滚','右髋俯仰','右膝','右踝']
print(f"{'关节':6s} 样本  相关系数  比例(报/实)")
for j,n in enumerate(NAMES):
    xs=[];ys=[]
    for i in range(1,len(R)-1):
        dt=R[i+1]['t']-R[i-1]['t']
        if not 0.03<dt<0.06: continue
        d=(R[i+1]['joints'][j]-R[i-1]['joints'][j])/dt; v=R[i]['velocities'][j]
        if abs(d)>0.3: xs.append(d); ys.append(v)
    if len(xs)<20: print(f"{n:6s} {len(xs):4d}  （动得太少）"); continue
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    cov=sum((a-mx)*(b-my) for a,b in zip(xs,ys)); sx=math.sqrt(sum((a-mx)**2 for a in xs)); sy=math.sqrt(sum((b-my)**2 for b in ys))
    ratio=sum(a*b for a,b in zip(xs,ys))/sum(a*a for a in xs)
    print(f"{n:6s} {len(xs):4d}  {cov/(sx*sy):+.3f}    {ratio:+.2f}")
