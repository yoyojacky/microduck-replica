"""用录像检验 IMU 自洽：机体系重力 g 的变化率应满足 dg/dt = -ω × g（ω 为机体系角速度）。
对比几种可能的错误（ω 取反、各轴取反、轴互换），哪一种和实测最吻合，就说明 IMU 实际是哪种情况。"""
import json, sys, math, itertools
R=[]
for _l in open(sys.argv[1], errors='replace'):
    try: R.append(json.loads(_l))
    except ValueError: pass
R=[r for r in R if r.get('policy') in ('walk','homing') or True]
g=[r['safety']['gravity'] for r in R]; w=[r['imu']['gyro'] for r in R]; t=[r['t'] for r in R]
def cross(a,b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
pairs=[]
for i in range(1,len(R)-1):
    dt=t[i+1]-t[i-1]
    if not 0.03<dt<0.06: continue
    dg=[(g[i+1][k]-g[i-1][k])/dt for k in range(3)]
    if math.sqrt(sum(x*x for x in dg))<0.3: continue          # 只看明显在转的时刻
    pairs.append((dg,w[i],g[i]))
print('有效样本', len(pairs), '（|dg/dt|>0.3/s）')
def score(T):
    num=0; den1=0; den2=0
    for dg,wi,gi in pairs:
        pred=[-x for x in cross(T(wi),gi)]
        num+=sum(a*b for a,b in zip(dg,pred)); den1+=sum(a*a for a in dg); den2+=sum(b*b for b in pred)
    return num/math.sqrt(den1*den2) if den1 and den2 else float('nan')
cands={'原样 ω':lambda v:v,'ω 整体取反':lambda v:[-v[0],-v[1],-v[2]]}
for perm in itertools.permutations(range(3)):
    for signs in itertools.product((1,-1),repeat=3):
        name=f"ω'=({','.join(('-' if s<0 else '+')+'xyz'[p] for p,s in zip(perm,signs))})"
        cands[name]=(lambda perm,signs:lambda v:[signs[k]*v[perm[k]] for k in range(3)])(perm,signs)
res=sorted(((score(T),n) for n,T in cands.items()),reverse=True)
print('原样 ω 的吻合度（1=完全一致，-1=完全相反）：', round(score(cands['原样 ω']),3))
print('最吻合的 5 种：'); [print('  ',round(s,3),n) for s,n in res[:5]]
# 站姿时的重力
held=[r['safety']['gravity'] for r in R if r.get('policy')=='held'][-200:]
if held: print('站着时重力均值', [round(sum(x[k] for x in held)/len(held),3) for k in range(3)])
