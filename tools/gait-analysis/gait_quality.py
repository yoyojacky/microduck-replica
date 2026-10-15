"""走路质量分析（只读）：从 duck_rec 的 50 Hz 录像里找出「颤、晃」来自哪里。
用法：python gait_quality.py run-xxx.jsonl [报告.txt]
分段：policy=walk 且指令速度 hypot(vx, vy)>0.02 记为「走」，policy=walk 且指令为 0 记为「原地」。
录像里 robotd 重启过（t 回退或有 rec_error 行）时在那里切段，不跨段。
指标：
  1. 目标抖动：关节目标逐拍变化量（deg/拍）的 RMS，和 >4 Hz 高频能量占比 —— 模型输出本身抖不抖
  2. 跟踪：目标 − 实际 的 RMS，和互相关估计的滞后（ms）—— 舵机跟不跟得上
  3. 躯干：陀螺仪 RMS、主频；重力方向前后/左右摆幅 —— 机身晃不晃、晃多快
  4. 步频：左右膝差的主频
  5. 时序：相邻帧间隔的抖动、丢拍
"""
import json, math, sys
import numpy as np

NAMES = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee', 'L_ankle', 'neck_pitch', 'head_pitch',
         'head_yaw', 'head_roll', 'mouth', 'R_hip_yaw', 'R_hip_roll', 'R_hip_pitch', 'R_knee', 'R_ankle']
LEGS = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14]
out = open(sys.argv[2], 'w', encoding='utf-8') if len(sys.argv) > 2 else None
def p(*a):
    s = ' '.join(str(x) for x in a); print(s)
    if out: out.write(s + '\n')

R = []
BREAK = {'_break': True}            # robotd 重启：t 回退或录像里记了 rec_error，段在这里断开
for l in open(sys.argv[1], encoding='utf-8', errors='replace'):
    try: d = json.loads(l)
    except ValueError: continue
    if 'rec_error' in d or (R and R[-1] is not BREAK and 't' in d and d['t'] < R[-1]['t']):
        if R and R[-1] is not BREAK: R.append(BREAK)
        if 'rec_error' in d: continue
    if 't' in d and d.get('joints') and d.get('targets'): R.append(d)
p(f'文件 {sys.argv[1]}：{sum(r is not BREAK for r in R)} 帧，robotd 重启 {sum(r is BREAK for r in R)} 次')

def segments(pred, min_len=50):
    segs, cur = [], []
    for r in R:
        if r is not BREAK and pred(r): cur.append(r)
        else:
            if len(cur) >= min_len: segs.append(cur)
            cur = []
    if len(cur) >= min_len: segs.append(cur)
    return segs
vx = lambda r: (r.get('move') or {}).get('requested', [0, 0, 0])
walking = segments(lambda r: r.get('policy') == 'walk' and math.hypot(vx(r)[0], vx(r)[1]) > 0.02, 50)
standing = segments(lambda r: r.get('policy') == 'walk' and math.hypot(vx(r)[0], vx(r)[1]) <= 0.02, 100)
held = segments(lambda r: r.get('policy') == 'held', 100)

def spectrum(x, fs=50.0):
    x = np.asarray(x) - np.mean(x)
    if len(x) < 32: return None, None
    w = np.hanning(len(x)); F = np.abs(np.fft.rfft(x * w)) ** 2; f = np.fft.rfftfreq(len(x), 1 / fs)
    return f, F

def analyse(name, segs):
    if not segs:
        p(f'\n## {name}：没有这种片段'); return
    T = np.concatenate([np.array([r['targets'] for r in s]) for s in segs])
    J = np.concatenate([np.array([r['joints'] for r in s]) for s in segs])
    G = np.concatenate([np.array([r['imu']['gyro'] for r in s]) for s in segs])
    g = np.concatenate([np.array([r['safety']['gravity'] for r in s]) for s in segs])
    t = np.concatenate([np.array([r['t'] for r in s]) for s in segs])
    dur = sum(s[-1]['t'] - s[0]['t'] for s in segs)
    cmd = np.concatenate([np.array([vx(r) for r in s]) for s in segs])
    p(f'\n## {name}：{len(segs)} 段，共 {dur:.1f} s，指令 vx 平均 {cmd[:, 0].mean():.2f} m/s，vy {cmd[:, 1].mean():.2f}，yaw {cmd[:, 2].mean():.2f}')
    # 5. timing
    dts = np.concatenate([np.diff([r['t'] for r in s]) for s in segs]) * 1000
    p(f'  时序：帧间隔 {dts.mean():.1f} ms，标准差 {dts.std():.2f} ms，最大 {dts.max():.1f} ms，>30 ms 的 {int((dts > 30).sum())} 次')
    # 1. target jitter
    p('  关节          目标逐拍变化RMS  目标>4Hz能量%  跟踪误差RMS  滞后ms  实际>4Hz能量%')
    for j in LEGS:
        dT = np.concatenate([np.diff(np.array([r['targets'][j] for r in s])) for s in segs])
        rms_step = math.degrees(np.sqrt(np.mean(dT ** 2)))
        hf = []; hfj = []; lags = []
        for s in segs:
            x = np.array([r['targets'][j] for r in s]); y = np.array([r['joints'][j] for r in s])
            f, F = spectrum(x)
            if f is not None and F[f > 0.3].sum() > 0: hf.append(F[f > 4].sum() / F[f > 0.3].sum())
            f2, F2 = spectrum(y)
            if f2 is not None and F2[f2 > 0.3].sum() > 0: hfj.append(F2[f2 > 4].sum() / F2[f2 > 0.3].sum())
            if len(x) > 60:
                xc = x - x.mean(); yc = y - y.mean()
                cc = [np.dot(xc[:len(xc) - k], yc[k:]) for k in range(0, 15)]
                lags.append(int(np.argmax(cc)) * 20)
        err = math.degrees(np.sqrt(np.mean((T[:, j] - J[:, j]) ** 2)))
        p(f'  {NAMES[j]:<13} {rms_step:8.2f}°        {100*np.mean(hf) if hf else float("nan"):6.1f}        {err:6.2f}°     {np.median(lags) if lags else float("nan"):5.0f}    {100*np.mean(hfj) if hfj else float("nan"):6.1f}')
    # 3. trunk
    pitch = np.degrees(np.arcsin(np.clip(g[:, 0] / np.linalg.norm(g, axis=1), -1, 1)))
    roll = np.degrees(np.arcsin(np.clip(-g[:, 1] / np.linalg.norm(g, axis=1), -1, 1)))
    p(f'  躯干：前后倾 均值 {pitch.mean():+.1f}° 摆幅(5–95%) {np.percentile(pitch,95)-np.percentile(pitch,5):.1f}°；左右倾 均值 {roll.mean():+.1f}° 摆幅 {np.percentile(roll,95)-np.percentile(roll,5):.1f}°')
    for k, ax in enumerate('xyz'):
        peaks = []
        for s in segs:
            f, F = spectrum([r['imu']['gyro'][k] for r in s])
            if f is not None:
                m = f > 0.3; peaks.append((f[m][np.argmax(F[m])], F[m & (f > 4)].sum() / F[m].sum()))
        if peaks:
            p(f'  陀螺 {ax}：RMS {np.degrees(np.sqrt(np.mean(G[:, k] ** 2))):6.1f}°/s，主频 {np.median([a for a, _ in peaks]):.2f} Hz，>4Hz 能量 {100*np.mean([b for _, b in peaks]):.0f}%')
    # 4. step frequency from knee difference
    fr = []
    for s in segs:
        d = np.array([r['joints'][3] - r['joints'][13] for r in s])
        f, F = spectrum(d)
        if f is not None:
            m = (f > 0.5) & (f < 5); fr.append(f[m][np.argmax(F[m])])
    if fr: p(f'  步频（左右膝差主频）：{np.median(fr):.2f} Hz；膝差摆幅 {math.degrees(np.ptp(np.concatenate([[r["joints"][3]-r["joints"][13] for r in s] for s in segs]))):.0f}°')

analyse('站 HOME（held）', held)
analyse('模型接管、原地（指令 0）', standing)
analyse('模型接管、在走（指令 > 0）', walking)
