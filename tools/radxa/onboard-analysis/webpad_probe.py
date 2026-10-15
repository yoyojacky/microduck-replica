"""在主板本机测网页手柄链路：连 webpad 套接字 → 发中性帧（无按键、摇杆归零）→ 看 padd 是否认到虚拟手柄。不发任何按键。"""
import json, socket, time, sys
t0 = time.monotonic()
def ms(): return f"{(time.monotonic()-t0)*1000:7.1f} ms"
s = socket.socket(socket.AF_UNIX); s.settimeout(3)
s.connect('/run/microduck-webpad/input.sock'); print(ms(), "connect 成功")
r = s.makefile('rb')
def call(req):
    a = time.monotonic(); s.sendall((json.dumps(req)+'\n').encode()); line = r.readline()
    return (time.monotonic()-a)*1000, json.loads(line)
dt, ans = call({'action': 'status'}); print(ms(), f"status 回执 {dt:.1f} ms", {k: ans.get(k) for k in ('accepted','source','real_connected','web_active')})
owner = 'claude-probe-20261012'
for i in range(15):   # 3 秒中性帧，每 0.2 s 一帧（webpad 租约 0.6 s）
    dt, ans = call({'action': 'state', 'owner': owner, 'frame': {'axes': {}, 'buttons': []}})
    if i in (0, 1, 2, 5, 14): print(ms(), f"state#{i} 回执 {dt:.1f} ms accepted={ans.get('accepted')} reason={ans.get('reason','')} source={ans.get('source')}")
    time.sleep(0.2)
dt, ans = call({'action': 'close', 'owner': owner}); print(ms(), f"close 回执 {dt:.1f} ms accepted={ans.get('accepted')}")
