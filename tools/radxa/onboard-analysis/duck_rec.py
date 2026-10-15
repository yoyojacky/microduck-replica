"""后台录 robotd 50 Hz 状态（关节、目标、速度指令、IMU、安全、策略状态），只读订阅，不发任何控制。
每秒 fsync 一次：拔电最多丢最后 1 秒。robotd 断开时记一行 rec_error 并重连。追加写，不覆盖旧文件。
用法：python3 duck_rec.py 秒数 输出文件"""
import json, os, socket, sys, time
secs = float(sys.argv[1]); out = sys.argv[2]
keep = ('t', 'policy', 'move', 'joints', 'targets', 'velocities', 'imu', 'safety', 'loop')
t0 = time.monotonic()
with open(out, 'a') as f:
    def sync():
        f.flush(); os.fsync(f.fileno())
    while time.monotonic() - t0 < secs:
        try:
            s = socket.socket(socket.AF_UNIX); s.settimeout(5); s.connect('/run/robotd.sock')
            s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{}}\n')
            r = s.makefile('rb'); last = time.monotonic()
            while time.monotonic() - t0 < secs:
                line = r.readline()
                if not line:
                    raise OSError('robotd closed the subscription')
                try:
                    m = json.loads(line); p = m.get('params', m.get('result'))
                except ValueError:
                    continue
                if not isinstance(p, dict) or 'joints' not in p:
                    continue
                p = {k: p[k] for k in keep if k in p}; p['wall'] = time.time()
                f.write(json.dumps(p, separators=(',', ':')) + '\n')
                if time.monotonic() - last >= 1:
                    sync(); last = time.monotonic()
        except (OSError, socket.timeout) as e:
            f.write(json.dumps({'wall': time.time(), 'rec_error': f'{type(e).__name__}: {e}'}) + '\n'); sync()
            time.sleep(1)
