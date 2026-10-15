"""每 0.5 s 记一次：舵机供电电压、最热舵机、CPU 温度和频率（robotctl health --json），
加 WiFi（wlan0 状态、信号）和已连接的蓝牙设备数。CSV，追加写，每行 fsync。只读。"""
import json, os, subprocess, sys, time
secs = float(sys.argv[1]); out = sys.argv[2]; t0 = time.time()
def wifi():
    try:
        st = open('/sys/class/net/wlan0/operstate').read().strip()
    except OSError:
        return 'none', ''
    lvl = ''
    for l in open('/proc/net/wireless').read().splitlines()[2:]:
        if l.strip().startswith('wlan0:'):
            lvl = l.split()[3].rstrip('.')
    return st, lvl
def bt():
    try:
        r = subprocess.run(['bluetoothctl', 'devices', 'Connected'], capture_output=True, text=True, timeout=2)
        return str(sum(1 for l in r.stdout.splitlines() if l.startswith('Device ')))
    except Exception as e:
        return 'err'
new = not os.path.exists(out)
with open(out, 'a') as f:
    if new:
        f.write('wall,volts,motor_max_c,motor_hottest,motor_mean_c,cpu_c,cpu_mhz,wlan,wlan_dbm,bt_conn\n')
    while time.time() - t0 < secs:
        a = time.time(); w = wifi(); b = bt()
        try:
            h = json.loads(subprocess.run(['robotctl', 'health', '--json'], capture_output=True, text=True, timeout=3).stdout)['robot']
            m = h.get('motors', {}); thr = h.get('cpu_throttle', {})
            f.write(f"{a:.2f},{h['battery']['volts']:.3f},{m.get('max_c')},{m.get('hottest')},{m.get('mean_c', 0):.1f},{h.get('cpu_temp_c')},{thr.get('khz', 0)//1000},{w[0]},{w[1]},{b}\n")
        except Exception as e:
            f.write(f"{a:.2f},,,,,,err:{type(e).__name__},{w[0]},{w[1]},{b}\n")
        f.flush(); os.fsync(f.fileno())
        time.sleep(max(0, 0.5 - (time.time() - a)))
