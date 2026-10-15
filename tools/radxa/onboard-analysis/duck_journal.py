"""把本次开机的系统日志实时抄一份到卡上，每秒 fsync（Armbian 的 /var/log 在内存里，拔电就丢）。
跳过 robotd 启动时每 20 ms 一条的 IMU settling 刷屏。用法：python3 duck_journal.py 输出文件"""
import os, subprocess, sys, threading, time
out = sys.argv[1]
p = subprocess.Popen(['journalctl', '-b', '-f', '-n', 'all', '-o', 'short-precise', '--no-pager'],
                     stdout=subprocess.PIPE, text=True, errors='replace')
f = open(out, 'a'); lock = threading.Lock()
def syncer():
    while True:
        time.sleep(1)
        with lock:
            f.flush(); os.fsync(f.fileno())
threading.Thread(target=syncer, daemon=True).start()
for line in p.stdout:
    if 'settling' in line:
        continue
    with lock:
        f.write(line)
