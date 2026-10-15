#!/bin/bash
# 主板降温：看是谁在发热、记录温度曲线、关掉走路用不到的服务（可一键恢复）。
#   sudo bash duck-quiet.sh status                 # 现在的温度、频率、最吃 CPU 的进程、相关服务状态
#   sudo bash duck-quiet.sh measure 120 空闲        # 每 5 秒记一次，记 120 秒，标签「空闲」
#   sudo bash duck-quiet.sh quiet                  # 关掉下面 QUIET 列表里正在开机自启的服务，并记下原状态
#   sudo bash duck-quiet.sh restore                # 按记录把关掉的服务原样恢复
#
# 为什么要这个：2026-10-12 第一台装好 kissqy 整套后，主板 85–92°C、CPU 一直锁在最低档 408 MHz，
# 网页摇杆因此来不及响应。没接摄像头时 mediad 不停崩溃重启，GStreamer 扫描占满一个核。
# 走路只要 robotd（控制）、padd 和网页手柄（摇杆）、SSH、网络。
# 日志：/var/log/duck-quiet/<日期>.log（分类、分级），温度曲线：/var/log/duck-quiet/thermal-<时间>-<标签>.csv
set -euo pipefail
export LC_ALL=C.UTF-8

# 走路用不到的。蓝牙（bluetooth.service）不在默认列表里：以后接蓝牙手柄要用它。
QUIET=(mediad tofd btd configd gamesir-xboxd updaterd robot-boot-check.timer)
STATE=/var/lib/duck-quiet/disabled.txt
LOGDIR=/var/log/duck-quiet
mkdir -p "$LOGDIR" "$(dirname "$STATE")"
LOG="$LOGDIR/$(date +%F).log"
log(){ printf '%s [%s][%s] %s\n' "$(date '+%F %T')" "$1" "$2" "$3" | tee -a "$LOG" >&2; }
[ "$(id -u)" = 0 ] || { echo "要 root：sudo bash $0 ..."; exit 1; }

temp_c(){ awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp; }
freq_mhz(){ awk '{printf "%d", $1/1000}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq; }
unit_name(){ case "$1" in *.*) echo "$1" ;; *) echo "$1.service" ;; esac; }

status(){
  log 温度 INFO "CPU $(temp_c)°C，频率 $(freq_mhz) MHz（408 = 最低档，在降频）"
  log 负载 INFO "1/5/15 分钟负载 $(cut -d' ' -f1-3 /proc/loadavg)"
  # 采两次取第二次，第一次的 CPU 占用是开机以来的平均
  top -b -n 2 -d 2 -o %CPU | awk '/^top/{n++} n==2' | sed -n '7,14p' | while read -r line; do log 进程 INFO "$line"; done
  for u in robotd padd microduck-webpad.socket "${QUIET[@]}" bluetooth; do
    n=$(unit_name "$u")
    # is-enabled 对 disabled 也返回非 0，不能用 || 兜底，否则「disabled」后面会多出一个「无」
    en=$(systemctl is-enabled "$n" 2>/dev/null || true); [ -n "$en" ] || en=无
    log 服务 INFO "$(printf '%-24s 开机自启=%-8s 现在=%s' "$n" "$en" "$(systemctl is-active "$n" 2>/dev/null || true)")"
  done
}

measure(){
  local secs="${1:-120}" label="${2:-未命名}" out
  out="$LOGDIR/thermal-$(date +%Y%m%d-%H%M%S)-${label}.csv"
  echo "秒,温度C,频率MHz,负载1分钟" > "$out"
  log 测温 INFO "开始：$label，$secs 秒，每 5 秒一次 → $out"
  local t=0
  while [ "$t" -le "$secs" ]; do
    echo "$t,$(temp_c),$(freq_mhz),$(cut -d' ' -f1 /proc/loadavg)" >> "$out"
    sleep 5; t=$((t + 5))
  done
  awk -F, 'NR>1 {n++; s+=$2; if($2>mx)mx=$2; if(mn==""||$2<mn)mn=$2; if($3<=408)low++}
           END {printf "温度 最低 %.1f / 平均 %.1f / 最高 %.1f °C；%d/%d 次在 408 MHz\n", mn, s/n, mx, low, n}' "$out" \
    | while read -r line; do log 测温 INFO "$label：$line"; done
}

quiet(){
  [ -f "$STATE" ] && { log 服务 WARN "已经有关闭记录 $STATE，先 restore 再 quiet，免得记录被覆盖"; exit 1; }
  : > "$STATE"
  for u in "${QUIET[@]}"; do
    n=$(unit_name "$u")
    if [ "$(systemctl is-enabled "$n" 2>/dev/null || true)" = enabled ]; then
      systemctl disable --now "$n" >/dev/null 2>&1 && { echo "$n" >> "$STATE"; log 服务 INFO "已关：$n"; } \
        || log 服务 ERROR "关不掉：$n"
    else
      systemctl stop "$n" >/dev/null 2>&1 || true
      log 服务 INFO "本来就没开机自启，只停一下：$n"
    fi
  done
  log 服务 INFO "记录在 $STATE；恢复：sudo bash $0 restore"
}

restore(){
  [ -f "$STATE" ] || { log 服务 WARN "没有关闭记录，什么都不用恢复"; exit 0; }
  while read -r n; do
    [ -n "$n" ] || continue
    systemctl enable --now "$n" >/dev/null 2>&1 && log 服务 INFO "已恢复：$n" || log 服务 ERROR "恢复失败：$n"
  done < "$STATE"
  mv "$STATE" "$STATE.restored-$(date +%Y%m%d-%H%M%S)"
}

case "${1:-status}" in
  status) status ;;
  measure) measure "${2:-120}" "${3:-未命名}" ;;
  quiet) quiet ;;
  restore) restore ;;
  *) echo "用法：sudo bash $0 status | measure [秒] [标签] | quiet | restore"; exit 1 ;;
esac
