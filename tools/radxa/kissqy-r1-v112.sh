#!/bin/bash
# 在 kissqy 的「一键系统镜像 R1」上改出一张 V1.12 板也能开机的卡。
#   sudo bash kissqy-r1-v112.sh <microduck-zero3w-0.15.1-R1.img.xz> [--no-camera] [--no-tof] [--overlay i2c4-m0 ...]
#
# 为什么要改：R1 用的是 Armbian 自带的主线 U-Boot 2025.10 + 内存初始化固件 v1.21，
# 咱们的 V1.12 板（AIC8800 Wi-Fi）用这套引导起不来，换成瑞莎的引导才行（见 README 和踩坑记录）。
#
# 只改这几处，其余（他的固件、模型、首次热点配网、账号锁定、SSH 密钥首启生成）原样保留：
#   1. 扇区 64 / 16384 写瑞莎 u-boot-rk2410 的 idbloader.img / u-boot.itb（版本和 sha256 跟 build-armbian-card.sh 一样）
#   2. 写 /boot/extlinux/extlinux.conf：瑞莎 U-Boot 只认它、不认 boot.scr。overlay、fdt、内核参数
#      都从镜像自己的 armbianEnv.txt 搬过来，所以他的 HAT 音频、摄像头、NPU 照旧生效
#   3. 可选 --overlay NAME：再加 overlay，比如 i2c4-m0（40 针 27/28 脚的 I²C4，IMU 模块飞线接这里）
#   4. 可选 --no-camera / --no-tof：不开机自启 mediad / tofd。没接摄像头、ToF 时它们会不停崩溃重启，
#      mediad 每次重启都扫一遍 GStreamer 插件，能占满一个核，把主板烧到降频
#
# 注意：换成瑞莎引导以后，改 armbianEnv.txt 不再生效，以后加 overlay 要改 extlinux.conf。
# 本脚本只换引导和开机配置，不能让他的程序认 IMU 模块（R1 只认总线上的 IMU 小板 ID 200）。
set -euo pipefail
export LC_ALL=C.UTF-8

R1_SHA=75da978b5d56acd9e4146256431732381de84523bb683e4ab7ac9f0580a7bce5   # kissqy 发布页公布的校验值
UBOOT_DEB_URL=https://radxa-repo.github.io/bookworm/pool/main/u/u-boot-rk2410/u-boot-rk2410_2017.09-64-39cd993_arm64.deb
UBOOT_DEB_SHA=fb319203ab7d568aac77505504c05698385aea8a0e8f895c226c6bc557a1bce7

XZ="${1:?用法: sudo bash $0 <microduck-zero3w-0.15.1-R1.img.xz> [--no-camera] [--no-tof] [--overlay NAME ...]}"
shift
NO_CAMERA=0; NO_TOF=0; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-camera) NO_CAMERA=1 ;;
    --no-tof) NO_TOF=1 ;;
    --overlay) EXTRA+=("${2:?--overlay 后面要跟名字，比如 i2c4-m0}"); shift ;;
    *) echo "不认识的参数：$1"; exit 1 ;;
  esac
  shift
done
HERE="$(cd "$(dirname "$0")" && pwd)"
[ "$(id -u)" = 0 ] || { echo "要 root：sudo bash $0 ..."; exit 1; }
[ -f "$XZ" ] || { echo "找不到 $XZ"; exit 1; }

echo "[1/7] 核对 R1 原包校验值"
GOT=$(sha256sum "$XZ" | cut -d' ' -f1)
[ "$GOT" = "$R1_SHA" ] || { echo "校验值不对：$GOT（应为 $R1_SHA）。别的版本先核对再改脚本。"; exit 1; }

OUT="${XZ%.img.xz}-V112.img"
WORK=$(mktemp -d /root/r1v112.XXXX); trap 'umount -q "$WORK/m" 2>/dev/null; rm -rf "$WORK"' EXIT

echo "[2/7] 瑞莎引导程序"
DEB="$HERE/$(basename "$UBOOT_DEB_URL")"
[ -f "$DEB" ] || curl -fL --retry 3 -o "$DEB" "$UBOOT_DEB_URL"
echo "$UBOOT_DEB_SHA  $DEB" | sha256sum -c - >/dev/null || { echo "u-boot 包校验失败"; exit 1; }
mkdir -p "$WORK/deb"; dpkg-deb -x "$DEB" "$WORK/deb"
UB="$WORK/deb/usr/lib/u-boot/radxa-zero3"; ls "$UB/idbloader.img" "$UB/u-boot.itb" >/dev/null

echo "[3/7] 解压镜像（放本地盘，/mnt 下 loop 挂不了）"
xz -dkc "$XZ" > "$WORK/card.img"

START=$(python3 - "$WORK/card.img" <<'PY'
import struct,sys
f=open(sys.argv[1],'rb'); f.seek(512); h=f.read(512)
elba=struct.unpack('<Q',h[72:80])[0]; esz=struct.unpack('<I',h[84:88])[0]
f.seek(elba*512); e=f.read(esz); print(struct.unpack('<Q',e[32:40])[0])
PY
)
# 引导程序要放在扇区 64..32767 里，分区必须从 32768 或更后面开始
[ "$START" -ge 32768 ] || { echo "根分区从扇区 $START 开始，放不下瑞莎引导，停止"; exit 1; }

echo "[4/7] 写入引导程序（idbloader @64，u-boot.itb @16384）"
dd if="$UB/idbloader.img" of="$WORK/card.img" bs=512 seek=64 conv=notrunc status=none
dd if="$UB/u-boot.itb"   of="$WORK/card.img" bs=512 seek=16384 conv=notrunc status=none

UUID=$(python3 - "$WORK/card.img" "$START" <<'PY'
import sys,uuid
f=open(sys.argv[1],'rb'); f.seek(int(sys.argv[2])*512+1024); sb=f.read(1024); print(uuid.UUID(bytes=sb[104:120]))
PY
)
echo "      根分区起始扇区 $START，UUID $UUID"
mkdir -p "$WORK/m"; mount -o loop,offset=$((START*512)) "$WORK/card.img" "$WORK/m"; R="$WORK/m"
KVER=$(ls "$R/lib/modules" | head -1)
ENV="$R/boot/armbianEnv.txt"
val(){ sed -n "s/^$1=//p" "$ENV" | tr -d '\r' | tail -1; }
PREFIX=$(val overlay_prefix); FDT=$(val fdtfile); ARGS=$(val extraargs)
read -r -a OVERLAYS <<< "$(val overlays)"
for o in "${EXTRA[@]}"; do [[ " ${OVERLAYS[*]} " == *" $o "* ]] || OVERLAYS+=("$o"); done

echo "[5/7] extlinux（瑞莎 U-Boot 认这个，不认 boot.scr）"
FILES=()
for o in "${OVERLAYS[@]}"; do
  f="/boot/dtb/rockchip/overlay/${PREFIX}-${o}.dtbo"
  [ -f "$R$f" ] || { echo "镜像里没有 overlay 文件 $f，停止"; exit 1; }
  FILES+=("$f")
done
[ -f "$R/boot/vmlinuz-$KVER" ] && [ -f "$R/boot/initrd.img-$KVER" ] && [ -f "$R/boot/dtb/$FDT" ] \
  || { echo "内核、initrd 或 $FDT 缺失，停止"; exit 1; }
mkdir -p "$R/boot/extlinux"
cat > "$R/boot/extlinux/extlinux.conf" <<CONF
default armbian
timeout 3
menu title kissqy R1 (Radxa U-Boot)

label armbian
  linux /boot/vmlinuz-$KVER
  initrd /boot/initrd.img-$KVER
  fdt /boot/dtb/$FDT
  fdtoverlays ${FILES[*]}
  append root=UUID=$UUID rootwait rw console=tty1 consoleblank=0 ${ARGS}
CONF
# armbianEnv 只同步 overlay 列表，留给以后看它的脚本；实际生效的是 extlinux
sed -i "s/^overlays=.*/overlays=${OVERLAYS[*]}/" "$ENV"

echo "[6/7] 服务"
off(){ rm -f "$R/etc/systemd/system/multi-user.target.wants/$1.service"; echo "      $1 不再开机自启（以后接上：sudo systemctl enable --now $1）"; }
[ "$NO_CAMERA" = 1 ] && off mediad
[ "$NO_TOF" = 1 ] && off tofd
cat > "$R/etc/microduck-v112-image.txt" <<NOTE
本卡由 microduck-replica tools/radxa/kissqy-r1-v112.sh 从 kissqy Public Image R1 改出（$(date -u +%F)）。
原包 sha256: $R1_SHA
改动：瑞莎 u-boot-rk2410 2017.09-64 引导；/boot/extlinux/extlinux.conf；overlays=${OVERLAYS[*]}；no-camera=$NO_CAMERA no-tof=$NO_TOF
换引导后改 overlay 要改 extlinux.conf，armbianEnv.txt 不再生效。
NOTE

echo "[7/7] 收尾"
umount "$WORK/m"
mv "$WORK/card.img" "$OUT"
echo "做好了：$OUT"
sha256sum "$OUT"
