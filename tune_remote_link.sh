#!/bin/bash
# ============================================================
# tune_remote_link.sh — NoMachine / WiFi 远程链路优化（Jetson Orin NX）
#
# 用法:
#   bash tune_remote_link.sh --status          # 只查看现状（不需要 sudo）
#   sudo bash tune_remote_link.sh              # 应用省电/延迟调优（不重连、不断网）
#   sudo bash tune_remote_link.sh --bluetooth  # 额外关闭蓝牙（2.4G 共存干扰；会断蓝牙鼠标）
#   sudo bash tune_remote_link.sh --nx         # 额外调 NoMachine 编码并重启 NX（会断开当前会话！）
#   sudo bash tune_remote_link.sh --mdns       # 让 ubuntu.local 只广播 WiFi 地址（修 docker0 抢答）
#   sudo bash tune_remote_link.sh --res 1600x1200   # 立刻改桌面分辨率（可选 1280x720 等）
#
# 说明:
#  - 省电关闭会立即生效且【不会断网】，但当前连接要重连后才持久生效。
#  - --nx 会重启 nxserver，NoMachine 会话会掉线（客户端会自动重连）。
# ============================================================
set -euo pipefail

IFACE="${IFACE:-wlP1p1s0}"
OUTPUT="${OUTPUT:-DP-1}"
DO_BT=0
DO_NX=0
DO_MDNS=0
DO_STATUS=0
RES=""

while [ $# -gt 0 ]; do
  case "$1" in
    --status)    DO_STATUS=1 ;;
    --bluetooth) DO_BT=1 ;;
    --nx)        DO_NX=1 ;;
    --mdns)      DO_MDNS=1 ;;
    --res)       shift; RES="${1:-}" ;;
    -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
  shift
done

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    echo "❌ 这个操作需要 root：请用  sudo bash $0 $*"
    exit 1
  fi
}

show_status() {
  echo "===== 当前链路状态 ====="
  iw dev "$IFACE" link 2>/dev/null | sed 's/^/  /' || true
  echo "  省电模式: $(iw dev "$IFACE" get power_save 2>/dev/null | awk '{print $3}')"
  iw dev "$IFACE" station dump 2>/dev/null | grep -E 'signal avg|tx bitrate|rx bitrate|tx retries|tx failed|beacon loss' | sed 's/^/  /' || true
  echo "  TCP: $(sysctl -n net.ipv4.tcp_congestion_control) / slow_start_after_idle=$(sysctl -n net.ipv4.tcp_slow_start_after_idle)"
  echo "  分辨率: $(DISPLAY=:0 xrandr --current 2>/dev/null | grep -oE '[0-9]+x[0-9]+' | head -1)"
  echo "  NX 进程: $(pgrep -c -f 'nxnode.bin|nxcodec.bin' || echo 0)"
}

# ---------- 0. 只查看 ----------
if [ "$DO_STATUS" -eq 1 ]; then
  show_status
  exit 0
fi

need_root "$@"

echo "===== 1. 关闭 WiFi 省电（立即生效，不断网） ====="
iw dev "$IFACE" set power_save off
echo "  -> $(iw dev "$IFACE" get power_save)"

echo "===== 2. 持久化省电关闭（NetworkManager） ====="
# 注意：NM 按文件名字典序加载 conf.d，后读的覆盖先读的。
# Ubuntu 自带 default-wifi-powersave-on.conf(d=100) 排在 99-(=57) 之后，
# 所以必须用 zz- 前缀才能覆盖它。
rm -f /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf
cat > /etc/NetworkManager/conf.d/zz-wifi-powersave-off.conf <<'EOF'
[connection]
wifi.powersave = 2
EOF
# 同时写到每个已保存的 WiFi 连接上（优先级高于全局默认）
while IFS=: read -r name type; do
  [ "$type" = "802-11-wireless" ] || continue
  nmcli connection modify "$name" 802-11-wireless.powersave 2 >/dev/null 2>&1 || true
  echo "  $name: powersave=2"
done < <(nmcli -t -f NAME,TYPE connection show)

echo "===== 3. TCP 交互延迟调优 ====="
cat > /etc/sysctl.d/99-remote-latency.conf <<'EOF'
# 空闲后不要重置拥塞窗口（远程桌面/按键突发更跟手）
net.ipv4.tcp_slow_start_after_idle = 0
# MTU 黑洞探测，避免个别链路卡死
net.ipv4.tcp_mtu_probing = 1
EOF
sysctl -q -p /etc/sysctl.d/99-remote-latency.conf
echo "  slow_start_after_idle=$(sysctl -n net.ipv4.tcp_slow_start_after_idle)"

# ---------- 可选: 蓝牙 ----------
if [ "$DO_BT" -eq 1 ]; then
  echo "===== 4. 关闭蓝牙（2.4G 与 WiFi 共存干扰） ====="
  rfkill block bluetooth || true
  systemctl disable --now bluetooth >/dev/null 2>&1 || true
  echo "  -> $(rfkill list bluetooth 2>/dev/null | head -1 || echo '已关闭')"
fi

# ---------- 可选: mDNS ----------
if [ "$DO_MDNS" -eq 1 ]; then
  echo "===== 5. 修 avahi 只广播 WiFi 地址 ====="
  cp -a /etc/avahi/avahi-daemon.conf "/etc/avahi/avahi-daemon.conf.bak_$(date +%Y%m%d_%H%M%S)"
  if grep -q '^deny-interfaces=' /etc/avahi/avahi-daemon.conf; then
    sed -i 's/^deny-interfaces=.*/deny-interfaces=docker0,l4tbr0/' /etc/avahi/avahi-daemon.conf
  else
    sed -i '/^\[server\]/a deny-interfaces=docker0,l4tbr0' /etc/avahi/avahi-daemon.conf
  fi
  systemctl restart avahi-daemon
  sleep 1
  echo "  -> ubuntu.local = $(avahi-resolve -4 -n "$(hostname).local" 2>/dev/null | awk '{print $2}')"
fi

# ---------- 可选: 分辨率 ----------
if [ -n "$RES" ]; then
  echo "===== 6. 切换分辨率 -> $RES ====="
  DISPLAY=:0 xrandr --output "$OUTPUT" --mode "$RES"
  DISPLAY=:0 xrandr --current | head -2
fi

# ---------- 可选: NoMachine 编码 ----------
if [ "$DO_NX" -eq 1 ]; then
  echo "===== 7. NoMachine 编码参数 ====="
  CFG=/usr/NX/etc/node.cfg
  cp -a "$CFG" "$CFG.bak_$(date +%Y%m%d_%H%M%S)"
  set_kv() {  # $1=key $2=value
    if grep -qE "^[#[:space:]]*$1[[:space:]]" "$CFG"; then
      sed -i -E "s|^[#[:space:]]*$1[[:space:]].*|$1 $2|" "$CFG"
    else
      printf '\n%s %s\n' "$1" "$2" >> "$CFG"
    fi
    echo "  $1 $2"
  }
  # 固定 H.264 + 帧率上限，避免软件编码把 CPU 打满（nxcodec.bin 无硬件编码）
  set_kv EnableDisplayServerVideoCodec 1
  set_kv DisplayServerVideoCodec h264
  set_kv DisplayServerVideoFrameRate 30
  set_kv DisplayServerUseVideoFrameRate 1

  echo "  ⚠️  5 秒后重启 nxserver，当前 NoMachine 会话会断开（客户端会自动重连）"
  sleep 5
  /usr/NX/bin/nxserver --restart >/dev/null 2>&1 || systemctl restart nxserver 2>/dev/null || true
  sleep 3
  echo "  -> $(pgrep -c -f 'nxnode.bin|nxcodec.bin' || echo 0) 个 NX 进程"
fi

echo
echo "✅ 完成。当前状态："
show_status
