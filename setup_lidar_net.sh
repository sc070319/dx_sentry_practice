#!/bin/bash
# ============================================================
# 配置雷达网口静态 IP（联调用，需要 sudo）
# 用法:  sudo bash setup_lidar_net.sh [网口名] [IP/掩码]
# 默认:  enP8p1s0, 192.168.1.5/24
#
# IP 来源: src/livox_ros_driver2/config/MID360_config.json
#   host_net_info : 192.168.1.5   （本机，必须是这个地址才能收到点云）
#   lidar ip      : 192.168.1.182 （MID360 雷达，序列号后两位）
#
# 本机网口由 NetworkManager 管理，通过 nmcli 修改并立即生效，
# 重启 / 重新插拔网线后依然保持。
# ============================================================
set -e

IFACE="${1:-enP8p1s0}"
IP_ADDR="${2:-192.168.1.5/24}"

# 找到该网口对应的 NetworkManager 连接
CON_NAME=$(nmcli -t -f NAME,DEVICE con show | awk -F: -v d="$IFACE" '$2==d {print $1; exit}')
if [ -z "$CON_NAME" ]; then
  echo "❌ 没找到网口 $IFACE 对应的 NetworkManager 连接，现有连接："
  nmcli -t -f NAME,DEVICE,TYPE con show
  exit 1
fi

echo "========== 配置 $IFACE（连接: $CON_NAME）-> $IP_ADDR =========="
nmcli con mod "$CON_NAME" ipv4.method manual ipv4.addresses "$IP_ADDR" ipv4.never-default yes
nmcli con up "$CON_NAME"

echo "✅ 完成:"
ip -brief addr show "$IFACE"
echo
echo "下一步: 确认雷达已上电、网线插入该网口，然后测试连通性:"
echo "  ping -c 3 192.168.1.182"
