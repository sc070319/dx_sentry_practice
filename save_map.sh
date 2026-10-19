#!/usr/bin/env bash
# save_map.sh —— 一键保存当前 SLAM 建图结果
#
# 做的事:
#   1) 存成 cod_bringup/maps/latest.pgm/.yaml  (导航 singlenav_launch 默认自动加载这张)
#   2) 在 maps/archive/ 留一份带时间戳的归档 (历史版本都保留)
#   3) 自动转 PNG 方便直接看
#
# 用法: 建图 launch (multiplenav) 还开着的时候, 在任意终端执行:
#       cd ~/Desktop/COD26_ws && ./save_map.sh
set -e

WS="$HOME/Desktop/COD26_ws"
MAPS="$WS/src/cod_-rm2026_-navigation/src/cod_bringup/maps"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# /map 必须在(说明 slam_toolbox 还跑着), 否则存出来是空的
if ! ros2 topic list 2>/dev/null | grep -qx "/map"; then
    echo "❌ 没有检测到 /map 话题 —— 建图的 launch 是不是已经关了?"
    echo "   地图必须在 slam_toolbox 还在运行时保存。"
    exit 1
fi

STAMP=$(date +%m%d_%H%M%S)
echo "正在保存地图 (约1秒)..."
ros2 run nav2_map_server map_saver_cli -f "$MAPS/latest"

# map_saver_cli 默认写 free_thresh: 0.25，而 PGM 里未知区域是 205
# (255-205)/255 = 0.196 < 0.25 → 会被当成 free，地图显示成一大片白。
# 改成 0.196，让未知区域正确恢复为 -1(unknown)。
sed -i 's/^free_thresh: .*/free_thresh: 0.196/' "$MAPS/latest.yaml"

# 归档一份带时间戳的
mkdir -p "$MAPS/archive"
cp "$MAPS/latest.pgm"  "$MAPS/archive/map_$STAMP.pgm"
cp "$MAPS/latest.yaml" "$MAPS/archive/map_$STAMP.yaml"
sed -i "s|^image: .*|image: map_$STAMP.pgm|" "$MAPS/archive/map_$STAMP.yaml"

echo ""
echo "✅ 保存完成:"
echo "   导航默认图 : $MAPS/latest.pgm / .yaml"
echo "   归档版本   : $MAPS/archive/map_$STAMP.pgm / .yaml"
echo ""
echo "   跑导航:  ros2 launch cod_bringup singlenav_launch.py"
python3 "$WS/map2png.py" "$MAPS/latest.pgm" || true
