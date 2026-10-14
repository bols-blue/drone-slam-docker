#!/usr/bin/env bash
# ROS2 bag 内のカメラ画像を RViz2 で表示する (drone_slam イメージの ROS2 Jazzy を使用)
#
#   ros2_tools/view_bag_images.sh <bagディレクトリ> [画像トピック] [再生速度]
#   例: ros2_tools/view_bag_images.sh ~/drone_slam_bags/raw_20260928_064026
#       ros2_tools/view_bag_images.sh ~/drone_slam_bags/raw_20260928_064026 /image_raw/compressed 0.5
#
# 画像トピックを省略すると bag の metadata.yaml から自動で選ぶ (CompressedImage は .../compressed のまま指定する)。
# 圧縮画像は compressed_relay.py で bgr8 に展開し直して /bag_view/image として表示する
# (usb_cam yuyv の圧縮画像は format に yuv422_yuy2 が付き、RViz2 の標準の展開では表示できないため)。
# bag はループ再生する。終了は RViz2 のウィンドウを閉じるか Ctrl+C。
set -euo pipefail
IMAGE="${DRONE_SLAM_ROS2_IMAGE:-drone_slam:latest}"   # rviz2 + image_transport_plugins + python3-opencv を含む ROS2 Jazzy イメージ
HERE="$(cd "$(dirname "$0")" && pwd)"

[[ $# -ge 1 ]] || { sed -n '2,9p' "$0"; exit 1; }
BAG="$(realpath "$1")"
TOPIC="${2:-}"
RATE="${3:-1.0}"
[[ -f "$BAG/metadata.yaml" ]] || { echo "ROS2 bag ディレクトリ (metadata.yaml があるもの) を指定してください: $BAG" >&2; exit 1; }

if [[ -z "$TOPIC" ]]; then
  # metadata.yaml から sensor_msgs/msg/Image または CompressedImage のトピックを探す
  TOPIC=$(awk '/name:/{n=$2} /type: sensor_msgs\/msg\/(Compressed)?Image$/{print n; exit}' "$BAG/metadata.yaml")
  [[ -n "$TOPIC" ]] || { echo "この bag には画像トピックがありません" >&2; exit 1; }
fi
echo "[view_bag_images] bag=$BAG topic=$TOPIC rate=$RATE"

CFG="$(mktemp --suffix=.rviz)"
trap 'rm -f "$CFG"' EXIT
VIEW_TOPIC="$TOPIC"
RELAY=""
if [[ "$TOPIC" == */compressed ]]; then
  VIEW_TOPIC=/bag_view/image
  RELAY="python3 /relay.py $TOPIC $VIEW_TOPIC > /tmp/relay.log 2>&1 &"
fi
sed "s#IMAGE_TOPIC_PLACEHOLDER#$VIEW_TOPIC#" "$HERE/bag_images.rviz" > "$CFG"
chmod 644 "$CFG"

xhost +local:docker >/dev/null 2>&1 || true
GPU=()
[[ -e /dev/dri ]] && GPU=(--device /dev/dri)

docker run --rm -it \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro "${GPU[@]}" \
  -v "$BAG:/bag:ro" -v "$CFG:/view.rviz:ro" -v "$HERE/compressed_relay.py:/relay.py:ro" \
  --entrypoint bash "$IMAGE" -c "
    source /opt/ros/jazzy/setup.bash
    $RELAY
    ros2 bag play /bag --loop --rate $RATE > /tmp/play.log 2>&1 &
    rviz2 -d /view.rviz
    kill \$(jobs -p) 2>/dev/null || true"
