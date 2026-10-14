#!/usr/bin/env bash
# ROS2 bag のカメラ画像と LiDAR 点群 (Mid-360S / XT16) を RViz2 で表示する
#
#   ros2_tools/view_bag_images.sh <bagディレクトリ> [オプション]
#   例: ros2_tools/view_bag_images.sh ~/claud_code/drone_slam_比較/raw_20261014_014135
#       ros2_tools/view_bag_images.sh <bag> --rate 0.5 --xt16-tf "0 0 -0.1 0 0 0"
#
# オプション:
#   --image TOPIC        画像トピック (既定: bag から自動。CompressedImage は .../compressed のまま)
#   --cloud TOPIC        表示する PointCloud2 トピックを追加 (複数可。既定: /lidar_points、無ければ最初の PointCloud2)
#   --rate R             再生速度 (既定 1.0)
#   --livox-tf "x y z roll pitch yaw"   base_link から見た Mid-360S (livox_frame) の位置 [m] と姿勢 [rad]
#                        (既定 "0 0 0 3.14159 0 0" = 天地逆取り付け)
#   --xt16-tf  "x y z roll pitch yaw"   base_link から見た XT16 (hesai_lidar) (既定 "0 0 0 0 0 0")
#   --decay SEC          Mid-360S の点群を残す秒数 (既定 0.5)
#
# Livox の CustomMsg は livox_to_pointcloud2.py で PointCloud2 に変換して表示する。
# usb_cam yuyv の圧縮画像は format に yuv422_yuy2 が付き RViz2 では展開できないため、compressed_relay.py で bgr8 にする。
# LiDAR は bag に入っている方 (Mid-360S の CustomMsg か XT16 などの PointCloud2) を自動で選んで表示する。
# bag はループ再生する。終了は RViz2 のウィンドウを閉じるか Ctrl+C。
set -euo pipefail
IMAGE="${DRONE_SLAM_ROS2_IMAGE:-ghcr.io/bols-blue/drone-slam:ros2-viewer}"   # rviz2 + image_transport_plugins + python3-opencv + livox_ros_driver2 メッセージを含む ROS2 Jazzy イメージ
HERE="$(cd "$(dirname "$0")" && pwd)"

[[ $# -ge 1 ]] || { sed -n '2,20p' "$0"; exit 1; }
BAG="$(realpath "$1")"; shift
[[ -f "$BAG/metadata.yaml" ]] || { echo "ROS2 bag ディレクトリ (metadata.yaml があるもの) を指定してください: $BAG" >&2; exit 1; }

IMG_TOPIC="" RATE="1.0" DECAY="0.5" LIVOX_TF="0 0 0 3.14159 0 0" XT16_TF="0 0 0 0 0 0"
CLOUDS=()
# 旧形式 (<bag> [画像トピック] [再生速度]) も受け付ける
if [[ $# -ge 1 && "$1" != --* ]]; then IMG_TOPIC="$1"; shift; fi
if [[ $# -ge 1 && "$1" != --* ]]; then RATE="$1"; shift; fi
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) IMG_TOPIC="$2"; shift 2 ;;
    --cloud) CLOUDS+=("$2"); shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --decay) DECAY="$2"; shift 2 ;;
    --livox-tf) LIVOX_TF="$2"; shift 2 ;;
    --xt16-tf) XT16_TF="$2"; shift 2 ;;
    *) echo "不明なオプション: $1" >&2; exit 1 ;;
  esac
done

# metadata.yaml からトピック名と型の組を取り出す
topics_of() { awk -v pat="$1" '/name:/{n=$2} $1=="type:" && $2 ~ pat {print n}' "$BAG/metadata.yaml"; }
[[ -n "$IMG_TOPIC" ]] || IMG_TOPIC=$(topics_of '^sensor_msgs/msg/(Compressed)?Image$' | head -1)
LIVOX_TOPIC=$(topics_of '^livox_ros_driver2/msg/CustomMsg$' | head -1)
if [[ ${#CLOUDS[@]} -eq 0 ]]; then
  ALL_PC=$(topics_of '^sensor_msgs/msg/PointCloud2$')
  if grep -qx /lidar_points <<<"$ALL_PC"; then CLOUDS=(/lidar_points)
  elif [[ -n "$ALL_PC" ]]; then CLOUDS=("$(head -1 <<<"$ALL_PC")"); fi
fi
[[ -n "$IMG_TOPIC$LIVOX_TOPIC${CLOUDS[*]:-}" ]] || { echo "この bag には画像も点群もありません" >&2; exit 1; }

NODES="" RVIZ_ARGS="--fixed base_link --decay $DECAY"
if [[ -n "$IMG_TOPIC" ]]; then
  if [[ "$IMG_TOPIC" == */compressed ]]; then
    NODES+="python3 /tools/compressed_relay.py $IMG_TOPIC /bag_view/image > /tmp/relay.log 2>&1 &"$'\n'
    RVIZ_ARGS+=" --image /bag_view/image"
  else
    RVIZ_ARGS+=" --image $IMG_TOPIC"
  fi
fi
tf_args() { read -r x y z r p yw <<<"$1"; echo "--x $x --y $y --z $z --roll $r --pitch $p --yaw $yw"; }
if [[ -n "$LIVOX_TOPIC" ]]; then
  NODES+="python3 /tools/livox_to_pointcloud2.py $LIVOX_TOPIC /bag_view/livox_points > /tmp/livox.log 2>&1 &"$'\n'
  NODES+="ros2 run tf2_ros static_transform_publisher $(tf_args "$LIVOX_TF") --frame-id base_link --child-frame-id livox_frame > /dev/null 2>&1 &"$'\n'
  RVIZ_ARGS+=" --livox /bag_view/livox_points"
fi
if [[ ${#CLOUDS[@]} -gt 0 ]]; then
  NODES+="ros2 run tf2_ros static_transform_publisher $(tf_args "$XT16_TF") --frame-id base_link --child-frame-id hesai_lidar > /dev/null 2>&1 &"$'\n'
  for c in "${CLOUDS[@]}"; do RVIZ_ARGS+=" --cloud $c"; done
fi
echo "[view_bag_images] bag=$BAG"
echo "  画像: ${IMG_TOPIC:-なし}  Mid-360S: ${LIVOX_TOPIC:-なし}  点群: ${CLOUDS[*]:-なし}  再生速度: $RATE"

xhost +local:docker >/dev/null 2>&1 || true
GPU=()
[[ -e /dev/dri ]] && GPU=(--device /dev/dri)
TTY=()
[[ -t 0 ]] && TTY=(-it)

# --init: Ctrl+C などのシグナルをコンテナ内に届け、終了時に子プロセスも止める
# --network none + 探索範囲 LOCALHOST: 他のコンテナや実機の ROS2 と混線しないようにする (X11 は UNIX ソケット経由)
docker run --rm --init --network none "${TTY[@]}" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DOMAIN_ID="${VIEW_ROS_DOMAIN_ID:-77}" \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb ${VIEW_EXTRA_ENV:-} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro "${GPU[@]}" \
  -v "$BAG:/bag:ro" -v "$HERE:/tools:ro" \
  --entrypoint bash "$IMAGE" -c "
    # Ctrl+C (INT) / docker stop (TERM) / ウィンドウを閉じる、のどれでも全プロセスを止めて終わる
    trap 'kill \$(jobs -p) 2>/dev/null; exit 0' INT TERM
    trap 'kill \$(jobs -p) 2>/dev/null' EXIT
    source /opt/ros/jazzy/setup.bash
    for ws in /opt/viewer_ws/install /drone_slam/ros2_ws/install; do [[ -f \$ws/setup.bash ]] && source \$ws/setup.bash; done
    python3 /tools/make_rviz.py $RVIZ_ARGS > /tmp/view.rviz
    $NODES
    ros2 bag play /bag --loop --rate $RATE > /tmp/play.log 2>&1 &
    rviz2 -d /tmp/view.rviz &
    wait \$!"
