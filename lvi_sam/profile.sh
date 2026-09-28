# shellcheck shell=bash
# LVI-SAM (LVI-SAM-Easyused) 用 slam-replay プロファイル
ALGO=lvi_sam
LIO_ODOM=/lvi_sam/lidar/mapping/odometry
VIO_ODOM=/lvi_sam/vins/odometry/odometry
RECORD_TOPICS="$LIO_ODOM /lvi_sam/lidar/mapping/path $VIO_ODOM /lvi_sam/vins/odometry/path"
# LVI_RECORD_DENSE=1 で、各スキャンを地図座標に変換した点群 (密な地図の元データ) も記録する (容量大)
if [[ "${LVI_RECORD_DENSE:-0}" == 1 ]]; then
  RECORD_TOPICS="$RECORD_TOPICS /lvi_sam/lidar/mapping/cloud_registered_raw"
fi
LAUNCH_WAIT=8

cfg() { python3 -c "import yaml,sys; c=yaml.safe_load(open('$CALIB')); print(eval(sys.argv[1]))" "$1"; }

algo_prepare() {
  MAP_DIR="$WORK/lvi_sam_map"
  python3 /opt/drone_slam/lvi_sam/gen_config.py --calib "$CALIB" --out "$WORK" --map-dir "$MAP_DIR"
  LIDAR_TOPIC=$(cfg "c['topics']['lidar']")
  BLIND=$(cfg "c.get('lidar',{}).get('blind',0.5)")
  NUM_LINES=$(cfg "c.get('lidar',{}).get('scan_lines',4)")
}

algo_launch() {
  # SIGINT 後の地図保存が 15s で打ち切られないよう猶予を延ばす
  exec roslaunch --sigint-timeout=600 /opt/drone_slam/lvi_sam/launch/replay.launch \
    params_lidar:="$WORK/params_lidar.yaml" params_camera:="$WORK/params_camera.yaml" \
    lidar_topic:="$LIDAR_TOPIC" blind:="$BLIND" num_lines:="$NUM_LINES" rviz:="$USE_RVIZ"
}

algo_finish() {
  echo "[lvi_sam] /lvi_sam/save_map で地図を保存します (resolution=${LVI_SAVE_RESOLUTION:-0.0})"
  if ! timeout "${LVI_SAVE_TIMEOUT:-600}" rosservice call /lvi_sam/save_map \
      "{resolution: ${LVI_SAVE_RESOLUTION:-0.0}, destination: '$MAP_DIR'}"; then
    echo "[lvi_sam] WARNING: save_map サービス呼び出しに失敗しました" >&2
  fi
  stop_launch 620
}

algo_collect() {
  if [[ -d "$MAP_DIR" ]]; then
    cp -r "$MAP_DIR"/. "$OUT/"
  else
    echo "[lvi_sam] WARNING: 地図が保存されていません ($MAP_DIR)" >&2
  fi
  if [[ -f "$OUT/slam_output.bag" ]]; then
    python /opt/drone_slam/lvi_sam/bag_to_tum.py "$OUT/slam_output.bag" "$LIO_ODOM" "$OUT/trajectory_lio_tum.txt" || true
    python /opt/drone_slam/lvi_sam/bag_to_tum.py "$OUT/slam_output.bag" "$VIO_ODOM" "$OUT/trajectory_vio_tum.txt" || true
  fi
}
