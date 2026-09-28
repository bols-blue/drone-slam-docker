# shellcheck shell=bash
# SR-LIVO 用 slam-replay プロファイル
ALGO=sr_livo
RECORD_TOPICS="/Odometry_after_opt /path"

algo_prepare() {
  mkdir -p "$WORK/output"
  python3 /opt/drone_slam/sr_livo/gen_config.py --calib "$CALIB" --out "$WORK/sr_livo.yaml" \
    --output-path "$WORK/output"
}

algo_launch() {
  # roslaunch 経由だと SIGINT 後 15 秒で SIGTERM に昇格し、大きな地図の保存が途中で切れるため
  # パラメータをロードしてノードを直接起動する
  rosparam load "$WORK/sr_livo.yaml"
  if [[ "$USE_RVIZ" == "true" ]]; then
    rosrun rviz rviz -d "$(rospack find sr_livo)/rviz_cfg/visualization.rviz" > /dev/null 2>&1 &
  fi
  exec rosrun sr_livo livo_node
}

algo_finish() {
  # SIGINT で ros::ok() が false になり、終了時に rgb_map.pcd を書き出す
  stop_launch 300
  pkill -INT -f rviz 2>/dev/null || true
}

algo_collect() {
  cp -v "$WORK/output/rgb_map.pcd" "$OUT/" 2>/dev/null || echo "[sr_livo] WARNING: rgb_map.pcd がありません" >&2
  # pose.txt: TUM 形式 (time tx ty tz qx qy qz qw)、IMU の世界座標系での姿勢
  for f in pose.txt velocity.txt bias.txt; do
    [[ -f "$WORK/output/$f" ]] && cp -v "$WORK/output/$f" "$OUT/"
  done
  true
}
