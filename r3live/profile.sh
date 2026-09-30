# shellcheck shell=bash
# R3LIVE 用 slam-replay プロファイル (common/scripts/slam-replay から source される)
ALGO=r3live
RECORD_TOPICS="/aft_mapped_to_init /camera_odom /path /camera_path"
R3_DIR=/opt/drone_slam/r3live
# NOTE: profile.sh は WORK 決定前に source されるので、WORK 依存のパスは algo_prepare 内で決める

_calib_topic() {
  python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1])).get('topics',{}).get(sys.argv[2], sys.argv[3]))" "$CALIB" "$1" "$2"
}

algo_prepare() {
  R3_MAP_DIR="$WORK/r3live_output"
  R3_TRIGGER="$WORK/save_map.trigger"
  mkdir -p "$R3_MAP_DIR"
  python3 "$R3_DIR/gen_config.py" --calib "$CALIB" --out "$WORK/r3live_config.yaml" --map-dir "$R3_MAP_DIR"
  LIDAR_TOPIC=$(_calib_topic lidar /livox/lidar)
  IMU_TOPIC=$(_calib_topic imu /livox/imu)
  IMAGE_TOPIC=$(_calib_topic image /image_raw)
  echo "[r3live] topics: lidar=$LIDAR_TOPIC imu=$IMU_TOPIC image=$IMAGE_TOPIC (圧縮画像は ${IMAGE_TOPIC}/compressed も購読)"
}

algo_launch() {
  export R3LIVE_SAVE_TRIGGER="$R3_TRIGGER"
  [[ "$USE_RVIZ" == true ]] || export R3LIVE_HEADLESS=1
  # roslaunch は既定で SIGINT 15 秒後に SIGTERM へエスカレートするので延長
  exec roslaunch --wait --sigint-timeout=600 "$R3_DIR/launch/r3live_replay.launch" \
    config:="$WORK/r3live_config.yaml" lidar_topic:="$LIDAR_TOPIC" imu_topic:="$IMU_TOPIC" \
    image_topic:="$IMAGE_TOPIC" rviz:="$USE_RVIZ"
}

# 再生終了後: トリガファイルで地図保存 -> SLAM 終了 -> オフラインでメッシュ化 + テクスチャ
algo_finish() {
  local timeout="${R3LIVE_SAVE_TIMEOUT:-900}"
  # 色付き地図の保存は VIO スレッドが行うため、画像の無い bag では保存されない。長く待たずに終える
  if ! grep -qE "^\s*(topics:)?\s*${IMAGE_TOPIC}(/compressed)?\s" "$OUT/bag_info.txt" 2>/dev/null; then
    echo "[r3live] WARNING: bag に画像トピック ${IMAGE_TOPIC} がありません。R3LIVE は画像なしでは地図を保存しないため、軌跡のみ出力します" >&2
    timeout=10
  fi
  echo "[r3live] 地図保存をトリガ (最大 ${timeout}s 待機)"
  rm -f "$R3_TRIGGER.done"
  touch "$R3_TRIGGER"
  for ((i = 0; i < timeout; i++)); do
    [[ -f "$R3_TRIGGER.done" ]] && break
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 1
  done
  if [[ -f "$R3_TRIGGER.done" ]]; then
    echo "[r3live] 地図保存完了"
  else
    echo "[r3live] WARNING: 地図保存が確認できませんでした (画像が1枚も処理されていない可能性)" >&2
  fi
  stop_launch 620

  if [[ "${R3LIVE_MESH:-1}" != 0 && -s "$R3_MAP_DIR/test.r3live" ]]; then
    echo "[r3live] オフラインメッシュ化 + テクスチャリング (r3live_meshing)"
    timeout "${R3LIVE_MESH_TIMEOUT:-7200}" roslaunch "$R3_DIR/launch/r3live_mesh.launch" \
      working_dir:="$R3_MAP_DIR" ${R3LIVE_MESH_ARGS:-} > "$OUT/mesh.log" 2>&1 \
      || echo "[r3live] WARNING: メッシュ化に失敗しました ($OUT/mesh.log を参照)" >&2
  else
    echo "[r3live] test.r3live が無いためメッシュ化をスキップ"
  fi
}

algo_collect() {
  mkdir -p "$OUT/map"
  # test.r3live は画像を含み大きくなるので copy ではなく move
  mv "$R3_MAP_DIR"/* "$OUT/map/" 2>/dev/null || true
  [[ -f "$OUT/map/rgb_pt.pcd" ]] && ln -sf map/rgb_pt.pcd "$OUT/colored_map.pcd"
  [[ -f "$OUT/map/textured_mesh.ply" ]] && ln -sf map/textured_mesh.ply "$OUT/textured_mesh.ply"
  # 軌跡を TUM 形式 (stamp x y z qx qy qz qw) で出力 (IMU 位置姿勢 / カメラ位置姿勢)
  if [[ -f "$OUT/slam_output.bag" ]]; then
    local topic name
    for topic in /aft_mapped_to_init /camera_odom; do
      name=$([[ $topic == /camera_odom ]] && echo trajectory_camera_tum.txt || echo trajectory_imu_tum.txt)
      rostopic echo -b "$OUT/slam_output.bag" -p "$topic" 2>/dev/null | awk -F, '
        NR == 1 { for (i = 1; i <= NF; i++) col[$i] = i; next }
        { printf "%.9f %s %s %s %s %s %s %s\n", $col["field.header.stamp"] / 1e9,
            $col["field.pose.pose.position.x"], $col["field.pose.pose.position.y"], $col["field.pose.pose.position.z"],
            $col["field.pose.pose.orientation.x"], $col["field.pose.pose.orientation.y"],
            $col["field.pose.pose.orientation.z"], $col["field.pose.pose.orientation.w"] }' > "$OUT/$name" || true
      [[ -s "$OUT/$name" ]] || rm -f "$OUT/$name"
    done
  fi
}
