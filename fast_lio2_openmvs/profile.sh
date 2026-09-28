# shellcheck shell=bash
# FAST-LIO2 + 後処理テクスチャリング (slam-replay から source される)
ALGO=fast_lio2_openmvs
RECORD_TOPICS="/Odometry /path"
FAST_LIO_DIR=/opt/drone_slam/ws/src/FAST_LIO
# TEXTURE=false で後処理 (メッシュ化・テクスチャ) をスキップ
TEXTURE="${TEXTURE:-true}"

algo_prepare() {
  python3 /opt/drone_slam/lib/gen_config.py --calib "$CALIB" --out "$WORK/fast_lio.yaml"
  rm -f "$FAST_LIO_DIR"/PCD/*.pcd
}

algo_launch() {
  # roslaunch は既定で SIGINT 後 15 秒で SIGTERM に切り替え、地図保存が途中で打ち切られるため延長する
  exec roslaunch --sigint-timeout=600 /opt/drone_slam/launch/mapping.launch config:="$WORK/fast_lio.yaml" rviz:="$USE_RVIZ"
}

algo_finish() {
  # FAST-LIO は SIGINT で終了する際に PCD/scans.pcd を書き出す。
  # その前に ros::Rate::sleep() が /clock を待つので、再生終了後も /clock を進めておく
  python3 /opt/drone_slam/lib/clock_keeper.py > "$OUT/clock_keeper.log" 2>&1 &
  local ck=$!
  stop_launch 600
  kill -INT "$ck" 2>/dev/null || true
}

algo_collect() {
  if [[ -f "$FAST_LIO_DIR/PCD/scans.pcd" ]]; then
    cp "$FAST_LIO_DIR/PCD/scans.pcd" "$OUT/scans.pcd"
    echo "[fast_lio2] 地図: $OUT/scans.pcd"
  else
    echo "[fast_lio2] WARNING: scans.pcd が生成されていません (SLAM が初期化できなかった可能性)" >&2
  fi
}

algo_postprocess() {
  [[ "$TEXTURE" == "true" ]] || { echo "[fast_lio2] TEXTURE=false のため後処理をスキップ"; return 0; }
  [[ -f "$OUT/scans.pcd" && -f "$OUT/slam_output.bag" ]] || { echo "[fast_lio2] 地図か軌跡が無いので後処理をスキップ" >&2; return 0; }
  python3 /opt/drone_slam/lib/texture_pipeline.py --out-dir "$OUT" --bag "$BAG" --calib "$CALIB" \
    ${TEXTURE_ARGS:-} 2>&1 | tee "$OUT/texture.log" || echo "[fast_lio2] WARNING: テクスチャリングに失敗しました (texture.log 参照)" >&2
}
