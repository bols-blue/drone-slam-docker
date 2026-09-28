# shellcheck shell=bash
# FAST-LIVO2 用 slam-replay プロファイル
ALGO=fast_livo2
FL2_SRC=/opt/drone_slam/ws/src/FAST-LIVO2
# /path は毎回全軌跡を含み bag が巨大になるので録らない (軌跡は trajectory_tum.txt を使う)
RECORD_TOPICS="/aft_mapped_to_init"
# 画像出力 (COLMAP 形式) を有効にする場合は FAST_LIVO2_COLMAP=1 を渡す
FAST_LIVO2_COLMAP="${FAST_LIVO2_COLMAP:-0}"

algo_prepare() {
  python3 /opt/drone_slam/fast_livo2/gen_config.py --calib "$CALIB" --out "$WORK"
  if [[ "$FAST_LIVO2_COLMAP" == "1" ]]; then
    sed -i 's/^\(  colmap_output_en:\).*/\1 true/' "$WORK/fast_livo2.yaml"
  fi
  # FAST-LIVO2 は ビルド時のソースディレクトリ直下 Log/ に結果を書くので、出力先へ付け替える
  local log="$OUT/Log"
  mkdir -p "$log"/{pcd,image,result,Colmap/sparse/0,Colmap/images}
  rm -rf "$FL2_SRC/Log"
  ln -sfn "$log" "$FL2_SRC/Log"
}

algo_launch() {
  local img_topic img_compressed=false
  img_topic=$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['common']['img_topic'])" "$WORK/fast_livo2.yaml")
  # bag に raw 画像が無く <topic>/compressed だけある場合は republish で展開する
  if ! grep -qE "^\s*(topics:)?\s*${img_topic}\s" "$OUT/bag_info.txt" && grep -qE "\s${img_topic}/compressed\s" "$OUT/bag_info.txt"; then
    echo "[fast_livo2] ${img_topic}/compressed を raw に展開して使用します"
    img_compressed=true
  fi
  exec roslaunch --wait --sigint-timeout=600 /opt/drone_slam/fast_livo2/replay.launch config_dir:="$WORK" rviz:="$USE_RVIZ" \
    image_topic:="$img_topic" image_compressed:="$img_compressed"
}

# ノードは while(ros::ok()) を抜けた後に PCD を保存する。大きな地図の保存・間引きには時間がかかるため、
# ノードに直接 SIGINT を送って終了 (=保存完了) を待ち、その後 roslaunch を止める
# (roslaunch 側も --sigint-timeout=600 で SIGTERM へのエスカレートを遅らせている)
algo_finish() {
  local pid
  pid=$(pgrep -f "fast_livo/fastlivo_mapping" | head -1 || true)
  if [[ -n "$pid" ]]; then
    echo "[fast_livo2] fastlivo_mapping に SIGINT を送信し PCD 保存を待ちます"
    kill -INT "$pid" || true
    for ((i = 0; i < ${SAVE_TIMEOUT:-1800}; i++)); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
  fi
  stop_launch 600
}

algo_collect() {
  local log="$OUT/Log"
  [[ -f "$log/pcd/all_raw_points.pcd" ]] && mv "$log/pcd/all_raw_points.pcd" "$OUT/map_rgb_raw.pcd"
  # 上流の間引き結果は広い地図だと間引かれないことがあるので、自前で間引き直す
  rm -f "$log/pcd/all_downsampled_points.pcd"
  if [[ -f "$OUT/map_rgb_raw.pcd" ]]; then
    local leaf
    leaf=$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['pcd_save']['filter_size_pcd'])" "$WORK/fast_livo2.yaml")
    python3 /opt/drone_slam/fast_livo2/voxel_downsample.py "$OUT/map_rgb_raw.pcd" "$OUT/map_rgb.pcd" --leaf "$leaf" \
      || echo "[fast_livo2] WARNING: 間引きに失敗しました (map_rgb_raw.pcd は残っています)" >&2
  fi
  [[ -f "$log/result/replay.txt" ]] && cp "$log/result/replay.txt" "$OUT/trajectory_tum.txt"
  [[ -f "$log/pcd/lidar_poses.txt" ]] && cp "$log/pcd/lidar_poses.txt" "$OUT/lidar_poses.txt"
  if [[ "$FAST_LIVO2_COLMAP" == "1" ]]; then
    mv "$log/Colmap" "$OUT/colmap"
  fi
  rm -f "$log"/mat_*.txt
  if [[ ! -f "$OUT/map_rgb.pcd" && ! -f "$OUT/map_rgb_raw.pcd" ]]; then
    echo "[fast_livo2] WARNING: 地図 PCD が出力されていません (slam.log を確認してください)" >&2
  fi
}
