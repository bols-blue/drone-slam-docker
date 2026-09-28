#!/usr/bin/env bash
# ホスト側ラッパー: ROS2 bag なら ROS1 bag に変換してから、指定した SLAM イメージでリプレイする
#
#   ./run.sh <algo> <bag> [--calib calib.yaml] [--out ./output] [slam-replay のオプション...]
#
#   algo: fast-livo2 | fast-lio2-openmvs | r3live | sr-livo | lvi-sam | all
#   bag : ROS2 bag ディレクトリ / .mcap / .db3 / ROS1 .bag
#
# 例:
#   ./run.sh fast-livo2 ~/drone_slam_bags/texture_20261010_101010 --calib ./my_calib.yaml
#   ./run.sh all ~/bags/flight01.bag --calib ./my_calib.yaml --rate 0.5
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/bols-blue/drone-slam}"
ALGOS_ALL=(fast-livo2 fast-lio2-openmvs r3live sr-livo lvi-sam)

[[ $# -ge 2 ]] || { sed -n '2,12p' "$0"; exit 1; }
ALGO="$1"; BAG_IN="$(realpath "$2")"; shift 2
CALIB="" OUT="$(pwd)/output" PASS=() RVIZ=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --calib) CALIB="$(realpath "$2")"; shift 2 ;;
    --out) OUT="$(realpath -m "$2")"; shift 2 ;;
    --rviz) RVIZ=true; PASS+=("$1"); shift ;;
    *) PASS+=("$1"); shift ;;
  esac
done
mkdir -p "$OUT"

# --- ROS2 bag -> ROS1 bag ---
if [[ -d "$BAG_IN" || "$BAG_IN" == *.mcap || "$BAG_IN" == *.db3 ]]; then
  base="${BAG_IN%/}"; base="${base%.mcap}"; base="${base%.db3}"
  BAG="${base}.ros1.bag"
  if [[ ! -f "$BAG" ]]; then
    echo "[run.sh] ROS2 bag を変換: $BAG_IN -> $BAG"
    src_dir="$(dirname "$BAG_IN")"
    docker run --rm -u "$(id -u):$(id -g)" -v "$src_dir:/data" "$IMAGE:tools" \
      "/data/$(basename "$BAG_IN")" "/data/$(basename "$BAG")"
  fi
else
  BAG="$BAG_IN"
fi

DOCKER_ARGS=(--rm -v "$(dirname "$BAG"):/data:ro" -v "$OUT:/output")
[[ -n "$CALIB" ]] && DOCKER_ARGS+=(-v "$CALIB:/config/calib.yaml:ro")
if $RVIZ; then
  xhost +local:docker >/dev/null 2>&1 || true
  DOCKER_ARGS+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
[[ -t 0 ]] && DOCKER_ARGS+=(-it)

if [[ "$ALGO" == all ]]; then ALGOS=("${ALGOS_ALL[@]}"); else ALGOS=("$ALGO"); fi
for a in "${ALGOS[@]}"; do
  echo "[run.sh] ===== $a ====="
  docker run "${DOCKER_ARGS[@]}" "$IMAGE:$a" --bag "/data/$(basename "$BAG")" "${PASS[@]}" || echo "[run.sh] $a が失敗しました" >&2
done
