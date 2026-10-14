#!/usr/bin/env bash
# drone-slam-docker のインストーラ
#
#   curl -fsSL https://raw.githubusercontent.com/bols-blue/drone-slam-docker/main/install.sh | bash
#   (clone 済みなら)  ./install.sh
#
# オプション:
#   --bin-dir DIR   コマンドを置く場所 (既定: ~/.local/bin)
#   --pull          全 Docker イメージを先に取得する (合計 約 25 GB。省略時は初回実行時に必要な分だけ取得)
#   --uninstall     コマンドを削除する (取得したイメージと clone したリポジトリは残す)
#
# インストールされるコマンド:
#   drone-slam-bag2html  bag から比較ビューア (report.html) まで一括で作る
#   drone-slam-view      bag のカメラ画像と LiDAR 点群を RViz2 で見る
#   drone-slam-run       手法を指定して bag をリプレイする
#   drone-slam-update    リポジトリとイメージを最新にする
set -euo pipefail

REPO_URL="https://github.com/bols-blue/drone-slam-docker.git"
IMAGE="ghcr.io/bols-blue/drone-slam"
TAGS=(tools fast-livo2 fast-lio2-openmvs r3live sr-livo lvi-sam ros2-viewer)
BIN_DIR="$HOME/.local/bin"
HOME_DIR="${DRONE_SLAM_HOME:-$HOME/.local/share/drone-slam-docker}"
PULL=false UNINSTALL=false
CMDS=(drone-slam-bag2html drone-slam-view drone-slam-run drone-slam-update)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bin-dir) BIN_DIR="$2"; shift 2 ;;
    --pull) PULL=true; shift ;;
    --uninstall) UNINSTALL=true; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "不明なオプション: $1" >&2; exit 1 ;;
  esac
done

say() { echo "[install] $*"; }
die() { echo "[install] エラー: $*" >&2; exit 1; }

if $UNINSTALL; then
  for c in "${CMDS[@]}"; do rm -f "$BIN_DIR/$c"; done
  say "コマンドを削除しました ($BIN_DIR)。リポジトリ ($HOME_DIR) とイメージは残っています"
  exit 0
fi

# ---- 必要なもの ----
command -v docker >/dev/null || die "Docker が必要です: https://docs.docker.com/engine/install/"
docker info >/dev/null 2>&1 || die "docker を sudo なしで実行できません。'sudo usermod -aG docker \$USER' の後、ログインし直してください"
command -v python3 >/dev/null || die "python3 (3.8 以上) が必要です"
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' || die "python3 は 3.8 以上が必要です"

# ---- リポジトリの場所 ----
SELF_DIR=""
if [[ -n "${BASH_SOURCE[0]:-}" && -f "${BASH_SOURCE[0]}" ]]; then
  SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
if [[ -n "$SELF_DIR" && -f "$SELF_DIR/bag2html.py" ]]; then
  REPO="$SELF_DIR"
  say "この clone を使います: $REPO"
else
  command -v git >/dev/null || die "git が必要です"
  if [[ -d "$HOME_DIR/.git" ]]; then
    say "リポジトリを更新します: $HOME_DIR"
    git -C "$HOME_DIR" pull --ff-only -q
  else
    say "リポジトリを取得します: $HOME_DIR"
    mkdir -p "$(dirname "$HOME_DIR")"
    git clone -q "$REPO_URL" "$HOME_DIR"
  fi
  REPO="$HOME_DIR"
fi

# ---- コマンド (実体のパスで起動するラッパー) ----
mkdir -p "$BIN_DIR"
write_cmd() {
  printf '#!/usr/bin/env bash\n# drone-slam-docker (%s) が生成\nexec %s "$@"\n' "$REPO" "$2" > "$BIN_DIR/$1"
  chmod +x "$BIN_DIR/$1"
}
write_cmd drone-slam-bag2html "python3 \"$REPO/bag2html.py\""
write_cmd drone-slam-view "\"$REPO/ros2_tools/view_bag_images.sh\""
write_cmd drone-slam-run "\"$REPO/run.sh\""
cat > "$BIN_DIR/drone-slam-update" <<UPD
#!/usr/bin/env bash
# drone-slam-docker ($REPO) が生成: リポジトリと取得済みイメージを最新にする
set -e
if [[ -d "$REPO/.git" ]]; then git -C "$REPO" pull --ff-only; fi
for t in ${TAGS[*]}; do
  if docker image inspect "$IMAGE:\$t" >/dev/null 2>&1; then docker pull -q "$IMAGE:\$t"; fi
done
UPD
chmod +x "$BIN_DIR/drone-slam-update"
say "コマンドを置きました: $BIN_DIR/{$(IFS=,; echo "${CMDS[*]}")}"

if $PULL; then
  for t in "${TAGS[@]}"; do say "イメージを取得中: $IMAGE:$t"; docker pull -q "$IMAGE:$t"; done
fi

case ":$PATH:" in
  *":$BIN_DIR:"*) ;;
  *) say "注意: $BIN_DIR が PATH に入っていません。~/.bashrc に次を追加してください:"
     echo "    export PATH=\"$BIN_DIR:\$PATH\"" ;;
esac
cat <<MSG

使い方:
  drone-slam-bag2html <bagフォルダ> --calib my_calib.yaml   # → ./output/<bag名>/report.html
  drone-slam-view <bagフォルダ>                              # RViz2 で画像と点群を見る
  drone-slam-run all <bagフォルダ> --calib my_calib.yaml     # 全手法でリプレイ
  drone-slam-update                                          # 最新版に更新
キャリブレーションのひな形: $REPO/common/config/calib.yaml
MSG
