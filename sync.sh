#!/usr/bin/env bash
# =============================================================================
# sync.sh —— 把 lab6 里 CI 需要的那部分拷进本仓的 bundle/
#
# 本仓是独立仓（跑在 GitHub Actions 上），源码正本在 ../（= lab6/）。
# 改完 .cpp / .h / launch / CMakeLists 之后先 sync 再 push，否则 CI 编的是旧代码。
#
# 同步三个目录：
#   lab6/        —— 要提交的 ROS 包 lab_6（D3/D4/D5 的实现）
#   lab_5/       —— 上一实验的特征跟踪包，lab_6 通过 tracker_shim 依赖它
#   dependencies/—— opencv3_catkin（已打 nonfree 补丁）+ gflags_catkin
# opengv / eigen_catkin / cv_bridge 不镜像，CI 里 git clone。
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SRC:-$(cd "$HERE/.." && pwd)}"
DEST="$HERE/bundle"
PKGS=(lab6)

DRY=""
[ "${1:-}" = "--dry-run" ] && DRY="--dry-run"

say() { printf '\033[1;36m[sync]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[sync] %s\033[0m\n' "$*" >&2; exit 1; }

command -v rsync >/dev/null 2>&1 || die "没有 rsync"
mkdir -p "$DEST"

for p in "${PKGS[@]}"; do
  [ -d "$SRC/$p" ] || die "SRC=$SRC 下没有 $p/"
  say "同步 $p/"
  rsync -a --delete $DRY \
    --exclude '__pycache__/' --exclude '*.pyc' --exclude '.DS_Store' \
    "$SRC/$p/" "$DEST/$p/"
done

# dependencies（opencv3_catkin 已打 nonfree 补丁 + gflags_catkin）与 lab_5 都来自
# 上一实验的目录：lab5/dependencies、lab5/feature_tracking
DEP="${DEP_SRC:-$(cd "$HERE/../.." && pwd)/lab5/dependencies}"
[ -d "$DEP" ] || die "找不到 dependencies 源：$DEP"
say "同步 dependencies/ <- $DEP"
rsync -a --delete $DRY \
  --exclude '__pycache__/' --exclude '*.pyc' --exclude '.DS_Store' \
  "$DEP/" "$DEST/dependencies/"

# lab_5 来自 ../lab5/feature_tracking（包名是 lab_5）
L5="${LAB5_SRC:-$(cd "$HERE/../.." && pwd)/lab5/feature_tracking}"
[ -d "$L5" ] || die "找不到 lab_5 源：$L5"
say "同步 lab_5/ <- $L5"
rsync -a --delete $DRY \
  --exclude '__pycache__/' --exclude '*.pyc' --exclude '.DS_Store' \
  "$L5/" "$DEST/lab_5/"

say "检查未实现标记："
if grep -rn "begin solution" "$DEST/lab6/src" 2>/dev/null | grep -q .; then
  say "  源码里仍有 begin solution 段（这是正常的，已经填过实现）"
fi

say "镜像现状："
for p in lab6 lab_5 dependencies; do
  [ -d "$DEST/$p" ] || continue
  printf '    %-14s %4s 文件\n' "$p" "$(find "$DEST/$p" -type f | wc -l | tr -d ' ')"
done
say "完成。下一步：python3 push_via_api.py"
