#!/usr/bin/env bash
# =============================================================================
# fetch_artifacts.sh —— 把某次 lab5-ci 运行的产物拉回本地
#
# 用法：
#   bash fetch_artifacts.sh                 # 取最近一次已完成的运行
#   bash fetch_artifacts.sh <run_id>        # 取指定运行
#   bash fetch_artifacts.sh <run_id> <dest> # 指定落地目录
#   DEST=... bash fetch_artifacts.sh
#
# 落地内容（默认 DEST=<lab5>/ci_runs/lab5_run<id>）：
#   report.md            两张统计表 + 图清单 + 构建证据（可直接往团队报告里贴）
#   fig/*.png            D4/D5 的 keypoints / matches / 内点外点图，D6/D7 的 LK 轨迹图，D8 光流图
#   pair_*.log           每个描述子一次 two_frames_tracking 的完整 stdout（含统计 7 行）
#   video_*.log          视频模式的累计统计
#   stage_*.log          每个 stage 的日志（apt / build_deps / build_main / ...）
#   build_evidence.txt   编译器版本、ldd 出来的 OpenCV 版本、可执行文件清单
#
# 顺带记一笔 run 元信息（run_meta.txt），便于事后对账「哪一趟是什么参数」。
# =============================================================================
set -euo pipefail

REPO="${REPO:-masai114514/vnav-lab5-ci}"
WORKFLOW="${WORKFLOW:-lab5-ci.yml}"
ARTIFACT="${ARTIFACT:-lab5-results}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB5="$(cd "$HERE/.." && pwd)"

RUN_ID="${1:-}"
if [ -n "${2:-}" ]; then
  DEST="$2"
elif [ -n "${DEST:-}" ]; then
  : # 用外部传入的
elif [ -n "$RUN_ID" ]; then
  # 目录名带上 run_id 尾部，避免「同一天两个 run 互相覆盖」
  DEST="$LAB5/ci_runs/lab5_run${RUN_ID}"
else
  DEST="$LAB5/ci_runs/lab5_r$(date +%m%d)"
fi

say()  { printf '\033[1;36m[fetch]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[fetch]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fetch] %s\033[0m\n' "$*" >&2; exit 1; }

say "仓        = $REPO"
say "工作流    = $WORKFLOW"
say "落地目录  = $DEST"

# ---- 定 run_id -------------------------------------------------------------
if [ -z "$RUN_ID" ]; then
  say "找最近一次运行 ..."
  RUN_ID="$(gh run list --repo "$REPO" --workflow "$WORKFLOW" --limit 30 \
              --json databaseId,conclusion,status,createdAt \
              --jq '[.[] | select(.status == "completed")][0].databaseId' 2>/dev/null || true)"
  [ -n "$RUN_ID" ] && [ "$RUN_ID" != "null" ] || \
    die "找不到已完成的运行。用 gh run list --repo $REPO --workflow $WORKFLOW 看看"
fi

say "run_id    = $RUN_ID"
mkdir -p "$DEST"

# ---- run 元信息 ------------------------------------------------------------
{
  echo "repo        : $REPO"
  echo "workflow    : $WORKFLOW"
  echo "run_id      : $RUN_ID"
  echo "下载时间    : $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo
  echo "--- gh run view ---"
  gh run view "$RUN_ID" --repo "$REPO" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' || true
} > "$DEST/run_meta.txt"
say "元信息 → $DEST/run_meta.txt"
grep -E "^(✓|X|✗|Status|Conclusion|run_id)" "$DEST/run_meta.txt" | head -5 || true

# ---- 拉 artifact -----------------------------------------------------------
AID="$(gh api "repos/$REPO/actions/runs/$RUN_ID/artifacts" \
        --jq ".artifacts[] | select(.name==\"$ARTIFACT\") | .id" 2>/dev/null | head -1 || true)"
if [ -n "$AID" ]; then
  SZB="$(gh api "repos/$REPO/actions/artifacts/$AID" --jq '.size_in_bytes' 2>/dev/null || echo '?')"
  say "artifact id=$AID  size=${SZB} bytes ($(( ${SZB:-0} / 1048576 )) MB)"
else
  warn "没查到名为 '$ARTIFACT' 的 artifact（可能没传成功）"
fi

say "下载 artifact '$ARTIFACT' ..."
ok=0
if gh run download "$RUN_ID" --repo "$REPO" --name "$ARTIFACT" --dir "$DEST"; then
  ok=1
fi

if [ "$ok" != 1 ] && [ -n "$AID" ]; then
  warn "gh run download 失败 —— 退回 REST zip 端点（大 artifact 更稳）"
  TMPZ="$(mktemp -d)/artifact.zip"
  if curl -fL --retry 3 --retry-delay 5 --max-time 1800 \
        -H "Authorization: Bearer $(gh auth token)" \
        -H "Accept: application/vnd.github+json" \
        -o "$TMPZ" \
        "https://api.github.com/repos/$REPO/actions/artifacts/$AID/zip"; then
    unzip -q -o "$TMPZ" -d "$DEST" && ok=1
  fi
fi
[ "$ok" = 1 ] || die "artifact 下载失败"

# ---- 整理：artifact 可能带一层 out/ ----------------------------------------
if [ -d "$DEST/out" ]; then
  say "artifact 里带了一层 out/，摊平"
  shopt -s dotglob nullglob
  mv "$DEST/out/"* "$DEST/" 2>/dev/null || true
  rmdir "$DEST/out" 2>/dev/null || true
  shopt -u dotglob nullglob
fi

say "落地内容："
ls -la "$DEST" | sed 's/^/    /'

echo
say "快速判读："
if [ -f "$DEST/report.md" ]; then
  sed -n '/^## 1\./,/^## 3\./p' "$DEST/report.md" | head -32 | sed 's/^/    /'
else
  warn "  没有 report.md"
fi
if [ -d "$DEST/fig" ]; then
  echo "    图：$(ls "$DEST/fig" | wc -l | tr -d ' ') 张 → $DEST/fig/"
  ls "$DEST/fig" | head -12 | sed 's/^/      /'
else
  warn "    没有 fig/"
fi
if grep -qi "patented\|nonfree" "$DEST"/pair_SURF.log 2>/dev/null; then
  warn "  SURF 报了 nonfree 错误（看 pair_SURF.log）"
fi

echo
say "下一步："
echo "    # 在浏览器里看这次运行的全部日志"
echo "    gh run view $RUN_ID --repo $REPO --log"
echo "    # 看图（macOS / WSL 各一条）"
echo "    open $DEST/fig 2>/dev/null || explorer.exe \$(wslpath -w $DEST/fig 2>/dev/null) 2>/dev/null || true"
