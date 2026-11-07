#!/usr/bin/env bash
# =============================================================================
# fetch_artifacts.sh —— 把某次 lab6-ci 运行的产物拉回本地
#
# 用法：
#   bash fetch_artifacts.sh                 # 取最近一次已完成的运行
#   bash fetch_artifacts.sh <run_id>        # 取指定运行
#   bash fetch_artifacts.sh <run_id> <dest> # 指定落地目录
#   DEST=... bash fetch_artifacts.sh
#
# 落地内容（默认 DEST=<lab6>/ci_out/lab6_run<id>）：
#   rpe_0.csv            5 点法（Nistér）逐帧旋转/平移误差
#   rpe_1.csv            8 点法
#   rpe_2.csv            2 点法（translation-only）
#   rpe_3.csv            Arun 3D-3D
#   rpe_{0,2}_norefine.csv  对照组（_refine:=false），用于分离非线性精化的影响
#   run_*.log            每个估计器的完整节点日志
#   build_evidence.txt   编译器 / OpenCV / opengv / ldd 证据
#   report.md            帧数 + 编译证据 + 各 stage 的 PASS/FAIL
#
# 顺带记一笔 run 元信息（run_meta.txt），便于事后对账「哪一趟是什么参数」。
# =============================================================================
set -euo pipefail

REPO="${REPO:-masai114514/vnav-lab6-ci}"
WORKFLOW="${WORKFLOW:-lab6-ci.yml}"
ARTIFACT="${ARTIFACT:-lab6-results}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB6="$(cd "$HERE/.." && pwd)"

RUN_ID="${1:-}"
if [ -n "${2:-}" ]; then
  DEST="$2"
elif [ -n "${DEST:-}" ]; then
  : # 用外部传入的
elif [ -n "$RUN_ID" ]; then
  # 目录名带上 run_id 尾部，避免「同一天两个 run 互相覆盖」
  DEST="$LAB6/ci_out/lab6_run${RUN_ID}"
else
  DEST="$LAB6/ci_out/lab6_r$(date +%m%d)"
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
  sed -n '/^## 各估计器的 CSV 行数/,/^## 编译证据/p' "$DEST/report.md" | head -12 | sed 's/^/    /'
else
  warn "  没有 report.md"
fi
n_csv=$(ls "$DEST"/rpe_*.csv 2>/dev/null | wc -l | tr -d ' ')
say "    CSV：$n_csv 个（每个估计器一份；_norefine 是对照组）"
if grep -qi "^\- FAIL: .\+" "$DEST/report.md" 2>/dev/null; then
  warn "  有 stage 失败 —— 看 $DEST/report.md 的 'stage 结果' 段"
fi

echo
say "下一步："
echo "    # 在浏览器里看这次运行的全部日志"
echo "    gh run view $RUN_ID --repo $REPO --log"
echo "    # 直接用 CSV 重画报告里的图 / 重算中位数"
echo "    python3 $LAB6/offline/plot_rpe.py --csv $DEST/rpe_0.csv --out $DEST/fig"
