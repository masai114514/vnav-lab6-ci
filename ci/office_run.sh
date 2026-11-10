#!/usr/bin/env bash
# =============================================================================
# office_run.sh —— 在官方 office.bag（已转成 ROS1）上跑 lab_6 节点
#
# 与 lab6_ci.sh 的区别：lab6_ci.sh 是"从 0 跑完整条 TUM 流水线"（13 个 stage，
# 含下载 TUM、四个估计器、汇总）；这个脚本只做三件事：
#   (1) 编译主工作区（ws_deps 从缓存恢复，不在这里编 OpenCV）
#   (2) 起 roscore + 节点
#   (3) rosbag play 前 N 秒
# 目的是先确认"官方数据能跑通、参数对、数出得来"，再决定要不要全量跑。
#
# 关键参数为什么不用覆盖：
#   pose_estimation.cpp 里内参默认值就是 Tesse 的 415.69 / (360,240) / 无畸变，
#   深度在 depthToMeters() 里遇到 CV_32F 会原样按米返回（office.bag 是 32FC1），
#   话题名也本来就是 /tesse/* —— 所以跑官方数据不需要动任何参数。
#
# 用法:
#   office_run.sh <bag> [estimator] [max_frames] [rate] [duration]
# 例:
#   office_run.sh data/office_ros1.bag 0 240 0.15 12
# =============================================================================
set -uo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BAG=${1:?需要 bag 路径}
EST=${2:-0}
NFRAMES=${3:-240}
RATE=${4:-0.15}
DUR=${5:-12}
CSV="$ROOT/out/office_est${EST}.csv"

OUT="$ROOT/out"; mkdir -p "$OUT"
WS_DEPS="$ROOT/ws_deps"
WS_MAIN="$ROOT/ws_main"
BUNDLE="$ROOT/bundle"

say()  { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*" >&2; }

say "环境"
# shellcheck disable=SC1091
source /opt/ros/melodic/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO  nproc=$(nproc)"
df -h "$ROOT" | tail -1
[ -f "$BAG" ] || { warn "找不到 bag: $BAG"; exit 2; }
ls -la "$BAG"

say "依赖工作区（应来自 Actions 缓存）"
if [ ! -d "$WS_DEPS/install" ]; then
  warn "ws_deps/install 不存在 —— 先让一次 lab6-ci 跑起来把 OpenCV 缓存落下"
  exit 3
fi
# shellcheck disable=SC1091
source "$WS_DEPS/install/setup.bash"

say "编译主工作区"
rm -rf "$WS_MAIN"
mkdir -p "$WS_MAIN/src"
cp -r "$BUNDLE/lab_5" "$WS_MAIN/src/lab_5"
cp -r "$BUNDLE/lab6"  "$WS_MAIN/src/lab_6"

rm -rf /tmp/vision_opencv
git clone --depth 1 -b melodic https://github.com/ros-perception/vision_opencv.git /tmp/vision_opencv 2>/dev/null \
  || git clone --depth 1 https://github.com/ros-perception/vision_opencv.git /tmp/vision_opencv
cp -r /tmp/vision_opencv/cv_bridge "$WS_MAIN/src/cv_bridge"
git clone --depth 1 https://github.com/ethz-asl/opengv.git       "$WS_MAIN/src/opengv"       2>/dev/null || true
git clone --depth 1 https://github.com/ethz-asl/eigen_catkin.git "$WS_MAIN/src/eigen_catkin" 2>/dev/null || true

cd "$WS_MAIN"
catkin config --extend "$WS_DEPS/install" --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build cv_bridge    -j"$(nproc)" --no-status || true
catkin build eigen_catkin -j"$(nproc)" --no-status || true
catkin build opengv       -j"$(nproc)" --no-status --summarize || warn "opengv 编译有问题"
catkin build lab_5 lab_6  -j"$(nproc)" --no-status --summarize || warn "lab_5/lab_6 编译有问题"
cd "$ROOT"
# shellcheck disable=SC1091
source "$WS_MAIN/devel/setup.bash"
ls -la "$WS_MAIN/devel/lib/lab_6/" 2>/dev/null || true

say "起 roscore"
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roscore > "$OUT/office_roscore.log" 2>&1 &
RC_PID=$!
sleep 8
kill -0 "$RC_PID" 2>/dev/null || { warn "roscore 没起来"; tail -20 "$OUT/office_roscore.log"; exit 4; }

say "起节点 estimator=$EST max_frames=$NFRAMES csv=$CSV"
rosrun lab_6 pose_estimation \
  _pose_estimator:="$EST" \
  _use_ransac:=true \
  _scale_translation:=true \
  _refine:=true \
  _max_frames:="$NFRAMES" \
  _csv_path:="$CSV" \
  /ground_truth_pose:=/tesse/odom \
  /rgb_images_topic:=/tesse/left_cam/rgb/image_raw \
  /depth_images_topic:=/tesse/depth_cam/mono/image_raw \
  > "$OUT/office_node_est${EST}.log" 2>&1 &
NODE_PID=$!
sleep 10
kill -0 "$NODE_PID" 2>/dev/null || warn "节点已经退出（可能启动失败），继续取日志"

say "播放 ${DUR}s（rate=$RATE）"
rosbag play -r "$RATE" -u "$DUR" "$BAG" > "$OUT/office_play.log" 2>&1 || warn "rosbag play 退出码 $?"

say "等节点结束（最多 180s）"
for _ in $(seq 1 90); do
  kill -0 "$NODE_PID" 2>/dev/null || break
  sleep 2
done
kill "$NODE_PID" 2>/dev/null || true
kill "$RC_PID"   2>/dev/null || true

say "结果"
if [ -f "$CSV" ]; then
  echo "行数 = $(wc -l < "$CSV")"
  head -3 "$CSV"
else
  warn "没有生成 CSV"
fi
echo "---------- 节点日志末尾 ----------"
tail -25 "$OUT/office_node_est${EST}.log" 2>/dev/null || true
echo "---------- 播放日志末尾 ----------"
tail -10 "$OUT/office_play.log" 2>/dev/null || true
