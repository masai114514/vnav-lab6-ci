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

say "安装编译依赖（apt）"
# 镜像 ros:melodic-ros-base 里**没有** catkin_tools（catkin 命令来自
# python-catkin-tools 包）。第一次冒烟 run 43041279738 就是在这里挂的：
# 四行 "catkin: command not found"，编译实际没发生，节点自然起不来，
# 但脚本用 `|| true` 兜着，整体还报 success —— 属于"假绿"。
# 这一段照抄 lab6_ci.sh 的 s_apt，只保留编主工作区真正用到的。
if ! command -v catkin >/dev/null 2>&1; then
  export DEBIAN_FRONTEND=noninteractive
  codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-bionic}}")"
  echo "发行版代号 = $codename"

  # bionic 的免费仓库 2023-05 之后搬到 old-releases 了。
  #
  # 之前两次尝试都失败（run 43042466692 / 43043532048）：
  #   · 第一次只 sed 了 sources.list，但 update 退出码是 0（ROS 源是好的），
  #     于是修复分支没进去，install 时 libeigen3-dev 等全部 "no installation candidate"；
  #   · 第二次无条件 sed，仍然一样 —— 说明 sed 根本没改到东西。
  # 所以这里不猜了：先打印原文件，再**直接重写**一份已知可用的 bionic 源。
  echo "---- 原 /etc/apt/sources.list ----"
  cat /etc/apt/sources.list 2>/dev/null || echo "(不存在)"
  echo "---- /etc/apt/sources.list.d ----"
  ls -la /etc/apt/sources.list.d/ 2>/dev/null || echo "(空)"

  cp -n /etc/apt/sources.list /etc/apt/sources.list.orig 2>/dev/null || true
  cat > /etc/apt/sources.list <<'EOF'
deb http://old-releases.ubuntu.com/ubuntu/ bionic main restricted universe multiverse
deb http://old-releases.ubuntu.com/ubuntu/ bionic-updates main restricted universe multiverse
deb http://old-releases.ubuntu.com/ubuntu/ bionic-security main restricted universe multiverse
EOF
  echo "---- 重写后的 sources.list ----"
  cat /etc/apt/sources.list

  # ROS 仓库 key 2025-06-01 过期，先修后 update
  mkdir -p /usr/share/keyrings
  curl -fsSL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg || warn "取 ROS key 失败"
  rm -f /etc/apt/sources.list.d/ros1-latest.list /etc/apt/sources.list.d/ros-latest.list
  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu ${codename} main" \
    > /etc/apt/sources.list.d/ros1-latest.list

  apt-get update -qq 2>&1 | tail -10 || warn "apt-get update 有问题（继续尝试装包）"

  apt-get install -y -qq --no-install-recommends \
    build-essential cmake git curl patch pkg-config \
    autoconf automake libtool m4 \
    python-dev python-numpy python-catkin-pkg python-empy python-nose \
    python-catkin-tools \
    libboost-python-dev libboost-system-dev libboost-thread-dev libboost-filesystem-dev \
    libeigen3-dev libtbb-dev libgtk2.0-dev \
    libjpeg-dev libpng-dev libtiff5-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    ros-melodic-cv-bridge ros-melodic-message-filters ros-melodic-image-transport ros-melodic-rosbag \
    ros-melodic-tf ros-melodic-tf-conversions ros-melodic-nav-msgs 2>&1 | tail -15
fi
command -v catkin >/dev/null 2>&1 || { warn "catkin 仍不可用，中止（否则会假绿）"; exit 5; }
echo "catkin = $(command -v catkin)  $(catkin --version 2>&1 | head -1)"
# Eigen 缺了 opengv 一定编不过，早点报出来，别等到 8 分钟后的 cmake
[ -d /usr/include/eigen3 ] || warn "没有 /usr/include/eigen3 —— opengv 会失败"
for p in cmake g++ git; do command -v "$p" >/dev/null || warn "缺 $p"; done

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
# cv_bridge / eigen_catkin 允许失败：镜像里可能自带可用的
catkin build cv_bridge    -j"$(nproc)" --no-status || warn "cv_bridge 编译失败"
catkin build eigen_catkin -j"$(nproc)" --no-status || warn "eigen_catkin 编译失败"
# opengv 和 lab_5/lab_6 必须成功 —— 之前用 `|| warn` 兜着，导致编译没发生
# 也照样报 success（run 43041279738 的假绿就是这么来的）。
catkin build opengv      -j"$(nproc)" --no-status --summarize || { warn "opengv 编译失败"; exit 6; }
catkin build lab_5 lab_6 -j"$(nproc)" --no-status --summarize || { warn "lab_5/lab_6 编译失败"; exit 7; }
cd "$ROOT"
# shellcheck disable=SC1091
source "$WS_MAIN/devel/setup.bash"
ls -la "$WS_MAIN/devel/lib/lab_6/" 2>/dev/null || true
[ -x "$WS_MAIN/devel/lib/lab_6/pose_estimation" ] || { warn "没有 build 出 pose_estimation"; exit 8; }

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
if [ -f "$CSV" ] && [ "$(wc -l < "$CSV")" -gt 1 ]; then
  echo "行数 = $(wc -l < "$CSV")"
  head -3 "$CSV"
  RC=0
else
  warn "没有生成 CSV（或用例数为 0）"
  RC=9
fi
echo "---------- 节点日志末尾 ----------"
tail -25 "$OUT/office_node_est${EST}.log" 2>/dev/null || true
echo "---------- 播放日志末尾 ----------"
tail -10 "$OUT/office_play.log" 2>/dev/null | tr '\r' '\n' | tail -6 || true
exit "${RC:-0}"
