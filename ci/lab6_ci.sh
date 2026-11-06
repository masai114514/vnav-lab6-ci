#!/usr/bin/env bash
# =============================================================================
# lab6_ci.sh —— LAB6（相对位姿估计：5pt / 8pt / 2pt / Arun）的无头 CI 主脚本
#
# 为什么要有它
# ------------
# LAB6 的团队部分（D3–D5）必须在「ROS1 + OpenCV 3.4.2(nonfree) + OpenGV」里跑：
#   ① 编译证据：lab_6 + opengv 真的编得过
#   ② 四个估计器（5pt/8pt/2pt/Arun）在真实序列上的逐帧 RPE CSV
# 本机 macOS 装不了这套环境（原因见 docs/环境与踩坑.md），所以搬到 GitHub
# Actions 的 x86_64 runner + `ros:melodic` 容器里跑，和 lab3/lab4 同一套思路。
#
# 设计要点（每条都是踩过或预判过的坑，别随手改回去）
# ---------------------------------------------------------------
#  1. ROS1 的 apt 仓库 key 在 2025-06-01 过期 → 先重装 key 再 apt update，
#     否则 `apt-get update` 直接 GPG error，后面一步都跑不了。
#  2. OpenCV 3.4.2 从源码编 ≈ 20 分钟 → 用 actions/cache 缓存 ws_deps/install，
#     脚本靠「install/setup.bash 存在」判断命中，命中就整段跳过。
#  3. 系统的 cv_bridge 是链接 OpenCV 3.2 编出来的。让 lab_5 用 3.4.2 而 cv_bridge
#     用 3.2 是 ODR 违规（同一进程里两份 cv::Mat）。ethz-asl 的 opencv3_catkin
#     README 明确要求重编下游 → 从 vision_opencv 源码编 cv_bridge。
#  4. 官方 6b 数据集已从 GitHub 消失（LFS 404）→ 用 ci_tools/mp4_to_bag 把仓库里
#     自带的 mp4 打成同话题同类型的 bag，官方 video_tracking.launch 原样能跑。
#  5. runner 没有显示器 → 全部 interactive:=false，图由 saveImage /
#     lk_tracks_*.png 落盘，不靠窗口截图。
#  6. `bash x.sh | tee log` 的退出码来自 tee → workflow 里必须 set -o pipefail。
# =============================================================================
set -uo pipefail

ROOT="${GITHUB_WORKSPACE:-$PWD}"
BUNDLE="$ROOT/bundle"
OUT="$ROOT/out"
WS_DEPS="$ROOT/ws_deps"
WS_MAIN="$ROOT/ws_main"

mkdir -p "$OUT" "$OUT/fig"

PASS=()
FAIL=()

hr() { printf '\n%s\n' "------------------------------------------------------------------------"; }
banner() { hr; printf '## %s\n' "$*"; hr; }
say()  { printf '\033[1;36m[ci]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ci]\033[0m %s\n' "$*"; }

# 跑一个 stage：日志同时进 out/stage_<name>.log 与 out/console.log。
# 失败只记账，不中断 —— 一趟 25 分钟的机器时间不该因为最后一个测试挂掉就白跑。
stage() {
  local name="$1"; shift
  banner "STAGE $name : $*"
  local log="$OUT/stage_${name}.log"
  local t0=$SECONDS rc=0
  # 退出码必须取 PIPESTATUS[0]（= 子 shell），不能取整条管道的状态：
  # 管道最后一个是 `tee`，它永远成功。Run #2 里 lab_5 的 cmake 明明失败、
  # track_features 根本没编出来，四个 stage 却全报 PASS，就栽在这里。
  set +e
  ( set -euo pipefail; "$@" ) 2>&1 | tee "$log"
  rc=${PIPESTATUS[0]}
  set -e
  if [ "$rc" -eq 0 ]; then
    say "PASS $name ($((SECONDS - t0))s)"
    PASS+=("$name")
  else
    warn "FAIL $name ($((SECONDS - t0))s, rc=$rc) —— 见 out/stage_${name}.log"
    FAIL+=("$name")
  fi
  cat "$log" >> "$OUT/console.log" || true
  return 0
}

# 运行时前置检查：没有 track_features 就什么都跑不了。
# 别让 roslaunch 在那儿干等 120 s 再被 timeout 杀掉 —— Run #2 里四个描述子
# 白烧了 20 分钟，日志里全是 "cannot launch node of type [lab_5/track_features]"。
need_bin() {
  local bin="$1"
  if [ ! -x "$bin" ]; then
    echo "✗ 缺少可执行文件：$bin" >&2
    echo "  lab_5 没编过 —— 先看 out/stage_build_main.log 里的 cmake 报错" >&2
    return 1
  fi
  return 0
}

# 自己起一个 master，而不是让 roslaunch 起。
#
# 为什么不用 roslaunch（Run #8 实测）：
#   节点明明 3.5 s 就跑完、图也落盘、统计也打了，roslaunch 却在节点退出后又干等
#   ~115 s，最后被 `timeout -s INT 120` 砍掉，退出码 124 → 四个描述子全被判 FAIL，
#   白烧 8 分钟。原因在 roslaunch 自己的收尾（关 rosout/master 那段），跟我们的
#   节点无关。改成 roscore + rosrun：节点一退出 rosrun 立刻返回，没有可挂的东西。
MASTER_PID=""
start_master() {
  if rosnode list >/dev/null 2>&1; then
    return 0
  fi
  : > "$OUT/roscore.log"
  roscore >> "$OUT/roscore.log" 2>&1 &
  MASTER_PID=$!
  local i
  for i in $(seq 1 60); do
    if rosnode list >/dev/null 2>&1; then
      say "roscore 就绪（pid=$MASTER_PID）"
      return 0
    fi
    sleep 0.5
  done
  warn "roscore 30 s 没起来 —— 见 out/roscore.log"
  tail -20 "$OUT/roscore.log" || true
  return 1
}

stop_master() {
  [ -n "${MASTER_PID:-}" ] || return 0
  kill -INT "$MASTER_PID" 2>/dev/null || true
  sleep 1
  kill -KILL "$MASTER_PID" 2>/dev/null || true
  wait "$MASTER_PID" 2>/dev/null || true
  MASTER_PID=""
}

# 等一个后台节点退出，最多 $2 秒；超时就 SIGINT，再不退就 KILL。
# 绝不用裸 `wait`：那是在等一个可能永不退出的进程（Run #2 挂过 37 分钟）。
wait_node() {
  local pid="$1" limit="${2:-60}" i
  for i in $(seq 1 "$limit"); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  warn "节点 ${limit}s 没退出，SIGINT"
  kill -INT "$pid" 2>/dev/null || true
  for i in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  warn "SIGINT 也不退，KILL"
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  return 1
}

# 给外部命令套一层硬超时。用 INT 而不是 TERM：rosnode / catkin 收到 INT 会走
# 正常收尾，日志才留得下来；20 s 还不退才 KILL。
#
# 为什么必须有它：Run #6 里 tum_play 的「等订阅者」是个死循环（节点崩了就永远
# 没有订阅者），冒烟阶段又是前台调用，整个 job 从 8 分钟直接挂到 47 分钟。
# 任何可能无限等待的外部命令都要经过这里。
tmo() {
  local lim="$1"; shift
  timeout --signal=INT --kill-after=20 "$lim" "$@"
}

# =============================================================================
# S1 环境信息
# =============================================================================
s_env() {
  echo "vCPU     = $(nproc)"
  free -m | awk '/Mem:/{printf "内存     = %s MB\n", $2}'
  grep -E '^(PRETTY_NAME|VERSION_CODENAME)' /etc/os-release || true
  echo "ROOT     = $ROOT"
  echo "catkin   = $(command -v catkin || echo '(没有 catkin_tools)')"
  echo "cmake    = $(cmake --version | head -1)"
  echo "gcc      = $(gcc -dumpversion)"
  echo "--- 已装的 ROS 包数: $(ls /opt/ros/melodic/share 2>/dev/null | wc -l)"
}

# =============================================================================
# S2 apt：修 key + 装依赖
# =============================================================================
s_apt() {
  export DEBIAN_FRONTEND=noninteractive
  local codename
  codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-bionic}}")"
  say "发行版代号 = $codename"

  # 第 1 步：ROS 仓库 key（2025-06-01 过期）
  if ! apt-get update -qq >/dev/null 2>&1; then
    warn "apt-get update 失败 —— 重装 ROS key 并重写源"
    mkdir -p /usr/share/keyrings
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    rm -f /etc/apt/sources.list.d/ros1-latest.list \
          /etc/apt/sources.list.d/ros-latest.list
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu ${codename} main" \
      > /etc/apt/sources.list.d/ros1-latest.list
    if ! apt-get update -qq; then
      warn "仍失败 —— 把 Ubuntu 源切到 old-releases 再试"
      sed -i -e 's|http://archive.ubuntu.com|http://old-releases.ubuntu.com|g' \
             -e 's|http://security.ubuntu.com|http://old-releases.ubuntu.com|g' \
             /etc/apt/sources.list
      apt-get update -qq
    fi
  fi
  say "apt update OK"

  apt-get install -y -qq --no-install-recommends \
    build-essential cmake git curl patch pkg-config \
    autoconf automake libtool m4 \
    python-dev python-numpy python-catkin-pkg python-empy python-nose \
    python-catkin-tools \
    libboost-python-dev libboost-system-dev libboost-thread-dev libboost-filesystem-dev \
    libeigen3-dev libtbb-dev libgtk2.0-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    ros-melodic-cv-bridge ros-melodic-message-filters ros-melodic-image-transport ros-melodic-rosbag \
    ros-melodic-tf ros-melodic-tf-conversions ros-melodic-nav-msgs \
    xvfb x11-utils imagemagick ffmpeg 2>&1 | tail -20

  say "---------- 关键依赖版本 ----------"
  python --version 2>&1 || true
  catkin --version 2>&1 | head -2 || true
  if [ -f /opt/ros/melodic/include/cv_bridge/cv_bridge.h ]; then
    echo "cv_bridge 头文件: 有"
  fi
}

# =============================================================================
# S3 构建依赖工作区（OpenCV 3.4.2 + glog/gflags/catkin_simple）
# =============================================================================

# 某个依赖有没有真的产出 CMake config。
# 不用 `find | grep -q`：grep 提前退出会给 find 送 SIGPIPE，pipefail 下变成 141。
dep_ok() {
  local f
  f="$(find "$WS_DEPS/install" -name "$1Config.cmake" -print -quit 2>/dev/null)"
  [ -n "$f" ]
}

# 缓存里的依赖工作区是否「真的」可用 —— 必须是四项 Config + libopencv 全在。
#
# 只看 install/setup.bash 会误判。实际踩到的：Run #3 在 10:37 起、10:38:59 被
# concurrency 取消，它的 `actions/cache/save`（当时挂在 always() 上）把刚建了一半的
# ws_deps/install（只有 catkin_simple，**120 KB**）存成了正式 key。Run #4 于是
# restore 报 cache-hit=true —— 但那份东西连 OpenCV 都没有，脚本只好重编 12 分钟，
# 而 save 又因为 cache-hit=true 被跳过，等于永远修不好。所以命中判定必须够严。
deps_ready() {
  [ -f "$WS_DEPS/install/setup.bash" ] || return 1
  ls "$WS_DEPS/install/lib"/libopencv_core.so* >/dev/null 2>&1 || return 1
  local d
  for d in opencv3_catkin glog_catkin gflags_catkin catkin_simple; do
    dep_ok "$d" || return 1
  done
  return 0
}

s_build_deps() {
  if deps_ready; then
    say "deps 缓存命中（$WS_DEPS/install 已就绪），跳过 OpenCV 编译"
    du -sh "$WS_DEPS/install"
    # catkin_tools 的 --extend 要找这个标记文件，cache 没存到就补一个
    touch "$WS_DEPS/install/.catkin"
    ls "$WS_DEPS/install/lib" 2>/dev/null | grep -c '^libopencv' \
      | awk '{print "  已缓存 libopencv*.so : "$1" 个"}' || true
    return 0
  fi

  if [ -f "$WS_DEPS/install/setup.bash" ]; then
    warn "缓存里的 install 不完整（半成品，见 deps_ready 注释）—— 丢掉重建"
  fi
  say "deps 缓存未命中 —— 开始编译（首次约 20 分钟）"
  rm -rf "$WS_DEPS"
  mkdir -p "$WS_DEPS/src"

  # 课程给的依赖（opencv3_catkin 已打 OPENCV_ENABLE_NONFREE=ON 的补丁）
  cp -r "$BUNDLE/dependencies/." "$WS_DEPS/src/"

  # rosinstall 里另外三个：catkin_simple / glog_catkin / gflags_catkin
  try_clone() {
    local url="$1" dest="$2" branch="${3:-master}"
    for i in 1 2 3; do
      if git clone --depth 1 --branch "$branch" "$url" "$dest" 2>/dev/null \
         || git clone --depth 1 "$url" "$dest" 2>/dev/null; then
        return 0
      fi
      warn "clone 失败（第 $i 次）：$url"
      rm -rf "$dest"
      sleep 5
    done
    return 1
  }
  try_clone https://github.com/catkin/catkin_simple.git   "$WS_DEPS/src/catkin_simple"
  try_clone https://github.com/ethz-asl/glog_catkin.git   "$WS_DEPS/src/glog_catkin"
  try_clone https://github.com/ethz-asl/gflags_catkin.git "$WS_DEPS/src/gflags_catkin"

  # 2 vCPU 上 `make -j8` 会把内存打爆（imgproc 的单个 TU 就要 1 GB+）
  sed -i 's/make -j8/make -j$(nproc)/g' "$WS_DEPS/src/opencv3_catkin/CMakeLists.txt"
  grep -n 'OPENCV_ENABLE_NONFREE\|make -j' "$WS_DEPS/src/opencv3_catkin/CMakeLists.txt"

  cd "$WS_DEPS"
  catkin config --install --extend /opt/ros/melodic \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  # catkin_simple 是 CMake 宏包，必须单独先建好
  catkin build catkin_simple -j1
  # OpenCV 的 ExternalProject 要下 ~120 MB 源码，超时设大一点。
  # 不用 `set -e` 直接中断：glog 挂了也要把 OpenCV 编完（它是 20 分钟的大头），
  # 后面再用 shim 兜 glog，这样下一趟缓存还能命中。
  local rc_deps=0
  # 2400 s 上限：OpenCV 那 20 分钟本来就慢，但不能无限等（编译卡死也要留证据）
  tmo 2400 catkin build -j"$(nproc)" --no-status --summarize || rc_deps=$?
  cd "$ROOT"

  # ---------- 校验四个依赖真的产出了 CMake config ----------
  # 只看 install/setup.bash 是否存在是不够的：Run #2 里 glog_catkin 编译失败
  # （bionic 缺 autoreconf），setup.bash 照样生成，于是 deps 被误判为成功，
  # 一路拖到 s_build_main 才炸在 `Could not find glog_catkin`。
  local missing=() d
  for d in opencv3_catkin glog_catkin gflags_catkin catkin_simple; do
    if dep_ok "$d"; then
      echo "  ✓ $d"
    else
      echo "  ✗ $d  （没有 $dConfig.cmake）"
      missing+=("$d")
    fi
  done

  # ---------- glog 兜底 ----------
  # lab_5 的源码里其实一行 glog 都没用（grep 过），只是 CMakeLists 里
  # find_package(glog_catkin) + 链接 ${catkin_LIBRARIES}。所以 glog_catkin
  # 用官方源码编不出来时，用系统 glog（bionic 正好也是 0.3.5）做一个同名
  # shim 包，接口等价、3 秒建好，完全不碰课程给的源。
  if printf '%s\n' "${missing[@]:-}" | grep -qx glog_catkin; then
    warn "glog_catkin 没编出来 —— 改用系统 libgoogle-glog-dev 的同名 shim"
    apt-get install -y -qq --no-install-recommends libgoogle-glog-dev 2>&1 | tail -3
    # 源目录、build/、logs/ 三处都要清，否则 catkin 会沿用上次失败的缓存
    rm -rf "$WS_DEPS/src/glog_catkin" \
           "$WS_DEPS/build/glog_catkin" \
           "$WS_DEPS/logs/glog_catkin"
    mkdir -p "$WS_DEPS/src/glog_catkin_shim"
    cat > "$WS_DEPS/src/glog_catkin_shim/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="2">
  <name>glog_catkin</name>
  <version>0.3.5</version>
  <description>CI shim: 用系统 libgoogle-glog 冒充 ethz-asl/glog_catkin</description>
  <maintainer email="ci@example.com">ci</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
XML
    cat > "$WS_DEPS/src/glog_catkin_shim/CMakeLists.txt" <<'CMK'
cmake_minimum_required(VERSION 3.0.2)
project(glog_catkin)
find_package(catkin REQUIRED)
find_library(GLOG_LIBRARY NAMES glog REQUIRED)
catkin_package(INCLUDE_DIRS /usr/include LIBRARIES ${GLOG_LIBRARY})
CMK
    cd "$WS_DEPS"
    catkin build glog_catkin -j1 --no-status --force-cmake || true
    cd "$ROOT"
    if dep_ok glog_catkin; then
      say "glog shim 生效"
      missing=("${missing[@]/glog_catkin/}")
    fi
  fi

  # 剩下还缺的就是真问题（尤其 opencv3_catkin），如实报错
  local still=()
  for d in "${missing[@]:-}"; do
    [ -n "$d" ] && still+=("$d")
  done
  if [ "${#still[@]}" -gt 0 ]; then
    echo "✗ 依赖缺失：${still[*]}（catkin build 退出码 $rc_deps）" >&2
    return 1
  fi

  # catkin_tools 的 --extend 会验证目标是个 catkin 空间（找 .catkin 标记文件）；
  # 缺了它 s_build_main 会直接报 "Unable to find catkin config file"。
  touch "$WS_DEPS/install/.catkin"
  test -f "$WS_DEPS/install/setup.bash" || {
    echo "install/setup.bash 没生成 —— catkin --install 没生效" >&2
    return 1
  }

  du -sh "$WS_DEPS/install"
  ls "$WS_DEPS/install/lib" 2>/dev/null | grep -c '^libopencv' \
    | awk '{print "  libopencv*.so : "$1" 个"}' || true
  # SURF 能不能用，取决于这两个宏；编完立刻验一次
  grep -i "OPENCV_ENABLE_NONFREE" "$WS_DEPS/install/include/opencv2/opencv_modules.hpp" 2>/dev/null || true
  if ls "$WS_DEPS/install/include/opencv2/xfeatures2d" >/dev/null 2>&1; then
    say "xfeatures2d 头文件存在（SIFT/SURF 可用）"
  else
    warn "找不到 xfeatures2d 头文件 —— SURF 行会缺"
  fi

  # 给 workflow 用的「可缓存」标记：只有这一趟**真的从源码编出来、且四项都验过**
  # 才写。半成品（比如被 concurrency 取消的 run）没有这个文件，cache/save 那步
  # 就不会执行，避免再出现「命中的是 120 KB 半成品」那种事。
  : > "$OUT/.deps_fresh"
  say "已标记 deps 可缓存（out/.deps_fresh）"
  return 0
}

# =============================================================================
# S4 构建主工作区：cv_bridge(源码) + opengv + eigen_catkin + lab_5 + lab_6
# =============================================================================
try_clone() {
  local url="$1" dest="$2" branch="${3:-}"
  local i
  for i in 1 2 3; do
    if [ -n "$branch" ] && git clone --depth 1 --branch "$branch" "$url" "$dest" 2>/dev/null; then
      return 0
    fi
    if git clone --depth 1 "$url" "$dest" 2>/dev/null; then
      return 0
    fi
    warn "clone 失败（第 $i 次）：$url"
    rm -rf "$dest"
    sleep 5
  done
  return 1
}

s_build_main() {
  rm -rf "$WS_MAIN"
  mkdir -p "$WS_MAIN/src"

  cp -r "$BUNDLE/lab_5" "$WS_MAIN/src/lab_5"
  cp -r "$BUNDLE/lab6"  "$WS_MAIN/src/lab_6"

  # cv_bridge 必须跟着 OpenCV 3.4.2 一起从源码编（系统那份链接的是 3.2）
  rm -rf /tmp/vision_opencv
  try_clone https://github.com/ros-perception/vision_opencv.git /tmp/vision_opencv melodic \
    || try_clone https://github.com/ros-perception/vision_opencv.git /tmp/vision_opencv
  cp -r /tmp/vision_opencv/cv_bridge "$WS_MAIN/src/cv_bridge"

  # OpenGV（相对位姿的 5pt/8pt/2pt/Arun 全在这里面）+ 它依赖的 eigen_catkin
  try_clone https://github.com/ethz-asl/opengv.git        "$WS_MAIN/src/opengv"        master
  try_clone https://github.com/ethz-asl/eigen_catkin.git  "$WS_MAIN/src/eigen_catkin"  master

  cd "$WS_MAIN"
  catkin config --extend "$WS_DEPS/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  # 顺序有讲究：cv_bridge 要在 lab_6 之前，opengv 要在 lab_6 之前
  tmo 900 catkin build cv_bridge  -j"$(nproc)" --no-status || true
  tmo 900 catkin build eigen_catkin -j"$(nproc)" --no-status || true

  local rc_opengv=0
  tmo 1200 catkin build opengv -j"$(nproc)" --no-status --summarize || rc_opengv=$?
  local rc_build=0
  tmo 900 catkin build lab_5 lab_6 -j"$(nproc)" --no-status --summarize || rc_build=$?
  cd "$ROOT"

  say "---------- 编译产物 ----------"
  ls -la "$WS_MAIN/devel/lib/lab_6/" 2>/dev/null || true
  ls -la "$WS_MAIN/devel/lib/" | grep -i opengv || true

  {
    echo "# catkin build 证据"
    echo
    echo "## 编译环境"
    echo "- ROS: $(. /opt/ros/melodic/setup.bash && echo "$ROS_DISTRO")"
    echo "- gcc: $(gcc -dumpversion)"
    echo "- cmake: $(cmake --version | head -1)"
    # version.hpp 里 CV_VERSION 是宏拼接，grep 出来是宏定义本身。
    # 直接抽 MAJOR/MINOR/REVISION 三个数字拼成 x.y.z。
    cv_hdr="$WS_DEPS/install/include/opencv2/core/version.hpp"
    cv_ver=$(awk '/#define +CV_VERSION_(MAJOR|MINOR|REVISION) +[0-9]+/{printf "%s.", $3}' \
             "$cv_hdr" 2>/dev/null | sed 's/\.$//')
    echo "- OpenCV: ${cv_ver:-unknown} (ws_deps，带 nonfree)"
    echo
    echo "## opengv 状态"
    [ "$rc_opengv" -eq 0 ] && echo "- opengv: 编译成功" || echo "- opengv: 编译失败 (rc=$rc_opengv)"
    ls -d "$WS_MAIN/devel/lib/libopengv"* 2>/dev/null || true
    echo
    echo "## lab_6 产物"
    ls -la "$WS_MAIN/devel/lib/lab_6/" 2>/dev/null || true
    echo
    echo "## 链接的 OpenCV 版本（ldd）"
    if [ -x "$WS_MAIN/devel/lib/lab_6/pose_estimation" ]; then
      ldd "$WS_MAIN/devel/lib/lab_6/pose_estimation" | grep -E 'opencv_core|opengv' || true
    fi
  } > "$OUT/build_evidence.txt" 2>&1
  cat "$OUT/build_evidence.txt"

  if [ ! -x "$WS_MAIN/devel/lib/lab_6/pose_estimation" ]; then
    warn "pose_estimation 没编出来（rc=$rc_build）"
    return 1
  fi
  return 0
}

# =============================================================================
# S5 准备 TUM 序列（office.bag 的替代数据集）
# =============================================================================
DATA="$ROOT/data/rgbd_dataset_freiburg1_xyz"
TUM_URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz"

s_data() {
  if [ -f "$DATA/rgb.txt" ] && [ -f "$DATA/groundtruth.txt" ]; then
    say "TUM 序列已存在：$DATA"
    echo "frames = $(grep -vc '^#' "$DATA/rgb.txt")"
    return 0
  fi
  mkdir -p "$ROOT/data"
  for i in 1 2 3; do
    # 460 MB，450 s 够用；三次重试最多 22 分钟，不再是 45 分钟
    if curl -fsSL --retry 2 --max-time 450 -o "$ROOT/data/tum.tgz" "$TUM_URL"; then
      break
    fi
    warn "下载失败（第 $i 次），重试"
    sleep 5
  done
  tar xzf "$ROOT/data/tum.tgz" -C "$ROOT/data"
  rm -f "$ROOT/data/tum.tgz"
  [ -f "$DATA/rgb.txt" ] || { warn "解压后没有 rgb.txt"; return 1; }
  echo "frames = $(grep -vc '^#' "$DATA/rgb.txt")"
}


# =============================================================================
# S5a 冒烟 + 崩溃诊断
# =============================================================================
# 用 gdb 跑 5 帧。目的不是出数据，是万一节点在启动阶段崩掉时，
# 能拿到一份带函数名/行号的栈（glog 的 handler 在缺符号时只打 (unknown)）。
s_smoke() {
  need_bin "$WS_MAIN/devel/lib/lab_6/pose_estimation" || return 1
  if ! command -v gdb >/dev/null 2>&1; then
    apt-get install -y gdb >/dev/null 2>&1 || warn "gdb 装不上，冒烟会退化成直接运行"
  fi
  start_master || return 1
  set_camera_params
  # shellcheck disable=SC1090
  source "$WS_MAIN/devel/setup.bash"

  local log="$OUT/smoke.log"
  : > "$log"
  local bin="$WS_MAIN/devel/lib/lab_6/pose_estimation"
  local runner
  if command -v gdb >/dev/null 2>&1; then
    runner=(gdb -batch -ex "set pagination off" -ex "run"
            -ex "echo \n===== BACKTRACE =====\n" -ex "thread apply all bt 30"
            --args "$bin")
  else
    runner=("$bin")
  fi

  "${runner[@]}" \
      __name:=lab_6 \
      _pose_estimator:=0 _use_ransac:=true _max_frames:=5 \
      _csv_path:="$OUT/smoke.csv" \
      /ground_truth_pose:=/tesse/odom \
      /rgb_images_topic:=/tesse/left_cam/rgb/image_raw \
      /depth_images_topic:=/tesse/depth_cam/mono/image_raw >> "$log" 2>&1 &
  local gpid=$!
  sleep 3

  # 播 12 帧就够触发启动路径。三个上限缺一不可：等订阅者 25 s（节点崩了就没有
  # 订阅者）、外部 timeout 180 s、节点用 wait_node 兜底。否则这里会永久挂住。
  local play_rc=0
  tmo 180 python "$ROOT/ci/tum_play.py" --data "$DATA" --rate 10 \
      --max-frames 12 --wait-subscribers 25 >> "$log" 2>&1 || play_rc=$?

  # 关键诊断：节点还在不在。之前 47 分钟的挂起就是因为没人问这一句。
  if kill -0 "$gpid" 2>/dev/null; then
    say "冒烟：节点仍在运行（tum_play rc=$play_rc）"
  else
    warn "冒烟：节点已经退出（tum_play rc=$play_rc）—— 不再等待，直接取栈"
  fi
  # 没有订阅者 / tum_play 超时的时候，把 ROS 图的状态也留下来
  if [ "$play_rc" -ne 0 ]; then
    {
      echo "---------- rosnode list ----------"
      rosnode list 2>&1 || true
      echo "---------- rosnode info /lab_6 ----------"
      rosnode info /lab_6 2>&1 | head -40 || true
      echo "---------- rostopic info ----------"
      rostopic info /tesse/left_cam/rgb/image_raw 2>&1 | head -20 || true
    } >> "$log" 2>&1
  fi

  wait_node "$gpid" 120 || true
  stop_master

  echo "---------- smoke.log 关键行 ----------"
  grep -nE "SIGABRT|SIGSEGV|munmap|BACKTRACE|^#[0-9]+ |Aborted|double free" "$log" | head -45 || true
  if grep -qE "SIGABRT|SIGSEGV|double free|munmap_chunk" "$log"; then
    warn "冒烟阶段节点崩溃 —— 完整栈见 out/smoke.log"
    return 1
  fi
  # 没有订阅者 / 播放超时：节点活着但没有 echo 到消息，同样算失败
  if [ "$play_rc" -ne 0 ]; then
    warn "冒烟阶段 tum_play 异常退出（rc=$play_rc）—— 节点没在收数据，见 $log"
    tail -40 "$log" || true
    return 1
  fi
  # 冒烟还要证明"真的算出了位姿"：只要有一行 RPE 就说明
  # 跟踪 → 去畸变 → RANSAC → RPE → 写 CSV 整条链通了。
  # 只看"没崩"是不够的：Run #7 里节点安静地 rc=1 退出，冒烟却判了 PASS。
  local srows=0
  [ -f "$OUT/smoke.csv" ] && srows=$(($(wc -l < "$OUT/smoke.csv") - 1))
  echo "smoke csv rows = $srows"
  if [ "$srows" -lt 1 ]; then
    warn "冒烟阶段没有写出任何 RPE 行 —— 节点大概启动后就退了，见 $log"
    tail -40 "$log" || true
    return 1
  fi
  echo "冒烟通过：节点未崩溃，且已写出 $srows 行 RPE"
}

# =============================================================================
# S6 跑四个估计器
# =============================================================================
# 节点读的私有参数（内参/畸变/深度），与 video_tracking_tum.launch 保持一致
set_camera_params() {
  rosparam set /lab_6/fx 517.306408
  rosparam set /lab_6/fy 516.469215
  rosparam set /lab_6/cx 318.643040
  rosparam set /lab_6/cy 255.313989
  rosparam set /lab_6/dist "[0.262383, -0.953104, -0.005358, 0.002628, 1.163314]"
  rosparam set /lab_6/depth_scale 5000.0
  rosparam set /lab_6/min_depth 0.30
  rosparam set /lab_6/max_depth 8.0
  # 关键：lab_5 的追踪器在 trackFeatures() 里会 showImage()，而 showImage 读
  # `~interactive`（默认 true）。CI 没有 X display，namedWindow 一进去就是
  # Gtk-WARNING "cannot open display"，紧接着整个节点以 rc=1 退出 —— 症状是
  # 目录里只有 sift_keypoints_img1/2.png、CSV 一行都没有（Run #7 就是这样）。
  # lab5 的 launch 里也是这么关的，这里必须同样关掉。
  rosparam set /lab_6/interactive false
  rosparam set /lab_6/output_dir "$OUT/fig"
}

s_run() {
  local est="$1" rate="${2:-8}"
  need_bin "$WS_MAIN/devel/lib/lab_6/pose_estimation" || return 1
  start_master || return 1
  set_camera_params

  local csv="$OUT/rpe_${est}.csv"
  local log="$OUT/run_${est}.log"
  : > "$log"
  # 正式跑不存 PNG：798 帧 × 2 张会让每一步多花几十秒，而且产物不看这些图
  rosparam set /lab_6/output_dir ""
  # shellcheck disable=SC1090
  source "$WS_MAIN/devel/setup.bash"

  # 参数走 ROS 私有参数（_name:=value），不再用 gflags：
  # melodic 下 gflags 的全局 std::string 与 roscpp 的 std::string 跨库传递会踩堆。
  rosrun lab_6 pose_estimation \
      __name:=lab_6 \
      _pose_estimator:="$est" \
      _use_ransac:=true \
      _scale_translation:=true \
      _csv_path:="$csv" \
      _ransac_threshold_bearing:=1e-4 \
      _ransac_threshold_3d:=0.03 \
      _ransac_max_iterations:=2000 \
      /ground_truth_pose:=/tesse/odom \
      /rgb_images_topic:=/tesse/left_cam/rgb/image_raw \
      /depth_images_topic:=/tesse/depth_cam/mono/image_raw \
      >> "$log" 2>&1 &
  local node_pid=$!
  sleep 3

  # 播放上限：帧数/频率 × 1.6 + 180 s。tum_play 自己到这个点就停，
  # wait_node 只当外层兜底 —— 之前这里写死 1200 s，"不动"和"正常播完"
  # 完全分不出来，一个估计器白等 20 分钟。
  local nframes play_lim
  nframes=$(grep -vc '^#' "$DATA/rgb.txt" 2>/dev/null || echo 0)
  play_lim=$(awk -v n="$nframes" -v r="$rate" 'BEGIN{printf "%d", n/r*1.6+180}')
  say "估计器 $est：$nframes 帧 @ ${rate} Hz，播放上限 ${play_lim}s"

  local py
  py="$(command -v python || command -v python3)"
  "$py" "$ROOT/ci/tum_play.py" --data "$DATA" --rate "$rate" \
      --max-seconds "$play_lim" >> "$log" 2>&1 &
  local play_pid=$!
  wait_node "$play_pid" "$((play_lim + 120))" || true

  # 播完之后给节点 5 s 把最后几帧处理完；它没有"bag 结束"信号，所以要主动收
  sleep 5
  if ! kill -0 "$node_pid" 2>/dev/null; then
    warn "节点在播放过程中退出了（估计器 $est）—— 见 $log"
  fi
  if ! wait_node "$node_pid" 15; then
    warn "节点没自己退出（估计器 $est），已 SIGINT"
  fi
  stop_master

  local rows=0
  [ -f "$csv" ] && rows=$(($(wc -l < "$csv") - 1))
  echo "estimator=$est  csv rows=$rows"
  grep -E '^\[RPE\] frame' "$log" | tail -3 || true
  if [ "$rows" -gt 50 ]; then
    return 0
  fi
  warn "估计器 $est 的 CSV 只有 $rows 行 —— 见 $log"
  tail -40 "$log" || true
  return 1
}

s_run_5pt()  { s_run 0 8; }
s_run_8pt()  { s_run 1 8; }
s_run_2pt()  { s_run 2 8; }
s_run_arun() { s_run 3 8; }

# =============================================================================
# S7 汇总
# =============================================================================
s_summary() {
  {
    echo "# LAB6 CI 结果"
    echo
    echo "## 各估计器的 CSV 行数"
    for f in "$OUT"/rpe_*.csv; do
      [ -f "$f" ] || continue
      echo "- $(basename "$f"): $(($(wc -l < "$f") - 1)) 帧"
    done
    echo
    echo "## 编译证据"
    echo '```'
    sed -n '1,40p' "$OUT/build_evidence.txt" 2>/dev/null || true
    echo '```'
    echo
    echo "## stage 结果"
    echo "- PASS: ${PASS[*]:-}"
    echo "- FAIL: ${FAIL[*]:-}"
  } > "$OUT/report.md"
  cat "$OUT/report.md"
}

main() {
  stage env        s_env
  stage apt        s_apt
  stage build_deps s_build_deps
  stage build_main s_build_main
  stage data       s_data
  stage smoke      s_smoke
  stage run_5pt    s_run_5pt
  stage run_8pt    s_run_8pt
  stage run_2pt    s_run_2pt
  stage run_arun   s_run_arun
  stage summary    s_summary

  hr
  if [ "${#FAIL[@]}" -eq 0 ]; then
    say "全部 stage 通过：${PASS[*]}"
    return 0
  fi
  warn "有 stage 失败：${FAIL[*]}"
  return 1
}

main
exit $?
