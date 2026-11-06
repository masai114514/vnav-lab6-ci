# vnav-lab5-ci —— LAB5 特征跟踪链路的 CI 载体

MIT 16.485 VNAV **Lab 5** 的团队交付物（D3–D8）全部是「跑出来的数字 + 图」：

- **D3/D5**：SIFT / SURF / ORB / FAST+BRIEF 在 `box.png` + `box_in_scene.png` 上的
  平均 keypoints / 匹配数 / 良好匹配数 / 内点数 / 内点率（官方 handout 里有一张参考表）
- **D4**：keypoints 图 + 全部匹配图 + 良好匹配图 + **内点绿/外点红**的叠加图
- **D6/D7**：视频序列上的累计统计 + Harris-LK 的跟踪轨迹图
- **D8**（可选 +10）：Farneback 稠密光流的彩色图

这些必须在 **ROS1 + OpenCV 3.4.2（含 nonfree）** 里跑。而 SIFT/SURF 在 OpenCV 3.4.2
里由 `OPENCV_ENABLE_NONFREE` 控制，**pip 轮子一律关着**——只能在源码编译时打开。
这台机器是 macOS，装不了这套 ROS1 环境；队友是 Windows，装 WSL2 + 编译 OpenCV
也要 40 分钟起，而且三个人各装一遍纯属浪费。

所以搬到 GitHub Actions：runner 是原生 x86_64，容器给原版 bionic + ROS melodic，
**三个人共用一份编译好的环境**，谁都不用在自己机器上装 ROS。

## 怎么用

```bash
# 0) 一次性：建仓（public → 4 vCPU + Actions 分钟数不计数；private → 2 vCPU + 2000 分钟/月）
PY=/Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python
$PY push_via_api.py --create public

# 1) 改完 lab5 里的代码后，先把源码同步进本仓的 bundle/
bash sync.sh

# 2) 推上去（本机 git push 走代理必挂，所以走 gh 的 Git Data API）
$PY push_via_api.py --message "feat: SIFT 描述子 + FLANN 匹配"

# 3) 触发
gh workflow run lab5-ci.yml --repo masai114514/vnav-lab5-ci

# 4) 等完取产物（约 6–10 分钟；首次要编 OpenCV，约 30–45 分钟）
bash fetch_artifacts.sh              # 最近一次
bash fetch_artifacts.sh <run_id>     # 指定一次
```

跑完看两处：Actions 页面那次运行的 **Summary**（自动贴 `report.md` 前 120 行），
和 artifact `lab5-results` 里的 `report.md` + `fig/*.png`。

## 目录

| 路径 | 是什么 |
|---|---|
| `.github/workflows/lab5-ci.yml` | 工作流。容器 `ros:melodic-ros-base`，`workflow_dispatch` 可分别指定图像对/视频要跑哪些描述子 |
| `ci/lab5_ci.sh` | ★ 主脚本：修 apt key → 装依赖 → 编 OpenCV → 编 cv_bridge/lab_5 → 图像对 ×4 → 视频 ×3 → 光流 → 汇总 |
| `ci/summarize.py` | 把各 stage 的日志解析成 `out/report.md`（两张表 + 图清单 + 构建证据），数字与日志一一对应，不手抄 |
| `ci/ci_tools/` | `mp4_to_bag`：把 mp4 打成与 TESSE 同话题的 rosbag（官方数据集已 404，见下） |
| `data/seq20s.mp4` | 20 s / 320×200 / 6 fps 测试序列（576 KB，从 LAB4 竞速录像裁的） |
| `sync.sh` | 把 `../`（lab5/）的 `feature_tracking/` + `dependencies/` rsync 进 `bundle/` |
| `push_via_api.py` | 绕过挂掉的 `git push`，走 GitHub Git Data API 推仓 |
| `fetch_artifacts.sh` | 拉回 artifact 并打印两张表 + 图清单 |
| `bundle/` | **自动生成的源码镜像，不要手改**（下次 sync 就覆盖了） |

## 六个容易踩回去的坑（都已在脚本/工作流里注释）

0. **不能用 job 级 `container:`。** lab3/lab4 那种写法在这里必挂：GitHub 从 2025-09 起
   把 JS action 的 runtime 升到 Node 24（要求 glibc ≥ 2.28），而 `ros:melodic` 是
   Ubuntu 18.04（glibc **2.27**），`actions/checkout` 一上来就
   `version 'GLIBC_2.28' not found`，35 秒整个 run 失败（实测 Run #1）。
   **解法：action 全在 host（ubuntu-22.04）上跑，只在干活那一步 `docker run` 进 melodic。**
   挂载必须用同一个绝对路径 `-v "$PWD":"$PWD"`，否则 catkin 的 `setup.bash` 和
   缓存的 `ws_deps/install` 对不上。
1. **ROS1 的 apt key 2025-06-01 过期。** `apt-get update` 直接 GPG error，后面一步都跑不了。
   脚本先试一次，失败就重装 `ros.key` 并重写 sources.list，再失败就把 Ubuntu 源切到
   `old-releases.ubuntu.com`。
2. **`opencv3_catkin` 默认没开 `OPENCV_ENABLE_NONFREE`。** OpenCV 3.4.2 里
   `SURF::create()` 会抛 `This algorithm is patented and is excluded`。
   `bundle/dependencies/opencv3_catkin/CMakeLists.txt` 里已打 `-DOPENCV_ENABLE_NONFREE=ON`；
   这个文件的 hash 是缓存 key 的一部分，改了会自动重建 OpenCV。
3. **系统的 `cv_bridge` 是链接 OpenCV 3.2 编出来的。** 让 lab_5 用 3.4.2 而 cv_bridge 用
   3.2，等于同一进程里两份 `cv::Mat`（ODR 违规）。ethz-asl 的 opencv3_catkin README
   明确要求重编下游 → 脚本从 `vision_opencv` 的 melodic 分支重编 cv_bridge。
4. **官方 6b 数据集已从 GitHub 消失。** `MIT-SPARK/VNAV-lab-data` 在 2022-10-14 移除 LFS 后，
   `30fps_424x240_...bag` 与 `vnav-lab5-smooth-trajectory.bag` 全是 135 字节指针、实际对象 404
   （已验证）。所以用 `ci_tools/mp4_to_bag` 造同话题 bag，**官方 launch 一行不改**。
   ⚠️ 但正式交付的数字仍建议用课程组发的真数据集；这个只是让 CI 能跑通链路。
5. **`bash x.sh | tee log` 的退出码来自 tee。** 不在 workflow 里加 `set -o pipefail`，
   脚本失败也会显示 success、留下空产物（lab3 踩过）。
   容器那步还要额外 `set +e` 收 `$?`，否则 GH 的 `-e` 会在 docker 非零退出时直接中断，
   后面的 `chown` 和退出码都没了。

## 耗时与缓存

| 场景 | 耗时 | 说明 |
|---|---|---|
| 首次 / 缓存失效 | 30–45 min | OpenCV 3.4.2 + contrib 从源码编，2 vCPU 约 20–25 min |
| 缓存命中 | 6–10 min | 只编 lab_5 + cv_bridge（几十秒），剩下是跑测试 |

缓存的是 `ws_deps/install`（OpenCV 的 install 空间，约 200 MB）。脚本靠
`ws_deps/install/setup.bash` 是否存在判断命中，命中就整段跳过编译。
改了 `opencv3_catkin/CMakeLists.txt` → cache key 变 → 自动重建。

## 为什么 MI 的 handout 说"有些算法会在这两张图上很差"

`box.png` 是 324×223 的小图，`box_in_scene.png` 是 512×384 的场景图，视角和尺度都变了。
官方参考解在 SIFT 上得到 603 / 969 / 93 / 78（83.9 % 内点率），而 ORB 在这对图上只有 7 个
良好匹配、0 个内点——这不是你写错了，是 ORB 的 32 字节二进制描述子 + 500 个特征上限
在这对图上确实不行。报告里要把这个差异讲成结论，不是当成 bug。
