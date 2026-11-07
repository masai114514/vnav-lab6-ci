# vnav-lab6-ci —— LAB6 相对位姿估计的 CI 载体

MIT 16.485 VNAV **Lab 6**：在序列上跑四个相对位姿估计器，每个都要逐帧的旋转/平移误差。

- **5 点法（Nistér）**、**8 点法**、**2 点法（translation-only）** → 2D-2D
- **Arun / Umeyama 3D-3D** → 点云配准

这些都必须在 **ROS1（melodic）+ OpenCV 3.4.2 + OpenGV** 里跑：`lab_6` 通过
`tracker_shim` 复用上一实验的 `lab_5` 特征跟踪包，而 `lab_5` 的 SIFT/SURF 需要
`OPENCV_ENABLE_NONFREE`（pip 轮子一律关着）。本机是 macOS，装不了 ROS1；队友是
Windows，装 WSL2 + 从源码编 OpenCV 也要 40 分钟起，三个人各装一遍纯属浪费。

所以和 lab3/lab4/lab5 一样搬到 GitHub Actions：runner 是原生 x86_64，容器给原版
bionic + ROS melodic，**三个人共用一份编好的环境**。官方 `office.bag` 已不可下载，
数据集换成 **TUM `rgbd_dataset_freiburg1_xyz`**（RGB + 真值轨迹，460 MB，CI 里现下）。

## 怎么用

```bash
# 1) 改完 lab6/ 里的代码后，先把源码同步进本仓的 bundle/
bash sync.sh

# 2) 推上去（本机 git push 走代理必挂，所以走 gh 的 Git Data API）
python3 push_via_api.py --repo masai114514/vnav-lab6-ci --message "fix: ..."
#    CI 编译的就是 bundle/ 的那份源码；忘了 sync 就是编旧代码

# 3) 触发
gh workflow run lab6-ci.yml --repo masai114514/vnav-lab6-ci
#    推 bundle/** 或 ci/** 的改动也会自动触发；改 README/本地脚本不会

# 4) 等完取产物（缓存命中约 15–30 分钟；首次要编 OpenCV，40 分钟起）
bash fetch_artifacts.sh                 # 最近一次
bash fetch_artifacts.sh 42231340010     # 指定一次
```

跑完看两处：Actions 页面那次运行的 **Summary**（自动贴 `out/report.md` 前 80 行），
和 artifact `lab6-results` 里的 `rpe_*.csv` + `run_*.log`（报告 §5 的图和数字就是从这来的）。

## 目录

| 路径 | 是什么 |
|---|---|
| `.github/workflows/lab6-ci.yml` | 工作流。`workflow_dispatch` 可手动跑；push 按 `paths` 过滤 `bundle/**`、`ci/**`、`data/**` |
| `ci/lab6_ci.sh` | ★ 主脚本：13 个 stage，每个独立计时/超时，最后汇总 PASS/FAIL 并写 `out/report.md` |
| `ci/tum_play.py` | TUM 序列播放节点。`--lockstep` 发一帧等一帧 `/camera_pose`（防丢帧）、`--max-seconds` 兜底、等订阅者带超时 |
| `bundle/lab6/` | ROS 包 `lab_6`：`src/pose_estimation.cpp`（四个估计器）+ `tracker_shim` 适配 `lab_5` |
| `bundle/lab_5/` | 上一实验的特征跟踪包，`lab_6` 依赖它取光流/匹配 |
| `bundle/dependencies/` | `opencv3_catkin`（已打 `OPENCV_ENABLE_NONFREE=ON` 补丁）+ `gflags_catkin` |
| `data/` | **不进仓**。CI 里现下 TUM fr1_xyz 解到这里，靠 cache `tum-fr1-xyz-v1` 免重下 |
| `sync.sh` | 把 `../`（= `lab6/`）里 CI 需要的那部分 rsync 进 `bundle/` |
| `push_via_api.py` | 绕过挂掉的 `git push`，走 GitHub Git Data API 推仓 |
| `fetch_artifacts.sh` | 拉回 artifact，落地到 `lab6/ci_out/lab6_run<id>/` 并列清单 |

`bundle/` 是**自动生成的镜像，不要手改**——下次 `sync.sh` 就覆盖了。源码正本在 `../`。

## 13 个 stage

| # | stage | 干什么 |
|---|---|---|
| 1 | `env` | 打印编译器 / OpenCV / OpenGV / 内存 / 磁盘，留证据 |
| 2 | `apt` | 修 apt key（ROS1 的 key 2025-06-01 已过期）→ 装 melodic 依赖 |
| 3 | `build_deps` | 编 `opencv3_catkin`（nonfree）+ `gflags_catkin` → `ws_deps/install` |
| 4 | `build_main` | 编 `lab_5` / `lab_6`（`catkin build`）→ `ws_main/devel` |
| 5 | `data` | 下 TUM fr1_xyz（三次重试，每次 450 s 上限） |
| 6 | `smoke` | 用 gdb 跑 5 帧，确认节点不崩；崩了抓符号化栈 |
| 7–10 | `run_5pt` `run_8pt` `run_2pt` `run_arun` | 四个估计器各跑一遍（`--lockstep`，797 帧），写 `rpe_{0,1,2,3}.csv` |
| 11–12 | `run_5pt_raw` `run_2pt_raw` | 对照组：`_refine:=false`，用来分离「非线性精化」的影响 |
| 13 | `summary` | 汇总帧数 / 中位误差 / 编译证据 → `out/report.md` |

## 最后一次全绿

**run `42231340010`**（13/13 PASS，31 分 11 秒）。报告里的 CI 数字全部来自这一趟：
5pt 1.927° / 1.168、8pt 1.108° / 1.006、2pt 0.000° / 0.392、Arun 0.280° / 4.4 mm。

## 九个踩回去过的坑（都在脚本/工作流里留了注释）

0. **不能用 job 级 `container:`。** GitHub 从 2025-09 起把 JS action 的 runtime 升到
   Node 24（要求 glibc ≥ 2.28），而 `ros:melodic` 是 18.04（glibc **2.27**），
   `actions/checkout` 一上来就 `version 'GLIBC_2.28' not found`。
   **解法：action 全在 host（ubuntu-22.04）上跑，只在干活那一步 `docker run` 进容器。**
   挂载用同一个绝对路径 `-v "$PWD":"$PWD"`，否则 catkin 的 `setup.bash` 和缓存的
   `ws_deps/install` 对不上。
1. **ROS1 的 apt key 过期** → `apt-get update` 直接 GPG error。先试一次，失败就重装
   `ros.key` 并重写 sources.list，再失败把 Ubuntu 源切 `old-releases.ubuntu.com`。
2. **`opencv3_catkin` 默认没开 nonfree。** OpenCV 3.4.2 里 `SIFT::create()` 会抛
   `This algorithm is patented and is excluded`。CMakeLists 的 hash 是缓存 key 的一部分，
   改了会自动重建 OpenCV。
3. **系统 `cv_bridge` 链的是 OpenCV 3.2**，和 `lab_5` 用的 3.4.2 混在一个进程里就是
   ODR 违规（两份 `cv::Mat`）。所以从 `vision_opencv` 的 melodic 分支重编 cv_bridge。
4. **`bash x.sh | tee log` 的退出码来自 tee。** 不加 `set -o pipefail`，脚本失败也显示
   success、留下空产物。容器那步还要 `set +e` 收 `$?`，否则 GH 的 `-e` 会在 docker
   非零退出时直接中断，后面的 `chown` 和退出码都丢了。
5. **headless 下 `showImage()` 把节点打死。** `lab_5` 的 tracker 读私有参数
   `interactive`（默认 true），没有 display 时 `namedWindow` 直接抛异常，节点 rc=1
   静默退出、CSV 是空的。CI 里先 `rosparam set /lab_6/interactive false`。
6. **等订阅者写成了死循环。** Run #6 挂了 47 分钟才发现：节点早崩了，播放器还在等
   `/camera_pose`。现在 `--wait-subscribers` 有超时、播放有 `--max-seconds` 兜底，
   并用 `kill -0` 判节点活不活。
7. **rate 模式会丢帧。** 各估计器只收到 315/312/316/393 帧（真值 798）。改成
   `--lockstep`（发一帧、等 `/camera_pose` 回来再发下一帧）后四个估计器都是 797 帧。
8. **两个内存/语义 bug。**
   ① `cv::Mat::zeros(cv::Size(5,1), CV_64F)` 是 **1×5** 不是 5×1，`at<double>(i,0)`
   （i≥1）越界写堆，崩在毫不相干的 `hasParam` 里，误导了六轮排查；改 `at<double>(0,i)`。
   ② `TranslationOnlySacProblem::optimizeModelCoefficients` 会调 `optimize_nonlinear`
   优化完整 6 自由度，把 `setR12` 给的真值 R 改掉（实测偏 0.435°）。修法：精化后把
   `model_coefficients_.block<3,3>(0,0)` 写回真值 R，797 帧的旋转误差才恒为 0。

## 耗时与缓存

| 场景 | 实测 | 说明 |
|---|---|---|
| 依赖缓存 + 数据集缓存都命中 | 15–31 min | 只跑 5 个估计器 × 797 帧，波动主要来自 runner 负载 |
| 缓存失效 | 40 min 起 | OpenCV 3.4.2 + contrib 从源码编，2 vCPU 约 20–25 min；数据集 2–4 min |

缓存两项：`ws_deps/install`（OpenCV 的 install 空间）与 `data/rgbd_dataset_freiburg1_xyz`。
artifact `lab6-results` 保留 14 天；本地 `lab6/ci_out/` 存着最后一趟的 CSV 与 `report.md`。
