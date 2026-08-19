# TianqueROS-Improved

> 模块化自主探索框架 — Autonomous Exploration Framework

## 架构

```
Mission → Exploration → Navigation → Adapter → ROS Navigation Stack
                                              └→ 自研算法 (双轨)
```

五层架构，每层职责单一。算法是插件，不是核心。

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 环境重建（全新机器）

本仓库包含全部自研源码 + 第三方依赖源码（src/ 与 third_party/），
按以下步骤可在全新 Ubuntu 20.04 机器上重建本机开发环境。

### 0. 环境要求

- Ubuntu 20.04 (Focal) x86_64
- ROS Noetic
- 内存 ≥ 8GB（建议 16GB），磁盘 ≥ 20GB（不含数据集）
- 图形桌面（Gazebo + rviz 目视验证需要）

### 1. 安装 ROS Noetic

按官方步骤安装 `ros-noetic-desktop-full`（含 gazebo、rviz、tf2 等）。

### 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-navigation ros-noetic-costmap-2d ros-noetic-dwa-local-planner \
  ros-noetic-hector-slam ros-noetic-hector-mapping \
  ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  libceres-dev libeigen3-dev liblua5.3-dev libgflags-dev libgoogle-glog-dev \
  libsuitesparse-dev libboost-all-dev libprotobuf-dev protobuf-compiler \
  ninja-build python3-catkin-tools
```

### 3. 获取源码（整个工作空间就是一个 git 仓库）

```bash
cd ~
git clone -b refactor-v2 https://github.com/ligjlj/tianqueros-improved.git catkin_ws
```

### 4. 构建 cartographer 核心（third_party，安装到 /usr/local）

```bash
# 4.1 abseil-cpp
cd ~/catkin_ws/third_party/abseil-cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j$(nproc) && sudo make install

# 4.2 cartographer（核心算法库，供 cartographer_ros 链接）
cd ~/catkin_ws/third_party/cartographer
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DCMAKE_PREFIX_PATH=/usr/local
make -j$(nproc) && sudo make install
```

### 5. 构建 catkin 工作空间

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 只构建框架包（跳过第三方，构建更快）：
> ```bash
> catkin_make -DCATKIN_WHITELIST_PACKAGES="warehouse_utils;warehouse_interfaces;warehouse_motion;warehouse_mission;warehouse_planner;warehouse_controller;warehouse_costmap;warehouse_navigation;warehouse_exploration;warehouse_recovery;simple_drone"
> ```

### 6. 测试

```bash
catkin_make run_tests   # Level 1 单元测试（gtest）
```

### 7. Demo 验证

```bash
roslaunch warehouse_utils demo_gazebo.launch
```

验证（14 个节点）:

```bash
rosnode list | wc -l          # 14
rostopic info /cmd_vel        # 唯一发布者 motion_controller
rostopic info /nav_status
rostopic info /coverage
```

### 8. 可选：VINS-Fusion 数据集

`data/`（EuRoC V1_01_easy 等，约 4GB）不入 git。需要复现 VINS 实验时
自行下载到 `~/catkin_ws/data/vicon_room1/`（EuRoC MAV 数据集官网）。

## 五层

| 层 | 职责 | 不知道 |
|---|---|---|
| Mission | 要不要探索 | 怎么走 |
| Exploration | 去哪 | A*/DWA |
| Navigation | 怎么导航 | Frontier |
| Backend | 执行规划 | 任务 |
| Localization | 在哪 | 谁用 |

## 技术栈

| 模块 | 方案 | 可替换 |
|---|---|---|
| Mission | 自研 MissionManager | — |
| Exploration | 自研 Frontier + GoalSelector | NBV, Active SLAM |
| Navigation | MoveBaseAdapter → move_base | Nav2, 自研 |
| Planner | navfn (ROS) | Hybrid A*, 自研 A* |
| Controller | DWA (ROS) | TEB, MPC, 自研 |
| Costmap | costmap_2d (ROS) | — |
| Localization | Cartographer / VINS / GT | FAST-LIO |
| Motion | 自研 MotionController | — |

## 仓库结构

```
catkin_ws/                  # 整个目录即 git 仓库
├── src/                    # catkin 包（catkin_make 构建）
│   ├── warehouse_*         # 自研框架包（12 个）
│   ├── simple_drone        # 机器人 URDF / 模型
│   ├── simple_planner      # 早期简单规划器
│   ├── cartographer_ros    # 第三方: Cartographer ROS 封装
│   ├── VINS-Fusion-RGBD    # 第三方: VINS-Fusion（RGBD 版）
│   ├── waterplus_map_tools # 第三方: 6-robot 地图工具
│   ├── wpb_home            # 第三方: 水伴机器人主包
│   └── wpr_simulation      # 第三方: 水伴机器人 Gazebo 仿真
├── third_party/            # 非 catkin 源码（cmake 单独构建）
│   ├── cartographer        # Cartographer 核心算法库
│   └── abseil-cpp          # Google Abseil（cartographer 依赖）
├── docs/                   # 架构文档（全中文）
├── tests/                  # 端到端脚本
└── README.md
```

## 第三方依赖来源

| 目录 | 上游 | 版本/commit |
|---|---|---|
| cartographer_ros | github.com/cartographer-project/cartographer_ros | c138034 (Remove Kinetic and fix CI #1746) |
| cartographer | github.com/cartographer-project/cartographer | 与官方 build 流程一致的源码快照 |
| abseil-cpp | github.com/abseil/abseil-cpp | 源码快照 |
| VINS-Fusion-RGBD | github.com/HKUST-Aerial-Robotics/VINS-Fusion | RGBD 版 + support_files |
| waterplus_map_tools | github.com/6-robot/waterplus_map_tools | a38e5b2 (update for wpr2) |
| wpb_home | github.com/6-robot/wpb_home | 0917443 |
| wpr_simulation | github.com/6-robot/wpr_simulation | 源码快照 |

## 常见问题

- VMware 虚拟显卡: rviz 段错误 → `export MESA_GL_VERSION_OVERRIDE=3.3`；
  Gazebo 黑屏 → `export LIBGL_ALWAYS_SOFTWARE=1`
- `spawn_model` 用 `-unpause` 不用 `-wait`（maze.world 加载慢时会无限阻塞）
- 磁盘清理: `rm -rf build devel` 后重新 `catkin_make`
- hector + sim_time 下 move_base 报 TF Extrapolation Error 属已知现象
  （hector 发布 map→odom 频率低于需求），不影响规划；可放宽
  costmap_common.yaml 的 transform_tolerance 到 0.5~1.0

## 文档

| 文档 | 用途 |
|---|---|
| [ROADMAP.md](docs/ROADMAP.md) | 唯一开发入口 |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 五层架构 |
| [INTERFACES.md](docs/INTERFACES.md) | Topic/事件规范 |
| [ADR.md](docs/ADR.md) | 架构决策 |
| [AGENTS.md](docs/AGENTS.md) | Multi-Agent 协作 |
| [CHANGELOG.md](docs/CHANGELOG.md) | 版本记录 |
| [REGRESSION.md](docs/REGRESSION.md) | 回归检查 |
