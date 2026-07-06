# TianqueROS-Improved

> 模块化自主探索框架 — Autonomous Exploration Framework

## 架构

```
Mission → Exploration → Navigation → Adapter → ROS Navigation Stack
                                              └→ 自研算法 (双轨)
```

五层架构，每层职责单一。算法是插件，不是核心。

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 快速开始

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 测试
catkin_make run_tests    # 72 tests

# Demo（需要图形桌面）
roslaunch warehouse_utils demo_gazebo.launch
```

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
