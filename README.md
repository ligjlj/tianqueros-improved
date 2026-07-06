# TianqueROS-Improved

> 模块化仓库自主探索框架 — MissionManager + Exploration + ROS Navigation Stack

## 架构

```
MissionManager → ExplorationManager → NavigationManager → MoveBaseAdapter → move_base
                                                              │
                                                        MotionController → cmd_vel
```

分层事件驱动。上层自研（Mission/Exploration/Motion），算法层使用 ROS Navigation Stack（navfn + DWA + costmap_2d），通过 Adapter 解耦。自研 A*/DWA 保留双轨。

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 快速开始

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 跑测试
catkin_make run_tests

# Demo（需要图形桌面）
roslaunch warehouse_utils demo_gazebo.launch
```

## 技术栈

| 层 | 方案 | 说明 |
|---|---|---|
| 任务管理 | 自研 MissionManager | BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION |
| 自主探索 | 自研 ExplorationFSM | Frontier + GoalSelector + CoverageMonitor |
| 导航管理 | 自研 NavigationManager | MoveBaseAdapter 封装 |
| 全局规划 | navfn (ROS Navigation) | 双轨：自研 A* 保留 |
| 局部规划 | DWA (ROS Navigation) | 双轨：自研 DWA 保留 |
| 代价地图 | costmap_2d (ROS Navigation) | Static + Obstacle + Inflation |
| 运动控制 | 自研 MotionController | 唯一 /cmd_vel 出口 |
| 定位 | Hector SLAM / Cartographer / VINS | 可切换 |

## 分支

| 分支 | 说明 |
|---|---|
| `main` | 稳定基线 |
| `refactor-v2` | 重构 + ROS Navigation 接入 |

## 文档索引

| 文档 | 用途 |
|---|---|
| [ROADMAP.md](docs/ROADMAP.md) | 开发计划（唯一入口） |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 系统架构 |
| [INTERFACES.md](docs/INTERFACES.md) | Topic 规格 |
| [ADR.md](docs/ADR.md) | 架构决策 |
| [AGENTS.md](docs/AGENTS.md) | Multi-Agent 协作 |
| [CHANGELOG.md](docs/CHANGELOG.md) | 版本记录 |
| [REGRESSION.md](docs/REGRESSION.md) | 回归测试 |
| [SESSION_LOG.md](docs/SESSION_LOG.md) | 开发日志 |

## 开发规范

- ROADMAP 是唯一开发入口
- 一个 Task = 代码 + Unit + Integration + System + Commit
- 只有 MotionController 发布 /cmd_vel
- 提交前核对 ARCHITECTURE / INTERFACES / ADR
