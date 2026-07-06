# TianqueROS-Improved

> 基于 TianqueROS + Gazebo 的仓库自主探索导航框架

## 架构

```
Mission → Exploration → Navigation → MotionController → cmd_vel
```

分层事件驱动设计。Cartographer/Hector 作为外部定位依赖，A*/DWA 作为可插拔 Planner/Controller。

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 快速开始

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 跑测试
catkin_make run_tests

# 启动 Demo（需要图形桌面）
roslaunch warehouse_utils demo_gazebo.launch
```

Gazebo: maze.world (300 walls) + ground_robot (diff-drive, 360° laser)
SLAM: hector_mapping | Planning: A* + DWA | Exploration: FSM 自主探索

## 分支

| 分支 | 说明 |
|---|---|
| `main` | 稳定基线 |
| `refactor-v2` | 绞杀者重构进行中 |

## 文档索引

| 文档 | 用途 |
|---|---|
| [ROADMAP.md](docs/ROADMAP.md) | 开发计划（看板，每天以它为准） |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 系统架构 |
| [INTERFACES.md](docs/INTERFACES.md) | Topic/Service 规格 |
| [ADR.md](docs/ADR.md) | 架构决策记录 |
| [CHANGELOG.md](docs/CHANGELOG.md) | 版本记录 |
| [AGENTS.md](docs/AGENTS.md) | AI 多 Agent 协作规范 |
| [SESSION_LOG.md](docs/SESSION_LOG.md) | 开发日志 |

## 开发规范

- 每个 PR：编译通过 + 测试通过 + Demo 可运行
- 提交前核对 ARCHITECTURE / INTERFACES / ADR / ROADMAP
- 只重构不改算法（绞杀者模式）
- 只有 MotionController 发 `/cmd_vel`
