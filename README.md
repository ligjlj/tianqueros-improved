# TianqueROS-Improved

> 基于 TianqueROS + Gazebo 的仓库自主探索导航框架

## 架构

```
Mission → Exploration → Navigation → MotionController → cmd_vel
```

分层事件驱动设计。Cartographer/Hector 作为外部定位依赖，A*/DWA 作为可插拔 Planner/Controller。

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 分支

| 分支 | 说明 |
|---|---|
| `master` | 稳定基线 (58 tests, demo 可运行) |
| `refactor-v2` | 绞杀者重构 (PR1 done, PR2-12 进行中) |

## 重构路线 (12 PRs)

| PR | 内容 | 状态 |
|---|---|---|
| PR1 | MotionController 接管 `/cmd_vel` | ✅ |
| PR2 | MissionManager 接管启动流程 | ✅ |
| PR3 | EventBus / Event 定义 | ✅ |
| PR4 | CostmapManager | ⬜ |
| PR5 | PlannerInterface | ⬜ |
| PR6 | ControllerInterface | ⬜ |
| PR7 | NavigationManager | ⬜ |
| PR8 | RecoveryManager | ⬜ |
| PR9 | CoverageMonitor 后台化 | ⬜ |
| PR10 | GoalManager 重构 | ⬜ |
| PR11 | FrontierDetector 重构 | ⬜ |
| PR12 | 接入 VINS-Fusion | ⬜ |

## 测试

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make run_tests  # 58 tests
```

## Demo

```bash
roslaunch warehouse_utils demo_gazebo.launch
```

Gazebo: maze.world (300 walls) + ground_robot (diff-drive, 360° laser)
SLAM: hector_mapping
Planning: A* + DWA
Exploration: FSM-based with ReachabilityChecker + GoalSelector

## 文档

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — 系统架构
- [INTERFACES.md](docs/INTERFACES.md) — Topic 规格
- [ADR.md](docs/ADR.md) — 架构决策记录
- [ROADMAP.md](docs/ROADMAP.md) — 5周开发计划
- [SESSION_LOG.md](docs/SESSION_LOG.md) — 对话记录

## 开发规范

- 每个 PR 必须: 编译通过 + 测试通过 + Demo 可运行
- `master` 永不动，所有开发在 `refactor-v2`
- 文档与代码同步更新
