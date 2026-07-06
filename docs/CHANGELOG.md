# Changelog

## v0.8 — 稳定性提升 (2026-07-06)

- sim_time → WallTime 全模块替换 (exploration FSM + DWA controller)
- move_base transform_tolerance: 2.0 → 0.3
- INITIAL_SCAN 旋转确认: 基于 odom 偏航角 (≥315°) + 14s 超时安全网
- E2E 自动化测试脚本: tests/exploration_e2e.py
- 修复 test_goal.cpp 预存失败 (3→4 tests pass)

## v0.7 — 五层架构重构 (2026-07-06)

**架构决策：** 从"自研所有算法"转变为"构建自主探索框架"。

- 五层架构: Mission → Exploration → Navigation → Backend → Localization
- ADR-009: 算法层使用 ROS Navigation Stack（navfn + DWA + costmap_2d）
- 自研 A*/DWA 保留，双轨切换
- MotionController 保留为速度仲裁器
- EventBus 保留为跨模块通信

## v0.6 — ROS Navigation Stack Adapter (2026-07-06)

- MoveBaseAdapter: 封装 move_base action client
- NavigationManager v2: 双轨 (move_base / 自研)
- costmap_2d 配置: global + local + DWA + navfn
- 自研 A*/DWA 保留，config 切换
- 架构: Mission → Exploration → NavigationManager → Adapter → move_base

## v0.5 — PR12 + 三级测试 (2026-07-06)

- URDF: RGB camera + IMU noise model
- VINS: config + topic relay
- 三级测试体系: Unit(72 tests) + Integration + System + Regression(8项)
- Multi-Agent: 6 profiles + Kanban board

## v0.4 — Week 2: Costmap + Interfaces + Navigation (2026-07-06)

- warehouse_costmap: RawCostmap + InflatedCostmap
- warehouse_interfaces: Planner/Controller plugins
- warehouse_navigation: NavigationManager
- warehouse_recovery: RecoveryManager

## v0.3 — EventBus + MissionFSM (2026-07-06)

- EventBus: 12 event types
- MissionFSM: 9 transition tests

## v0.2 — MissionManager (2026-07-06)

- Lifecycle: BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION→FINISHED

## v0.1 — MotionController (2026-07-05)

- Sole /cmd_vel publisher, priority mux

## v0.0 — Baseline (2026-07-04)

- GridMap + A* + DWA + Frontier, 58 tests
