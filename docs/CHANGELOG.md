# Changelog

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
