# Changelog

## v0.5 — 12-PR 绞杀者重构完成: VINS-Fusion 接入 + Week 4 Exit Criteria (2026-07-06)

- PR12: ground_robot.urdf 增加 RGB 相机 + IMU（带噪声模型）
- PR12: LocalizationManager 多源定位切换（VINS-Fusion / Cartographer / Ground Truth）
- PR12: ATE 对比实验（VINS vs Cartographer vs GT）
- ROADMAP: 12-PR 绞杀者重构全部完成，Week 1-4 Exit Criteria 全部达成
- Week 5 剩余: 10 次完整探索实验 + 论文级图表

## v0.4 — Week 2 Complete: Costmap + Interfaces + Navigation + Recovery (2026-07-06)

- warehouse_costmap: RawCostmap + InflatedCostmap (4 tests)
- warehouse_interfaces: GlobalPlannerPlugin + LocalPlannerPlugin (ADR-005)
- warehouse_navigation: NavigationManager (/nav_status, /nav_stuck)
- warehouse_recovery: RecoveryManager (BACKUP 2s + ROTATE 3s)
- Demo: 13 nodes running with full pipeline
- ROADMAP Week 1-2 Exit Criteria met

## v0.3 — EventBus + MissionFSM 测试 (2026-07-06)

- EventBus: 进程内线程安全事件总线（warehouse_utils）
- 12 种事件类型：MAP_READY 到 ERROR
- MissionFSM: 可测试的生命周期状态机（回调注入）
- 9 项状态转移单元测试
- MissionManager 主循环改用 WallTime（不依赖 sim_time）
- ROADMAP 1.3 完成

## v0.2 — MissionManager (2026-07-06)

- warehouse_mission 包
- MissionManager: BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION→FINISHED
- INITIAL_SCAN 原地旋转 8 秒建初始地图
- 从 ExplorationFSM 移除 WAIT_FOR_MAP / INITIAL_SCAN
- ExplorationFSM 从 DETECT_FRONTIER 直接启动
- ADR-003 部分落地（while+spinOnce+rate.sleep）

## v0.1 — MotionController (2026-07-05)

- warehouse_motion 包
- MotionController: 优先级复用 (recovery > nav > stop)
- DWA → /cmd_vel_nav, Exploration → /cmd_vel_recovery
- MotionController 是唯一 /cmd_vel 发布者
- ADR-001 落地

## v0.0 — Baseline (2026-07-04)

- warehouse_utils: GridMap + Inflation (19 tests)
- warehouse_planner: A* (13 tests, 含 100 组随机)
- warehouse_controller: DWA (10 tests) + PathTracker (7 tests)
- warehouse_exploration: FrontierDetector + GoalSelector + FSM (16 tests)
- 58 项测试全部通过
- maze.world + Hector SLAM Demo 可运行
