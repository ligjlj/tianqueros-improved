# Warehouse Autonomous Exploration Framework — Session Log

## Timeline

```
Phase 0-2 (Jul 4)    GridMap + A* + DWA     ✅ 58 tests
Phase 3-5 (Jul 4)    PathTracker + FSM v1   ✅ 
Phase 6-8 (Jul 5 AM) Exploration + Demo     ⚠️ demo 不稳定
Debug    (Jul 5 AM)  根因分析 + 修复          4个工程问题
Design   (Jul 5 PM)  架构讨论 → ADR          8条决策
PR1      (Jul 5 PM)  MotionController       ✅ 完成
```

## Current Branch Status

```
main          — 基线 (58 tests, demo runnable)
refactor-v2   — 重构分支 (PR1 done, PR2-12 pending)
```

## Debugging Journey (Jul 5 AM)

4 root causes found in the demo:

| # | Symptom | Root Cause | Fix |
|---|---|---|---|
| 1 | FSM stuck in one state forever | `ros::Timer` depends on `/clock`; Gazebo maze loading causes clock stalls | Replace with `ros::WallTimer` |
| 2 | CPU pegged, FSM unresponsive | Coverage O(2048²) computed every 0.1s tick | Cache coverage, compute every 2s |
| 3 | A* never finds a path | `inflation_radius=1.0m` (20 cells) blocks all narrow maze passages | Reduce to 0.3m |
| 4 | Robot spins indefinitely after INITIAL_SCAN | Single stop command overridden by residual diff_drive state | Continuous stop for 1s after motion mode |

## Architecture Decisions (ADR, Jul 5 PM)

| ADR | Decision |
|---|---|
| 001 | Single `/cmd_vel` publisher: MotionController |
| 002 | 4-layer Costmap: Raw → Planner / Frontier → Dynamic |
| 003 | Event-driven FSM: emit events, don't call planners |
| 004 | Worker pattern: background tasks in thread pool |
| 005 | Plugin interfaces: A*/DWA swappable via YAML |
| 006 | Blackboard pattern: shared state, no inter-module imports |
| 007 | 3 separations: Mission ⊥ Planner ⊥ Frontier ⊥ Controller ⊥ Map |
| 008 | Directory: `warehouse_nav/` with 11 sub-modules |

## 12-PR Strangler Refactoring Plan

| PR | Content | Requirement |
|---|---|---|
| ✅ PR1 | MotionController 接管 `/cmd_vel` | Demo runnable |
| ⬜ PR2 | MissionManager 接管启动流程 | Demo runnable |
| ⬜ PR3 | EventBus / Event 定义 | Demo runnable |
| ⬜ PR4 | CostmapManager | Demo runnable |
| ⬜ PR5 | PlannerInterface | Demo runnable |
| ⬜ PR6 | ControllerInterface | Demo runnable |
| ⬜ PR7 | NavigationManager | Demo runnable |
| ⬜ PR8 | RecoveryManager | Demo runnable |
| ⬜ PR9 | CoverageMonitor 后台化 | Demo runnable |
| ⬜ PR10 | GoalManager 重构 | Demo runnable |
| ⬜ PR11 | FrontierDetector 重构 | Demo runnable |
| ⬜ PR12 | 接入 VINS-Fusion | Demo runnable |

## PR1 Detail

**MotionController** — sole `/cmd_vel` publisher.

Priority multiplexer:
- Priority 3: `/cmd_vel_recovery` (Exploration: INITIAL_SCAN, RECOVERY)
- Priority 2: `/cmd_vel_nav` (DWA)
- Priority 1: stop (default timeout 0.5s)

Existing nodes unchanged. Remapping in launch file:
```xml
<remap from="/cmd_vel" to="/cmd_vel_nav"/>       <!-- DWA -->
<remap from="/cmd_vel" to="/cmd_vel_recovery"/>   <!-- Exploration -->
```

Files:
- `src/warehouse_motion/src/motion_controller_node.cpp` (60 lines)
- `src/warehouse_motion/launch/motion.launch`
- `src/warehouse_utils/launch/demo_gazebo.launch` (updated)

## Files Created

```
~/catkin_ws/docs/
├── ARCHITECTURE.md    — 4-layer architecture, module responsibilities
├── INTERFACES.md      — 15 topics, TF tree, message ownership rules
├── ROADMAP.md         — 5-week × 5-task checklist
├── ADR.md             — 8 architecture decisions with rationale
└── SESSION_LOG.md     — this file

~/catkin_ws/src/warehouse_motion/   ← PR1 (new)
├── src/motion_controller_node.cpp
├── launch/motion.launch
├── CMakeLists.txt
└── package.xml
```

## Key Principles Going Forward

1. **Strangler pattern only.** New layers wrap old code. Never rewrite.
2. **Demo must run after every PR.** If it breaks, rollback.
3. **One change per PR.** Don't refactor + optimize simultaneously.
4. **main branch never touched.** All work on refactor-v2.
5. **Robot always moves through MotionController.** No other cmd_vel publishers.
