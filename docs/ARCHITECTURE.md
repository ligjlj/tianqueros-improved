# Warehouse Autonomous Exploration Framework — Architecture

## 1. System Overview

```
                        Mission Manager
                              │
           ┌──────────────────┼──────────────────┐
           ▼                  ▼                  ▼
    Localization         Exploration          Diagnostics
           │                  │                  │
           ▼                  ▼                  ▼
         Pose          Frontier / Goal      Resource Monitor
           │                  │             Coverage Monitor
           │                  │
           └──────────┬───────┘
                      ▼
              Navigation Manager
                      │
           ┌──────────┴──────────┐
           ▼                     ▼
    Global Planner          Local Planner
    (A* Interface)          (DWA Interface)
           │                     │
           └──────────┬──────────┘
                      ▼
              Motion Controller
                      │
                   cmd_vel
```

## 2. Layer Responsibilities

### Layer 0 — Mission Manager (`warehouse_mission`)
Lifecycle FSM. Does NOT know about frontiers, paths, or cmd_vel.

| Responsibility | Detail |
|---|---|
| Lifecycle states | BOOT → WAIT_MAP → INITIAL_SCAN → LOCALIZATION_READY → EXPLORATION → SAVE_MAP → FINISHED |
| Input | System health, localization status, exploration completion signal |
| Output | Mission state (`/mission_state`), exploration enable flag |

### Layer 1 — Exploration Manager (`warehouse_exploration`)
Frontier detection, goal selection, exploration strategy. Does NOT control the robot.

| Module | Responsibility |
|---|---|
| FrontierDetector | Find free cells adjacent to unknown |
| FrontierClusterer | BFS cluster frontier cells |
| GoalSelector | Weighted scoring: distance × info × size × fail_count |
| ReachabilityChecker | BFS flood-fill; filter unreachable clusters |
| GoalManager | Blacklist, fail count, retry cooldown |
| ExplorationFSM | DETECT → SELECT → PLAN → FOLLOW → RECOVERY |
| CoverageMonitor | Background 2Hz: map coverage %, independent of FSM |

Input:  `/map` (OccupancyGrid), `/tf` (robot pose)
Output: `/exploration_goal` (PoseStamped), `/frontier_marker`, `/exploration_state`

### Layer 2 — Navigation Manager (`warehouse_navigation`)
Path planning and execution orchestration.

| Module | Responsibility |
|---|---|
| CostmapManager | Three-layer costmap: Raw / Inflated / Dynamic |
| GlobalPlanner | A* interface: map + start + goal → Path |
| LocalPlanner | DWA interface: path + laser + odom → cmd_vel |
| PathTracker | Pure Pursuit: global path → lookahead sub-goal |
| RecoveryManager | Backup, rotate, re-plan on failure |
| NavigationMonitor | Stuck detection, oscillation, timeout, progress |

Input:  `/exploration_goal`, `/map`, `/odom`, `/scan`
Output: `/cmd_vel`, `/global_path`, `/local_path`, `/nav_status`

### Layer 3 — Diagnostics (`warehouse_diagnostics`)
Performance monitoring and logging. Read-only, never controls the robot.

| Module | Responsibility |
|---|---|
| ResourceMonitor | CPU, tick time, map update latency |
| CsvLogger | Structured CSV export for experiments |
| Visualization | RViz markers for debugging |

## 3. External Dependencies (NOT part of the framework)

| Component | Role | Interface |
|---|---|---|
| Cartographer / Hector / VINS | SLAM + localization | Publishes `/map`, `/tf` (map→odom) |
| Gazebo | Simulation | Spawns robot, runs physics |
| robot URDF/SDF | Robot model | Sensors (laser, IMU, camera), actuators |

## 4. Invariants

1. Only the Motion Controller publishes `/cmd_vel`.
2. No module modifies Cartographer, A*, or DWA internals.
3. All module communication is via ROS topics (no direct class calls across layers).
4. All parameters come from YAML files; no hardcoded values.
5. Every module has a corresponding unit test.
