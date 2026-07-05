# Architecture Decision Records — Warehouse Autonomous Exploration Framework

## ADR-001: Single cmd_vel Publisher

**Status:** Accepted

**Context:** Multiple modules (DWA, Recovery, Exploration bootstrap, manual teleop) all need to move the robot. Without a single authority, cmd_vel messages conflict, causing oscillation or stuck behavior.

**Decision:** Exactly one node in the entire system publishes `/cmd_vel`: **MotionController**. All other modules request motion through a defined interface.

```
Recovery ──► MotionController.requestRecovery() ──► cmd_vel
Exploration ──► MotionController (via NavigationManager)
```

**Consequences:**
- Recovery is NOT a Controller. It requests a recovery action; MotionController executes it.
- Adding TEB/MPC later does not require changing Recovery or Exploration.
- Manual teleop uses a separate topic `/cmd_vel_teleop`; a multiplexer in MotionController decides priority.

---

## ADR-002: Multi-Layer Costmap

**Status:** Accepted

**Context:** Different modules need different cost representations. Global inflation confuses Frontier detection (free cells near walls appear occupied) and prevents A* from finding narrow passages.

**Decision:** Four-layer costmap, each layer generated independently from the same OccupancyGrid.

```
OccupancyGrid
      │
      ▼
 RawCostmap (isFree / isOccupied / isUnknown)
      │
      ├──────────────┐
      ▼              ▼
PlannerCostmap   FrontierCostmap
(Inflation 0.3m)  (Raw, no inflation)
      │              │
      └──────┬───────┘
             ▼
    DynamicCostmap (laser + real-time obstacles)
```

| Layer | Used by | Generation |
|---|---|---|
| Raw | ReachabilityChecker, FrontierDetector | Direct from OccupancyGrid |
| Planner | A*, GlobalPlanner | Inflate raw by robot_radius + safety margin |
| Frontier | FrontierDetector, GoalSelector | Raw; Frontiers must not be affected by inflation |
| Dynamic | DWA, LocalPlanner | Planner layer + live laser scan overlay |

**Consequences:**
- Frontier detection always sees the true free/unknown boundary.
- A* can use aggressive inflation without hiding frontiers.
- DWA gets real-time obstacle data independent of the static map.

---

## ADR-003: Event-Driven FSM

**Status:** Accepted

**Context:** A polling FSM (`switch(state) { case X: doX(); }`) mixes state logic with action execution. When a planner call blocks or Gazebo stalls, the entire FSM freezes.

**Decision:** The FSM only manages state transitions and emits events. Action execution happens in separate workers that report results back via events.

```
FSM: "NeedPlan" ────► Planner worker ────► "PlanningSuccess" ────► FSM
                      (runs in background)
```

FSM states are pure: IDLE, WAITING_FOR_PLAN, FOLLOWING_PATH, etc.
FSM does NOT call `planner.plan()` directly.
FSM does NOT depend on ros::Timer — driven by `while(ros::ok()) { spinOnce; processEvents(); rate.sleep(); }`.

**Event types:**
- MAP_READY, SCAN_COMPLETE, NEED_PLAN, PLANNING_SUCCESS, PLANNING_FAILED
- GOAL_REACHED, STUCK, RECOVERY_REQUESTED, RECOVERY_DONE
- EXPLORATION_DONE, ERROR

**Consequences:**
- Planner can be a Plugin, replaced at runtime.
- FSM is testable in isolation: feed events, verify state transitions.
- Gazebo stalls do not halt the FSM main loop.

---

## ADR-004: Worker Pattern for Background Tasks

**Status:** Accepted

**Context:** Coverage computation, frontier visualization, CSV logging, and map saving are I/O-bound or CPU-heavy. Running them in the FSM tick blocks state transitions.

**Decision:** Background tasks use a Worker pattern with a job queue.

```cpp
class CoverageWorker {
    void enqueue(const OccupancyGrid& map);
    // Runs in dedicated thread, publishes /coverage at 1-2 Hz.
};
```

All Workers share a thread pool (future: configurable thread count).

**Workers planned:**
- CoverageWorker: compute coverage % at 2 Hz
- FrontierVizWorker: generate frontier markers at 2 Hz
- LogWorker: flush CSV buffers
- MapSaveWorker: non-blocking map save

**Consequences:**
- FSM tick time remains constant regardless of map size.
- Easy to add new background tasks without touching FSM.

---

## ADR-005: Plugin Architecture for Planner and Controller

**Status:** Accepted

**Context:** Hardcoding A* and DWA makes the framework a one-off demo. For a long-term research framework, planners and controllers must be swappable.

**Decision:** Define abstract Plugin interfaces. Implementation is loaded at runtime via pluginlib or manual dlopen.

```cpp
class GlobalPlannerPlugin {
public:
    virtual bool plan(const Costmap& map,
                      const Pose& start,
                      const Pose& goal,
                      Path& out) = 0;
    virtual ~GlobalPlannerPlugin() = default;
};

class LocalPlannerPlugin {
public:
    virtual Twist computeVelocity(const Costmap& map,
                                   const Pose& pose,
                                   const Path& path) = 0;
    virtual ~LocalPlannerPlugin() = default;
};
```

Selection via YAML:
```yaml
global_planner: "AStarPlanner"   # or "RRTStar", "HybridAStar"
local_planner:  "DWAPlanner"     # or "TEBPlanner", "MPCPlanner"
```

**Consequences:**
- Adding a new planner requires only implementing the interface; no other code changes.
- Planners are testable in complete isolation (no ROS, no Gazebo).
- NavigationManager never knows which concrete planner is running.

---

## ADR-006: Blackboard Pattern for Shared State

**Status:** Accepted

**Context:** Modules need to share state (robot pose, current goal, current path, mission state) but direct inter-module calls create tight coupling.

**Decision:** All shared state lives in a Blackboard. Modules read and write the Blackboard; they never call each other directly.

```cpp
class Blackboard {
public:
    // Thread-safe get/set.
    Pose getRobotPose();
    void setCurrentGoal(const Pose& g);
    Path getCurrentPath();
    void setNavigationStatus(NavStatus s);
    // ...
};
```

```
MissionManager ──write──► Blackboard.current_goal
Planner        ──read───► Blackboard.current_goal
Planner        ──write──► Blackboard.current_path
Controller     ──read───► Blackboard.current_path
Recovery       ──read───► Blackboard.navigation_status
```

**Consequences:**
- No module directly imports another module's header.
- Adding a new module only requires Blackboard read/write, no changes to existing code.
- Blackboard is the single source of truth; easy to log/replay for debugging.

---

## ADR-007: Separation Principles

**Status:** Accepted

**Principle 1 — Mission does not know the Planner.**
Mission sets `Blackboard.current_goal` and emits `NEED_PLAN`. It never calls `planner.plan()` or references A*/DWA by name.

**Principle 2 — Planner does not know Frontier.**
Planner accepts `(Costmap, Start, Goal) → Path`. It does not select goals, detect frontiers, or know about exploration.

**Principle 3 — Controller does not know the Map.**
Controller accepts `(Costmap, Pose, Path) → Twist`. It does not read OccupancyGrid or know about SLAM.

```
Mission ──► Goal ──► Planner ──► Path ──► Controller ──► cmd_vel
  │                                                    │
  └── Never references A*/DWA ─────────────────────────┘
```

**Consequences:**
- Planner can be replaced without touching Mission or Exploration.
- Controller works with any path source (A*, RRT*, manual).
- Each module is independently testable with mock inputs.

---

## ADR-008: Directory Structure

**Status:** Accepted

```
warehouse_nav/
├── mission/           — Lifecycle FSM, MissionManager
├── exploration/       — Frontier, Goal, Coverage
├── navigation/        — Planner interface, Controller interface, NavigationManager
├── costmap/           — Raw, Planner, Frontier, Dynamic layers
├── localization/      — CartographerAdapter, VINSAdapter, LocalizationManager
├── recovery/          — RecoveryManager + strategies
├── motion/            — MotionController (sole cmd_vel publisher)
├── diagnostics/       — Performance, Logger, Statistics
├── blackboard/        — Shared state
├── bringup/           — Launch files, rviz configs
├── interfaces/        — Plugin headers (planner, controller, recovery)
└── docs/              — ARCHITECTURE.md, INTERFACES.md, ROADMAP.md, ADR.md
```

Each directory is a ROS package or a logical module within a package. Dependencies flow top-down only: `bringup → mission/exploration/navigation → costmap/blackboard/interfaces`.

---

## Decision Log

| ADR | Date | Status | Title |
|---|---|---|---|
| 001 | 2026-07-05 | Accepted | Single cmd_vel Publisher |
| 002 | 2026-07-05 | Accepted | Multi-Layer Costmap |
| 003 | 2026-07-05 | Accepted | Event-Driven FSM |
| 004 | 2026-07-05 | Accepted | Worker Pattern |
| 005 | 2026-07-05 | Accepted | Plugin Architecture |
| 006 | 2026-07-05 | Accepted | Blackboard Pattern |
| 007 | 2026-07-05 | Accepted | Separation Principles |
| 008 | 2026-07-05 | Accepted | Directory Structure |
