# Warehouse Autonomous Exploration Framework — Roadmap

## Week 1: System Refactoring

### 1.1 Package Structure
- [ ] Create `warehouse_mission` package (CMakeLists, package.xml)
- [ ] Create `warehouse_navigation` package
- [ ] Create `warehouse_diagnostics` package
- [ ] Move existing `warehouse_utils`, `warehouse_planner`, `warehouse_controller`, `warehouse_exploration` to new structure
- [ ] Update all CMakeLists for inter-package dependencies

### 1.2 Interface Definitions
- [ ] Define all topic names and message types (per INTERFACES.md)
- [ ] Create `warehouse_msgs` for custom messages (if needed)
- [ ] Document each topic's publisher, subscribers, and rate

### 1.3 MissionManager
- [ ] Implement MissionManager FSM: BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED
- [ ] MissionManager publishes `/mission_state` and `/exploration_enable`
- [ ] Main loop uses `while(ros::ok()) { spinOnce; fsm.update(); rate.sleep(); }`
- [ ] Wall-clock based timing (no sim_time dependency)
- [ ] Unit tests for state transitions

### 1.4 Event Mechanism
- [ ] Define event types: MAP_READY, SCAN_COMPLETE, EXPLORATION_DONE, ERROR
- [ ] Event bus: publish/subscribe within process (no ROS topic overhead)
- [ ] FSM transitions triggered by events, not polling

### 1.5 Diagnostics
- [ ] Implement ResourceMonitor: CPU, tick time, map update latency
- [ ] Implement CsvLogger in `warehouse_diagnostics`
- [ ] Adaptive frequency: slow down when CPU > 80%
- [ ] All modules log to CSV for experiment data collection

**Week 1 Goal:** System starts, enters all lifecycle states, rviz shows state transitions. No exploration yet.

---

## Week 2: Navigation Refactoring

### 2.1 CostmapManager
- [ ] Three-layer costmap: Raw / Inflated (0.3m) / Dynamic (laser)
- [ ] `getCost(layer, x, y)` interface
- [ ] ReachabilityChecker uses Layer 1 (Raw)
- [ ] A* uses Layer 2 (Inflated)
- [ ] DWA uses Layer 3 (Dynamic)
- [ ] Unit tests for each layer

### 2.2 NavigationManager
- [ ] Orchestrates: receive goal → plan → execute → report
- [ ] Does NOT publish cmd_vel (delegates to LocalPlanner)
- [ ] Publishes `/nav_status` (IDLE / PLANNING / FOLLOWING / STUCK / RECOVERY)

### 2.3 RecoveryManager
- [ ] Recovery strategies: BACKUP, ROTATE, REPLAN, SKIP_GOAL
- [ ] Triggered by NavigationMonitor
- [ ] Recovery publishes cmd_vel during active recovery phase only
- [ ] Unit tests for recovery transitions

### 2.4 Planner Interface
- [ ] Abstract `GlobalPlanner` base class
- [ ] A* implementation conforms to interface
- [ ] `plan(map, start, goal) → Path`
- [ ] RRT* can be added later without changing NavigationManager

### 2.5 Controller Interface
- [ ] Abstract `LocalPlanner` base class
- [ ] DWA implementation conforms to interface
- [ ] `computeVelocity(map, state, path) → (v, w)`
- [ ] TEB/MPC can be added later

**Week 2 Goal:** Robot reliably navigates from point A to point B using rviz "2D Nav Goal".

---

## Week 3: Autonomous Exploration

### 3.1 FrontierDetector
- [ ] Scan OccupancyGrid O(N) for free cells adjacent to unknown
- [ ] Output: list of frontier GridCells
- [ ] Unit tests: empty map, full map, boundary cases

### 3.2 FrontierClusterer
- [ ] BFS connected-components clustering (8-connectivity)
- [ ] Output: clusters with center, size, bounding box
- [ ] Unit tests: single cluster, multiple clusters, tiny clusters

### 3.3 GoalManager
- [ ] Track goal state: UNTRIED → TRIED → FAILED → BLACKLISTED
- [ ] Blacklist with 60s cooldown, auto-expire
- [ ] Deduplication by proximity (0.5m radius)
- [ ] Unit tests: blacklist, expiry, dedup

### 3.4 ExplorationFSM
- [ ] States: DETECT → SELECT → PLAN → FOLLOW → RECOVERY → UPDATE
- [ ] Event-driven transitions (no while(true) polling)
- [ ] Reachability filter: only score clusters BFS-connected to robot
- [ ] GoalSelector: 4-dim weighted scoring with fail_count penalty
- [ ] Unit tests: state transitions, timeout, retry

### 3.5 CoverageMonitor
- [ ] Background 2Hz thread, independent of FSM
- [ ] Subscribes to `/map`, computes coverage %
- [ ] Publishes `/coverage` (Float32)
- [ ] Triggers FINISHED signal when coverage stagnates

**Week 3 Goal:** Robot autonomously explores and maps the maze without human intervention.

---

## Week 4: Visual-Inertial Fusion

### 4.1 Gazebo Sensors
- [ ] Add RGB camera to ground_robot.urdf
- [ ] Add IMU with noise model to ground_robot.urdf
- [ ] Verify camera publishes `/camera/image_raw`
- [ ] Verify IMU publishes `/imu`

### 4.2 IMU Noise Model
- [ ] Configure realistic noise parameters (gyro, accel biases)
- [ ] Verify noise is visible in `/imu` data

### 4.3 VINS-Fusion Integration
- [ ] Launch VINS-Fusion with camera + IMU topics
- [ ] Verify VINS publishes `/vins_estimator/odometry`
- [ ] Compare VINS trajectory vs ground truth P3D

### 4.4 LocalizationManager
- [ ] Support multiple localization sources: Cartographer, VINS, Ground Truth
- [ ] Publish unified `/localization_pose`
- [ ] Runtime switching via parameter or service
- [ ] Unit tests for source switching

### 4.5 Localization Comparison
- [ ] Collect GT, Cartographer, VINS trajectories on same path
- [ ] Compute ATE (Absolute Trajectory Error) for each
- [ ] Generate comparison plots

**Week 4 Goal:** Quantitative comparison of localization methods' impact on mapping and exploration.

---

## Week 5: Optimization & Experiments

### 5.1 Frontier Scoring Optimization
- [ ] Tune weights: distance, information, size, fail_count
- [ ] A/B test different scoring strategies
- [ ] Measure: total exploration time, coverage speed, failure rate

### 5.2 Recovery Strategy Optimization
- [ ] Compare recovery strategies: backup-only, rotate-only, combined
- [ ] Measure recovery success rate and time cost
- [ ] Optimize recovery parameters (backup distance, rotate angle)

### 5.3 Parameter Tuning
- [ ] Grid search or manual tuning: inflation_radius, goal_tolerance, timeouts
- [ ] DWA parameters: v_samples, w_samples, alpha/beta/gamma
- [ ] Exploration: min_cluster_size, blacklist_duration

### 5.4 Experiment Data Collection
- [ ] Run 10 full exploration trials per configuration
- [ ] Log: coverage_vs_time, path_length, planning_time, recovery_count
- [ ] Export CSV to `~/catkin_ws/experiments/`

### 5.5 Paper Figures
- [ ] Coverage over time plot (comparing configurations)
- [ ] Trajectory overlay on map
- [ ] Planning time histogram
- [ ] Recovery event timeline
- [ ] Localization error comparison (Cartographer vs VINS vs GT)

**Week 5 Goal:** Complete experiment dataset and publication-quality figures.
