# Warehouse Autonomous Exploration Framework — Interfaces

## 1. Topic Overview

```
/map ────────────────► Exploration ──► /exploration_goal ──► Navigation ──► /cmd_vel ──► Robot
                        │                                       │
                        ▼                                       ▼
                  /frontier_marker                         /global_path
                  /exploration_state                       /local_path
                  /coverage                                /nav_status
                                                          /dwa_traj

/odom ────────────────────────────────────────────────────► Navigation
/scan ────────────────────────────────────────────────────► Navigation (DWA)
/tf ─────────────────────► Exploration, Navigation
```

## 2. Topic Specifications

### Mission Layer

| Topic | Type | Publisher | Subscribers | Rate | Description |
|---|---|---|---|---|---|
| `/mission_state` | `std_msgs/String` | mission_node | diagnostics, rviz | 1 Hz | Current mission FSM state |
| `/exploration_enable` | `std_msgs/Bool` | mission_node | exploration_node | latched | Enable/disable exploration |

### Exploration Layer

| Topic | Type | Publisher | Subscribers | Rate | Description |
|---|---|---|---|---|---|
| `/exploration_goal` | `geometry_msgs/PoseStamped` | exploration_node | navigation_node | on change | Selected frontier goal |
| `/exploration_state` | `std_msgs/String` | exploration_node | diagnostics, rviz | 2 Hz | FSM state name |
| `/frontier_marker` | `visualization_msgs/Marker` | exploration_node | rviz | 2 Hz | Frontier cluster visualization |
| `/coverage` | `std_msgs/Float32` | coverage_monitor | diagnostics | 1 Hz | Map coverage percentage |

### Navigation Layer

| Topic | Type | Publisher | Subscribers | Rate | Description |
|---|---|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | global_planner | local_planner, exploration, rviz | on plan | A* output path |
| `/local_path` | `nav_msgs/Path` | local_planner | rviz | 10 Hz | DWA best trajectory |
| `/cmd_vel` | `geometry_msgs/Twist` | local_planner | robot (diff_drive) | 10 Hz | Velocity command |
| `/nav_status` | `std_msgs/String` | navigation_monitor | diagnostics, mission | 5 Hz | IDLE / PLANNING / FOLLOWING / STUCK / RECOVERY |
| `/dwa_traj` | `visualization_msgs/Marker` | local_planner | rviz | 2 Hz | All sampled DWA trajectories |

### Sensor Inputs (from external modules)

| Topic | Type | Publisher | Description |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | cartographer/hector | SLAM output |
| `/odom` | `nav_msgs/Odometry` | robot driver | Wheel/visual odometry |
| `/scan` | `sensor_msgs/LaserScan` | robot laser | 2D LiDAR |
| `/tf` | `tf2_msgs/TFMessage` | cartographer + robot | map→odom→base_link→laser |
| `/clock` | `rosgraph_msgs/Clock` | gazebo | Simulation time |

## 3. TF Tree

```
map ──► odom ──► base_link ──► laser
(carto)  (diff_drive)         (static)
```

All algorithms use the `map` frame. The `world` frame is never referenced.

## 4. Service Interfaces (future)

| Service | Type | Provider | Description |
|---|---|---|---|
| `/save_map` | `std_srvs/Trigger` | mission_node | Trigger map save |
| `/replan` | `std_srvs/Trigger` | navigation_node | Force re-plan |
| `/abort_goal` | `std_srvs/Trigger` | navigation_node | Cancel current navigation goal |

## 5. Message Ownership Rule

Each topic has exactly ONE publisher. No two nodes publish to the same topic (except diagnostics which is read-only).

Exception: during RECOVERY, `recovery_manager` temporarily publishes `/cmd_vel`. This is the only case where two nodes publish to the same topic, and it is explicitly gated by the FSM state.
