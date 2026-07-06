# 仓库自主探索框架 — 接口定义

## 1. Topic 总览

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

## 2. Topic 规格

### 任务层

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/mission_state` | `std_msgs/String` | mission_node | diagnostics, rviz | 1 Hz | 当前任务 FSM 状态 |
| `/exploration_enable` | `std_msgs/Bool` | mission_node | exploration_node | 锁存 | 启停探索 |

### 探索层

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/exploration_goal` | `geometry_msgs/PoseStamped` | exploration_node | navigation_node | 按需 | 选中的 Frontier 目标 |
| `/exploration_state` | `std_msgs/String` | exploration_node | diagnostics, rviz | 2 Hz | FSM 状态名 |
| `/frontier_marker` | `visualization_msgs/Marker` | exploration_node | rviz | 2 Hz | Frontier 聚类可视化 |
| `/coverage` | `std_msgs/Float32` | coverage_monitor | diagnostics | 1 Hz | 地图覆盖率百分比 |

### 导航层

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | global_planner | local_planner, exploration, rviz | 按需 | A* 输出路径 |
| `/local_path` | `nav_msgs/Path` | local_planner | rviz | 10 Hz | DWA 最优轨迹 |
| `/cmd_vel` | `geometry_msgs/Twist` | motion_controller | robot (diff_drive) | 10 Hz | 速度指令 |
| `/nav_status` | `std_msgs/String` | navigation_monitor | diagnostics, mission | 5 Hz | IDLE / PLANNING / FOLLOWING / STUCK / RECOVERY |
| `/dwa_traj` | `visualization_msgs/Marker` | local_planner | rviz | 2 Hz | DWA 全部采样轨迹 |

### 传感器输入（来自外部模块）

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | cartographer / hector | 建图输出 |
| `/odom` | `nav_msgs/Odometry` | robot driver | 轮式/视觉里程计 |
| `/scan` | `sensor_msgs/LaserScan` | robot laser | 2D 激光雷达 |
| `/tf` | `tf2_msgs/TFMessage` | cartographer + robot | map→odom→base_link→laser |
| `/clock` | `rosgraph_msgs/Clock` | gazebo | 仿真时间 |

## 3. TF 树

```
map ──► odom ──► base_link ──► laser
(carto)  (diff_drive)         (static)
```

所有算法统一使用 `map` 坐标系。禁止直接使用 `world` 坐标系。

## 4. Service 接口（规划中）

| Service | 类型 | 提供者 | 说明 |
|---|---|---|---|
| `/save_map` | `std_srvs/Trigger` | mission_node | 触发地图保存 |
| `/replan` | `std_srvs/Trigger` | navigation_node | 强制重规划 |
| `/abort_goal` | `std_srvs/Trigger` | navigation_node | 取消当前导航目标 |

## 5. 消息所有权规则

每个 Topic 有且仅有一个发布者。两个节点不得发布到同一个 Topic（诊断层只读，例外）。

唯一例外：RECOVERY 期间，`recovery_manager` 临时发布 `/cmd_vel`。这是唯一两个节点发布到同一 Topic 的场景，且由 FSM 状态显式门控。
