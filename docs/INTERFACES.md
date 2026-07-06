# 仓库自主探索框架 — 接口定义

> 五层架构之间的通信规范。每层只暴露接口，不暴露实现。

## 1. Topic 总览

```
/map ────────────────► Exploration ──► /exploration_goal ──► Navigation ──► /cmd_vel ──► Robot
                        │                                       │
                        ▼                                       ▼
                  /frontier_marker                         /nav_status
                  /exploration_state                       /current_goal
                  /coverage                       /move_base/feedback
                  /mission_state                  /move_base/result
```

## 2. Mission Layer

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/mission_state` | String | Mission→Diag | 1Hz | BOOT/WAIT_MAP/INITIAL_SCAN/EXPLORATION/FINISHED |
| `/exploration_enable` | Bool (latched) | Mission→Exploration | 按需 | 启停探索 |

## 3. Exploration Layer

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/exploration_goal` | PoseStamped (latched) | Exploration→Navigation | 按需 | 选中的 Frontier 目标 |
| `/exploration_state` | String | Exploration→Diag | 2Hz | FSM 当前状态名 |
| `/frontier_marker` | Marker | Exploration→RViz | 2Hz | Frontier 聚类可视化 |
| `/coverage` | Float32 | Exploration→Diag | 1Hz | 地图覆盖率% |

Exploration 输入：`/map` (OccupancyGrid), `/tf` (robot pose)
Exploration 输出：`/exploration_goal` (PoseStamped)

## 4. Navigation Layer

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/nav_status` | String | Nav→Diag | 2Hz | IDLE/ACTIVE/SUCCEEDED/ABORTED |
| `/current_goal` | PoseStamped | Nav→Diag | 按需 | 当前导航目标 |

Navigation 输入：`/exploration_goal`, `/map`, `/odom`, `/scan`
Navigation 输出：通过 Adapter 调用 Backend

**NavigationManager 接口（C++）：**
```cpp
bool sendGoal(const PoseStamped& goal);
void cancelGoal();
enum State { IDLE, ACTIVE, SUCCEEDED, ABORTED };
State getState();
bool isGoalReached();
```

## 5. Navigation Backend

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/move_base/goal` | MoveBaseActionGoal | Nav→move_base | 按需 | 通过 actionlib 发送 |
| `/move_base/feedback` | MoveBaseActionFeedback | move_base→Nav | 1Hz | 导航进度 |
| `/move_base/result` | MoveBaseActionResult | move_base→Nav | 按需 | 导航结果 |
| `/cmd_vel_nav` | Twist | move_base→Motion | 20Hz | 规划的速度指令 |

move_base 内部管理：
- costmap_2d（global + local）
- navfn / global_planner
- dwa_local_planner
- recovery_behaviors

## 6. Localization Layer

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/localization_pose` | PoseWithCovarianceStamped | Loc→Nav | 30Hz | 统一定位输出 |
| `/tf` (map→odom) | TFMessage | Loc→全系统 | 30Hz | 坐标系变换 |

Localization 输入：传感器数据（/scan, /imu, /camera）
Localization 输出：Pose + TF

## 7. MotionController

| Topic | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/cmd_vel_nav` | Twist | move_base→Motion | 20Hz | 正常导航 |
| `/cmd_vel_recovery` | Twist | Recovery→Motion | 按需 | 恢复动作 |
| `/cmd_vel_manual` | Twist | Joystick→Motion | 按需 | 人工遥控 |
| `/cmd_vel_emergency` | Twist | Safety→Motion | 按需 | 紧急停止 |
| `/cmd_vel` | Twist | **唯一** Motion→Robot | 20Hz | **唯一出口** |

## 8. EventBus（进程内事件）

| 事件 | 发送者 | 监听者 |
|---|---|---|
| MAP_READY | Mission (mapCb) | Exploration |
| SCAN_COMPLETE | Mission (INITIAL_SCAN done) | Exploration |
| EXPLORATION_ENABLED | Mission | Exploration |
| EXPLORATION_DONE | Exploration | Mission |
| GOAL_REACHED | Navigation | Exploration + Mission |
| STUCK | NavigationMonitor | Recovery + Mission |
| RECOVERY_REQUESTED | Navigation | RecoveryManager |
| RECOVERY_DONE | RecoveryManager | Navigation |
| ERROR | Any | Mission |

## 9. 消息所有权规则

- 每个 Topic 有唯一发布者（诊断层除外）。
- 只有 MotionController 发布 `/cmd_vel`。
- 只有 Mission/Exploration 可以设置 Goal。
- Navigation 不知道 Goal 来源。
