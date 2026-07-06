# TianqueROS-Improved — 系统架构

> 模块化自主探索框架（Autonomous Exploration Framework）。
> 五层架构，每层职责单一。算法是插件，不是核心。

## 1. 五层架构

```
                    Application Layer
┌──────────────────────────────────────────────────┐
│                MissionManager                    │
│  BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORE       │
│                                                   │
│  负责：要不要探索？                                 │
│  不知道：机器人怎么走                               │
└──────────────────────────────────────────────────┘
                          │
                          ▼
                 Exploration Layer
┌──────────────────────────────────────────────────┐
│             ExplorationManager                   │
│                                                   │
│  FrontierDetector → Clusterer → GoalSelector     │
│  CoverageMonitor                                  │
│                                                   │
│  输出：geometry_msgs::PoseStamped                 │
│  不知道：A* / DWA / move_base                     │
└──────────────────────────────────────────────────┘
                          │
                    Navigation Goal
                          │
                          ▼
                 Navigation Layer
┌──────────────────────────────────────────────────┐
│             NavigationManager                     │
│                                                   │
│  sendGoal(pose) → bool                            │
│  cancelGoal()                                     │
│  getState() → {IDLE,ACTIVE,SUCCEEDED,ABORTED}    │
│                                                   │
│  Adapter Pattern：                                │
│  ┌──────────────┬──────────────┐                  │
│  │ MoveBaseAdapter │ SelfAdapter │               │
│  └──────────────┴──────────────┘                  │
│  不知道：Frontier / Mission                       │
└──────────────────────────────────────────────────┘
                          │
                    Adapter 调用
                          │
                          ▼
              Navigation Backend
┌──────────────────────────────────────────────────┐
│           ROS Navigation Stack                    │
│                                                   │
│  costmap_2d (Static + Obstacle + Inflation)      │
│  navfn / global_planner (A* / Dijkstra)          │
│  dwa_local_planner (速度采样)                     │
│  recovery_behaviors (旋转恢复 + 清除代价)          │
│                                                   │
│  → cmd_vel_nav → MotionController                 │
└──────────────────────────────────────────────────┘
                          │
                          ▼
               Localization Layer
┌──────────────────────────────────────────────────┐
│            LocalizationManager                    │
│                                                   │
│  Ground Truth  |  Cartographer  |  VINS-Fusion   │
│  → Pose                                          │
│  → /tf (map → odom)                              │
│                                                   │
│  Navigation 不知道定位来源                         │
└──────────────────────────────────────────────────┘
```

## 2. 辅助模块

### MotionController（速度仲裁）

```
move_base    → /cmd_vel_nav       ─┐
Recovery     → /cmd_vel_recovery  ─┤
Manual       → /cmd_vel_manual    ─┼→ MotionController → /cmd_vel
Emergency    → /cmd_vel_emergency ─┘

唯一 /cmd_vel 发布者。优先级：emergency > recovery > nav > manual
```

### Diagnostics（独立诊断层）

```
ResourceMonitor  → CPU / Memory / Tick
CsvLogger        → 实验数据采集
Visualization    → RViz Marker
Regression       → 8项回归检查点
```

### EventBus（跨模块事件）

```
MAP_READY → SCAN_COMPLETE → EXPLORATION_ENABLED → GOAL_REACHED
→ STUCK → RECOVERY_REQUESTED → EXPLORATION_DONE → FINISHED
```

所有 Manager 监听 EventBus，不直接互相调用。

## 3. 层间依赖规则

```
Mission        → 不知道 Planner / Controller
Exploration    → 不知道 A* / DWA / move_base
Navigation     → 不知道 Frontier / Goal 来源
Backend        → 不知道 Mission / Exploration
Localization   → 不知道谁在用 Pose
Diagnostics    → 只读，不写
MotionControl  → 唯一 /cmd_vel 出口
```

## 4. 包目录

```
warehouse_nav/
├── warehouse_mission/        — MissionManager
├── warehouse_exploration/    — Frontier + Goal + Coverage
├── warehouse_navigation/     — NavigationManager + MoveBaseAdapter
├── warehouse_localization/   — LocalizationManager + Adapters
├── warehouse_motion/         — MotionController
├── warehouse_diagnostics/    — ResourceMonitor + CsvLogger
├── warehouse_interfaces/     — Plugin接口
├── warehouse_costmap/        — 自研 Costmap（保留）
├── warehouse_planner/        — 自研 A*（保留，双轨）
├── warehouse_controller/     — 自研 DWA（保留，双轨）
├── warehouse_recovery/       — Recovery（保留）
├── bringup/                  — Launch + RViz + 参数
└── docs/                     — 架构文档
```

## 5. 扩展实验

| 实验维度 | 可选方案 |
|---|---|
| Localization | GT / Cartographer / VINS / FAST-LIO |
| Global Planner | navfn / global_planner / 自研 A* / Hybrid A* |
| Local Planner | DWA / TEB / MPC / 自研 DWA |
| Exploration | Frontier / NBV / Active SLAM |

Mission 层完全不用修改。
