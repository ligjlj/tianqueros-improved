# 仓库自主探索框架 — 系统架构

## 1. 系统总览

```
                     MissionManager（任务管理器）
                              │
           ┌──────────────────┼──────────────────┐
           ▼                  ▼                  ▼
    Localization          Exploration          Diagnostics
    （定位层）            （探索层）            （诊断层）
           │                  │                  │
           ▼                  ▼                  ▼
         Pose           Frontier / Goal      Resource Monitor
        （位姿）         （前沿/目标）         （资源监控）
           │                  │             Coverage Monitor
           │                  │             （覆盖率监控）
           └──────────┬───────┘
                      ▼
              NavigationManager
              （导航管理器）
                      │
           ┌──────────┴──────────┐
           ▼                     ▼
    GlobalPlanner           LocalPlanner
    （全局规划器）           （局部规划器）
    (A* 接口)               (DWA 接口)
           │                     │
           └──────────┬──────────┘
                      ▼
              MotionController
              （运动控制器）
                      │
                   cmd_vel
```

## 2. 各层职责

### 第 0 层 — MissionManager（`warehouse_mission`）

生命周期状态机。不知道 Frontier、Path 或 cmd_vel 的存在。

| 职责 | 说明 |
|---|---|
| 生命周期状态 | BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED |
| 输入 | 系统健康状态、定位状态、探索完成信号 |
| 输出 | `/mission_state`、`/exploration_enable` |

### 第 1 层 — ExplorationManager（`warehouse_exploration`）

Frontier 检测、目标选择、探索策略。不控制机器人。

| 模块 | 职责 |
|---|---|
| FrontierDetector | 寻找与未知区域相邻的空闲格子 |
| FrontierClusterer | BFS 连通分量聚类 |
| GoalSelector | 加权评分：距离 × 信息增益 × 面积 × 失败次数 |
| ReachabilityChecker | BFS 漫水填充；过滤不可达聚类 |
| GoalManager | 黑名单、失败计数、重试冷却 |
| ExplorationFSM | DETECT → SELECT → PLAN → FOLLOW → RECOVERY |
| CoverageMonitor | 后台 2Hz：地图覆盖率统计，独立于 FSM |

输入：`/map`（OccupancyGrid）、`/tf`（机器人位姿）
输出：`/exploration_goal`（PoseStamped）、`/frontier_marker`、`/exploration_state`

### 第 2 层 — NavigationManager（`warehouse_navigation`）

路径规划与执行编排。

| 模块 | 职责 |
|---|---|
| CostmapManager | 四层代价地图：Raw / Planner / Frontier / Dynamic |
| GlobalPlanner | A* 接口：地图 + 起点 + 目标 → 路径 |
| LocalPlanner | DWA 接口：路径 + 激光 + 里程计 → cmd_vel |
| PathTracker | Pure Pursuit：全局路径 → 预瞄子目标 |
| RecoveryManager | 后退、旋转、重规划 |
| NavigationMonitor | 卡住检测、震荡检测、超时、进度监控 |

输入：`/exploration_goal`、`/map`、`/odom`、`/scan`
输出：`/cmd_vel`、`/global_path`、`/local_path`、`/nav_status`

### 第 3 层 — Diagnostics（`warehouse_diagnostics`）

性能监控与日志。只读，永远不控制机器人。

| 模块 | 职责 |
|---|---|
| ResourceMonitor | CPU、tick 耗时、地图更新延迟 |
| CsvLogger | 结构化 CSV 导出，用于实验数据采集 |
| Visualization | RViz Marker 调试可视化 |

## 3. 外部依赖（不属于框架）

| 组件 | 角色 | 接口 |
|---|---|---|
| Cartographer / Hector / VINS | 建图 + 定位 | 发布 `/map`、`/tf`（map→odom） |
| Gazebo | 仿真 | 生成机器人、运行物理引擎 |
| Robot URDF/SDF | 机器人模型 | 传感器（激光、IMU、相机）、执行器 |

## 4. 不变量

1. 只有 MotionController 发布 `/cmd_vel`。
2. 任何模块不得修改 Cartographer、A* 或 DWA 的内部实现。
3. 所有模块间通信通过 ROS Topic（禁止跨层直接调用类）。
4. 所有参数来自 YAML 文件；零硬编码。
5. 每个模块都有对应的单元测试。
