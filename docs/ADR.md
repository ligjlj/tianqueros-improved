# 架构决策记录 — 仓库自主探索框架

## ADR-001：单一 cmd_vel 发布者

**状态：** 已采纳

**背景：** DWA、Recovery、Exploration 启动扫描、手动遥控等多个模块都需要控制机器人运动。没有单一出口时，cmd_vel 消息互相冲突，导致机器人震荡或卡住。

**决策：** 全系统有且仅有一个节点发布 `/cmd_vel`：**MotionController**。所有其他模块通过定义好的接口请求运动。

```
Recovery ──► MotionController.requestRecovery() ──► cmd_vel
Exploration ──► MotionController（通过 NavigationManager）
```

**后果：**
- Recovery 不是 Controller。它只请求恢复动作，MotionController 负责执行。
- 以后加入 TEB/MPC 不需要改 Recovery 或 Exploration。
- 手动遥控使用独立 Topic `/cmd_vel_teleop`，MotionController 内部做多路复用仲裁。

---

## ADR-002：多层代价地图

**状态：** 已采纳

**背景：** 不同模块需要不同的代价表示。全局统一膨胀会让 Frontier 检测误判（墙边自由格子看起来像障碍物），也让 A* 在窄通道中找不到路。

**决策：** 四层代价地图，每层从同一个 OccupancyGrid 独立生成。

```
OccupancyGrid
      │
      ▼
 RawCostmap（isFree / isOccupied / isUnknown）
      │
      ├──────────────┐
      ▼              ▼
PlannerCostmap   FrontierCostmap
(膨胀 0.3m)       (无膨胀，Raw)
      │              │
      └──────┬───────┘
             ▼
    DynamicCostmap（激光 + 实时障碍物）
```

| 层 | 使用者 | 生成方式 |
|---|---|---|
| Raw | ReachabilityChecker、FrontierDetector | OccupancyGrid 直接读取 |
| Planner | A*、GlobalPlanner | Raw + 机器人半径 + 安全余量膨胀 |
| Frontier | FrontierDetector、GoalSelector | Raw；Frontier 不能受膨胀影响 |
| Dynamic | DWA、LocalPlanner | Planner + 实时激光叠加 |

**后果：**
- Frontier 检测永远看到真实的自由/未知边界。
- A* 可以用激进膨胀而不遮盖 Frontier。
- DWA 获得实时障碍物数据，不依赖静态地图。

---

## ADR-003：事件驱动 FSM

**状态：** 已采纳

**背景：** 轮询式 FSM（`switch(state) { case X: doX(); }`）把状态逻辑和动作执行混在一起。当 Planner 阻塞或 Gazebo 卡顿时，整个 FSM 冻结。

**决策：** FSM 只管理状态转移和发出事件。动作执行放在独立的 Worker 中，完成后通过事件回报结果。

```
FSM: "NeedPlan" ────► Planner Worker ────► "PlanningSuccess" ────► FSM
                      （后台运行）
```

FSM 状态是纯粹的：IDLE、WAITING_FOR_PLAN、FOLLOWING_PATH 等。
FSM 不直接调用 `planner.plan()`。
FSM 不依赖 `ros::Timer`——由 `while(ros::ok()) { spinOnce; processEvents(); rate.sleep(); }` 驱动。

**事件类型：**
- MAP_READY、SCAN_COMPLETE、NEED_PLAN、PLANNING_SUCCESS、PLANNING_FAILED
- GOAL_REACHED、STUCK、RECOVERY_REQUESTED、RECOVERY_DONE
- EXPLORATION_DONE、ERROR

**后果：**
- Planner 可以是 Plugin，运行时替换。
- FSM 可以独立测试：喂事件，验证状态转移。
- Gazebo 卡顿不会让 FSM 主循环停止。

---

## ADR-004：Worker 模式——后台任务

**状态：** 已采纳

**背景：** 覆盖率计算、Frontier 可视化、CSV 日志、地图保存都是 IO 密集型或 CPU 密集型任务。放在 FSM tick 里会阻塞状态转移。

**决策：** 后台任务使用 Worker 模式 + 任务队列。

```cpp
class CoverageWorker {
    void enqueue(const OccupancyGrid& map);
    // 在独立线程中运行，以 1-2 Hz 发布 /coverage。
};
```

所有 Worker 共享线程池（后续可配置线程数）。

**规划的 Worker：**
- CoverageWorker：2 Hz 计算覆盖率
- FrontierVizWorker：2 Hz 生成 Frontier Marker
- LogWorker：异步刷写 CSV 缓冲
- MapSaveWorker：非阻塞地图保存

**后果：**
- FSM tick 时间恒常，不随地图大小增长。
- 加新的后台任务不需要动 FSM。

---

## ADR-005：Planner / Controller 插件架构

**状态：** 已采纳

**背景：** 把 A* 和 DWA 写死会让框架变成一次性 Demo。作为长期研究框架，规划器和控制器必须可替换。

**决策：** 定义抽象 Plugin 接口。实现通过 pluginlib 或手动 dlopen 在运行时加载。

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

通过 YAML 选择：
```yaml
global_planner: "AStarPlanner"   # 或 "RRTStar"、"HybridAStar"
local_planner:  "DWAPlanner"     # 或 "TEBPlanner"、"MPCPlanner"
```

**后果：**
- 加新规划器只需实现接口，不改其他代码。
- 规划器完全隔离测试（不需要 ROS，不需要 Gazebo）。
- NavigationManager 不知道具体运行的是哪个规划器。

---

## ADR-006：Blackboard 模式——共享状态

**状态：** 已采纳

**背景：** 模块之间需要共享状态（机器人位姿、当前目标、当前路径、任务状态），但直接跨模块调用造成紧耦合。

**决策：** 所有共享状态放在 Blackboard 中。模块读写 Blackboard，互不直接调用。

```cpp
class Blackboard {
public:
    // 线程安全读写。
    Pose getRobotPose();
    void setCurrentGoal(const Pose& g);
    Path getCurrentPath();
    void setNavigationStatus(NavStatus s);
};
```

```
MissionManager ──写──► Blackboard.current_goal
Planner        ──读──► Blackboard.current_goal
Planner        ──写──► Blackboard.current_path
Controller     ──读──► Blackboard.current_path
Recovery       ──读──► Blackboard.navigation_status
```

**后果：**
- 没有模块直接 import 另一个模块的头文件。
- 加新模块只需读写 Blackboard，不改已有代码。
- Blackboard 是唯一真相源；方便日志记录和回放调试。

---

## ADR-007：三大分离原则

**状态：** 已采纳

**原则 1 — Mission 不知道 Planner。**
Mission 设置 `Blackboard.current_goal` 并发出 `NEED_PLAN`。它从不调用 `planner.plan()` 或引用 A*/DWA 的名字。

**原则 2 — Planner 不知道 Frontier。**
Planner 接受 `(Costmap, Start, Goal) → Path`。它不选择目标、不检测 Frontier、不知道探索的存在。

**原则 3 — Controller 不知道 Map。**
Controller 接受 `(Costmap, Pose, Path) → Twist`。它不读取 OccupancyGrid、不知道 SLAM。

```
Mission ──► Goal ──► Planner ──► Path ──► Controller ──► cmd_vel
  │                                                    │
  └── 永远不引用 A*/DWA ───────────────────────────────┘
```

**后果：**
- Planner 可以替换，不碰 Mission 或 Exploration。
- Controller 适用于任何路径来源（A*、RRT*、手动）。
- 每个模块可以用 mock 输入独立测试。

---

## ADR-008：目录结构

**状态：** 已采纳

```
warehouse_nav/
├── mission/           — 生命周期 FSM、MissionManager
├── exploration/       — Frontier、Goal、Coverage
├── navigation/        — Planner 接口、Controller 接口、NavigationManager
├── costmap/           — Raw、Planner、Frontier、Dynamic 层
├── localization/      — CartographerAdapter、VINSAdapter、LocalizationManager
├── recovery/          — RecoveryManager + 策略
├── motion/            — MotionController（唯一 cmd_vel 发布者）
├── diagnostics/       — 性能、日志、统计
├── blackboard/        — 共享状态
├── bringup/           — Launch 文件、rviz 配置
├── interfaces/        — Plugin 头文件（planner、controller、recovery）
└── docs/              — ARCHITECTURE.md、INTERFACES.md、ROADMAP.md、ADR.md
```

每个目录是一个 ROS 包或包内逻辑模块。依赖仅从上到下单向流动：`bringup → mission/exploration/navigation → costmap/blackboard/interfaces`。

---

## 决策记录

| ADR | 日期 | 状态 | 标题 |
|---|---|---|---|
| 001 | 2026-07-05 | 已采纳 | 单一 cmd_vel 发布者 |
| 002 | 2026-07-05 | 已采纳 | 多层代价地图 |
| 003 | 2026-07-05 | 已采纳 | 事件驱动 FSM |
| 004 | 2026-07-05 | 已采纳 | Worker 模式 |
| 005 | 2026-07-05 | 已采纳 | 插件架构 |
| 006 | 2026-07-05 | 已采纳 | Blackboard 模式 |
| 007 | 2026-07-05 | 已采纳 | 三大分离原则 |
| 008 | 2026-07-05 | 已采纳 | 目录结构 |
