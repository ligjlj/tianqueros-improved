# 仓库自主探索框架 — 开发日志

## 时间线

```
Phase 0-2  (7/4)     GridMap + A* + DWA           ✅ 58 tests
Phase 3-5  (7/4)     PathTracker + FSM v1         ✅
Phase 6-8  (7/5 AM)  Exploration + Demo           ⚠️ demo 不稳定
Debug      (7/5 AM)  根因分析 + 修复              4个工程问题
Design     (7/5 PM)  架构讨论 → ADR               8条决策
PR1        (7/5 PM)  MotionController             ✅ 完成
PR2        (7/6 AM)  MissionManager               ✅ 完成
PR3        (7/6 AM)  EventBus                     ✅ 完成
PR4        (7/6 AM)  CostmapManager               ✅ 完成
PR5        (7/6 AM)  PlannerInterface             ✅ 完成
PR6        (7/6 AM)  ControllerInterface          ✅ 完成
PR7        (7/6 PM)  NavigationManager            ✅ 完成
PR8        (7/6 PM)  RecoveryManager              ✅ 完成
PR9        (7/6 PM)  CoverageMonitor              ✅ 完成
PR10       (7/6 PM)  GoalManager 重构             ✅ 完成
PR11       (7/6 PM)  FrontierDetector 重构        ✅ 完成
PR12       (7/6 PM)  VINS-Fusion 接入             ✅ 完成
```

## 分支状态

```
main          — 基线 (58 tests, demo runnable)
refactor-v2   — 重构分支 (12-PR 全部完成)
```

## 调试记录 (7/5 AM)

Demo 跑不起来，发现 4 个根因：

| # | 症状 | 根因 | 修复 |
|---|---|---|---|
| 1 | FSM 永远卡在同一个状态 | `ros::Timer` 依赖 `/clock`；Gazebo 加载迷宫导致时钟停顿 | 改用 `ros::WallTimer` |
| 2 | CPU 跑满，FSM 无响应 | Coverage O(2048²) 每 0.1s tick 计算一次 | 缓存覆盖度，每 2s 算一次 |
| 3 | A* 永远找不到路径 | `inflation_radius=1.0m`（20 格）堵死所有窄通道 | 减小到 0.3m |
| 4 | INITIAL_SCAN 后机器人一直转 | 只发一次 stop，diff_drive 保持上一帧指令 | 退出后持续发 stop 1 秒 |

## 架构决策 (7/5 PM)

| ADR | 决策 |
|---|---|
| 001 | 单一 `/cmd_vel` 发布者：MotionController |
| 002 | 四层 Costmap：Raw → Planner / Frontier → Dynamic |
| 003 | 事件驱动 FSM：发事件，不调规划器 |
| 004 | Worker 模式：后台任务进线程池 |
| 005 | Plugin 接口：A*/DWA 通过 YAML 切换 |
| 006 | Blackboard 模式：共享状态，模块间不 import |
| 007 | 三大分离：Mission ⊥ Planner ⊥ Frontier ⊥ Controller ⊥ Map |
| 008 | 目录：`warehouse_nav/` 下 11 个子模块 |

## 12-PR 绞杀者重构计划

| PR | 内容 | 状态 |
|---|---|---|
| PR1 | MotionController 接管 `/cmd_vel` | ✅ |
| PR2 | MissionManager 接管启动流程 | ✅ |
| PR3 | EventBus / Event 定义 | ✅ |
| PR4 | CostmapManager | ✅ |
| PR5 | PlannerInterface | ✅ |
| PR6 | ControllerInterface | ✅ |
| PR7 | NavigationManager | ✅ |
| PR8 | RecoveryManager | ✅ |
| PR9 | CoverageMonitor 后台化 | ✅ |
| PR10 | GoalManager 重构 | ✅ |
| PR11 | FrontierDetector 重构 | ✅ |
| PR12 | 接入 VINS-Fusion | ✅ |

## 开发原则

1. **只用绞杀者模式。** 新层包裹旧代码，永远不重写。
2. **每个 PR 后 Demo 必须能跑。** 坏了就回滚。
3. **一次只改一件事。** 不边重构边优化算法。
4. **main 分支永不动。** 所有工作在 refactor-v2。
5. **机器人永远通过 MotionController 运动。** 没有其他 cmd_vel 发布者。
6. **每次 PR 提交前逐项核对 docs/：** ARCHITECTURE（层级正确）、INTERFACES（Topic 无冲突）、ADR（决策合规）、ROADMAP（勾掉已完成项）。

## 每日开发流程

```
打开 ROADMAP → 选一个 Task（不是 PR）→ 编码 → 单元测试
→ Demo验证 → Commit → 勾掉 Task → 更新 SESSION_LOG
```

**原子单位是 Task，不是 PR。** PR 是一组 Task 全部完成后自然形成的 Git 提交单位。

**开发时只看一个文档：ROADMAP。**
- README：给别人看，不指导开发
- ARCHITECTURE：架构调整时才看
- INTERFACES：写通信代码时查
- ROADMAP：每天开发唯一入口
- SESSION_LOG：每天结束时记录
