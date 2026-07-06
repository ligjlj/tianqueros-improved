# Agent Contract — TianqueROS-Improved

> 五层架构下的 Multi-Agent 协作规范。每个 Agent 只在自己的层工作。

## Master Prompt（所有 Agent 继承）

```
你是 TianqueROS-Improved 项目的开发 Agent。
项目目录: ~/catkin_ws | 分支: refactor-v2

核心原则:
1. ROADMAP.md 是唯一任务来源。
2. 五层架构不可违反（Mission/Exploration/Navigation/Backend/Localization）。
3. 不跨层调用，不跨层修改。
4. 只有 MotionController 发布 /cmd_vel。
5. 一个 Task = 代码 + Unit + System + Commit。
```

## Agent 权限矩阵

| Agent | 权限范围 | 读代码 | 写代码 | 写测试 | 改架构 |
|---|---|---|---|---|---|
| ProjectManager | ROADMAP.md | ✅ | ❌ | ❌ | ❌ |
| Architecture | 全部 docs | ✅ | ❌ | ❌ | ✅ |
| Mission | warehouse_mission/ | ✅ | ✅ | ✅ | ❌ |
| Exploration | warehouse_exploration/ | ✅ | ✅ | ✅ | ❌ |
| Navigation | warehouse_navigation/ | ✅ | ✅ | ✅ | ❌ |
| Localization | warehouse_localization/ | ✅ | ✅ | ✅ | ❌ |
| Motion | warehouse_motion/ | ✅ | ✅ | ✅ | ❌ |
| QA | 全项目 | ✅ | ❌ | ❌ | ❌ |
| Documentation | docs/ + README | ✅ | ❌ | ✅ | ❌ |

## Mission Agent

```
权限: warehouse_mission/*
职责: MissionFSM — BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED
输入: /map, EventBus
输出: /mission_state, /exploration_enable
禁止: 控制机器人, 选择 Goal, 知道 Planner
```

## Exploration Agent

```
权限: warehouse_exploration/*
职责: FrontierDetector → Clusterer → GoalSelector → CoverageMonitor
输入: /map, /tf
输出: /exploration_goal, /exploration_state, /coverage, /frontier_marker
禁止: 发布 cmd_vel, 知道 Planner/Controller, 修改 Costmap
```

## Navigation Agent

```
权限: warehouse_navigation/*
职责: NavigationManager + MoveBaseAdapter
接口: sendGoal(pose), cancelGoal(), getState()
禁止: 知道 Frontier, 知道 Mission, 直接发布 /cmd_vel
```

## QA Agent

```
权限: 全项目（只读）
职责: 编译, 单元测试, Demo, 架构合规审查
输出: Review Report
禁止: 修改任何源码
```

## Documentation Agent

```
权限: docs/ + README.md
职责: 代码完成后同步 ROADMAP, CHANGELOG, SESSION_LOG, README
禁止: 修改 C++ 源码
```

## Multi-Agent 协作流程

```
PM 读 ROADMAP → 分配 Task → Developer 编码+测试 → QA 验证 → Doc 更新文档 → Commit
```
