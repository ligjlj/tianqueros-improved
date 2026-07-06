# 仓库自主探索框架 — 开发路线图

> 每天开发流程：打开本文件 → 选 Task → 编码 → 测试 → Commit → 打勾

## 当前架构

```
Mission → Exploration → Navigation → Adapter → ROS Navigation Stack
                                           └→ 自研 A*/DWA (双轨保留)
```

## DONE ✅

### Phase 1: 框架核心
- [x] MissionManager — 生命周期 FSM
- [x] ExplorationFSM — Frontier + GoalSelector + CoverageMonitor
- [x] MotionController — 单一 /cmd_vel 出口
- [x] EventBus — 跨模块事件
- [x] GoalManager — 黑名单 + 状态机

### Phase 2: 算法接入
- [x] MoveBaseAdapter — ROS Navigation 封装
- [x] NavigationManager — 双轨 (move_base / self)
- [x] costmap_2d + navfn + DWA 配置
- [x] ADR-009 — 算法复用决策

### Phase 3: 测试体系
- [x] Level 1: 72 Unit Tests
- [x] Level 2: 14-node Demo
- [x] Level 3: REG-001~008 回归检查

---

## TODO ⬜

### 稳定性提升
- [x] sim_time → WallTime 全部模块替换
- [x] move_base TF transform_tolerance 调优
- [x] INITIAL_SCAN 旋转确认
- [x] Exploration 端到端自动化测试

### 实验数据采集 (Week 5)
- [ ] 10 次完整探索运行
- [ ] Coverage vs Time 图
- [ ] Planner 对比 (navfn vs 自研 A*)
- [ ] Controller 对比 (ROS DWA vs 自研 DWA)
- [ ] Localization 对比 (GT vs Cartographer)
- [ ] Recovery 对比 (ROS vs 自研)

---

## 每周 Exit Criteria

### Week 1 ✅ 框架搭建
- Mission FSM + EventBus + MotionController
- GridMap + A* + DWA + Frontier (58 tests)

### Week 2 ✅ 导航栈
- Costmap + Interfaces + NavigationManager
- RecoveryManager + CoverageMonitor

### Week 3 ✅ 自主探索
- Frontier + GoalSelector + GoalManager
- 72 tests, 14-node Demo

### Week 4 ✅ 算法迁移 + 稳定性
- ROS Navigation Stack 接入
- Demo 端到端稳定性

### Week 5 ⬜ 实验
- 10次探索 + 论文图表
