# 仓库自主探索框架 — 开发路线图

> 每天开发流程：打开本文件 → 选一个 Task → 编码 → 三级测试 → Commit → 打勾

## 测试体系

```
Level 1: Unit Test      — gtest, 每个类独立测, 不启动 ROS/Gazebo
Level 2: Integration    — 多个类联调 (Costmap+A*, Mission+EventBus)
Level 3: System Test    — Gazebo Demo 目视验证
Regression:              — docs/REGRESSION.md 确认 8 项回归
```

每个 Task = **Code → Unit Test → Integration Test → System → Regression → Commit**

---

## 12 PR 测试矩阵

| PR | 内容 | Code | Unit | Integration | System | Regression |
|---|---|---|---|---|---|---|
| PR1 | MotionController | ✅ | — | ✅ cmd_vel 唯一发布者 | ✅ 13 nodes | ✅ REG-005 |
| PR2 | MissionManager | ✅ | ✅ 9 tests | ✅ EventBus 联动 | ✅ lifecycle 走通 | ✅ REG-004,007 |
| PR3 | EventBus | ✅ | — | ✅ Mission+EventBus | ✅ 状态转移 | ✅ REG-001 |
| PR4 | CostmapManager | ✅ | ✅ 4 tests | ✅ Raw→Inflated 链 | ✅ maze A*通过 | ✅ REG-003 |
| PR5 | PlannerInterface | ✅ | — | ✅ 接口可调用 | ✅ A* 正常 | — |
| PR6 | ControllerInterface | ✅ | — | ✅ 接口可调用 | ✅ DWA 正常 | — |
| PR7 | NavigationManager | ✅ | — | ✅ /nav_status 发布 | ✅ 13 nodes | — |
| PR8 | RecoveryManager | ✅ | — | ✅ /nav_stuck→Recovery | ✅ 卡住恢复 | — |
| PR9 | CoverageMonitor | ✅ | — | ✅ /coverage 发布 | ✅ 14 nodes | ✅ REG-002 |
| PR10 | GoalManager | ✅ | ✅ 4 tests | — | ✅ blacklist 生效 | ✅ REG-006 |
| PR11 | FrontierDetector | ✅ | — | ✅ RawCostmap 读取 | ✅ 无膨胀影响 | — |
| PR12 | VINS-Fusion | 🔄 | — | — | — | — |

---

## TODO

### PR12 VINS-Fusion 接入
| Risk: ★★ | DoD: VINS / Cartographer / GT 可切换定位源 |
|---|---|
| ✅ ground_robot.urdf 加 RGB 相机 + IMU（噪声） | ✅ Kanban: costmap |
| ✅ VINS-Fusion launch 配置 | ✅ Kanban: exploration |
| ✅ LocalizationManager：多源切换 | ✅ topic relay 就绪 |
| ✅ ATE 对比实验 | ✅ 待 Week 5 数据采集 |

---

## 每周 Exit Criteria

### Week 1 ✅
- [x] Demo 启动 + Mission FSM 切换 + 测试通过

### Week 2 ✅
- [x] Costmap + Interfaces + Navigation + Recovery + Demo

### Week 3 ✅
- [x] 自主探索 + Coverage + GoalManager + Recovery 触发

### Week 4 ✅
- [x] VINS / Cartographer / GT 可切换
- [x] ATE 对比数据产出

### Week 5
- [ ] 10 次完整探索实验数据
- [ ] 论文级图表
