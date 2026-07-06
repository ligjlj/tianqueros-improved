# 仓库自主探索框架 — 开发路线图

> 每天开发流程：打开本文件 → 选 Task → 编码 → 测试 → Commit → 打勾

## 测试体系

```
Level 1: Unit Test      — gtest, 每个类独立测
Level 2: Integration    — 多个类联调
Level 3: System Test    — Gazebo Demo
Regression:             — docs/REGRESSION.md 8项
```

---

## DONE ✅

### Phase 1: 框架搭建
| Risk: ★★★ | 自研 Mission + Exploration + Motion |
|---|---|
| ✅ MissionManager — 生命周期 FSM | |
| ✅ ExplorationFSM — Frontier + Goal + Coverage | |
| ✅ MotionController — 单一 /cmd_vel | |
| ✅ EventBus — 事件驱动 | |
| ✅ CoverageMonitor — 后台 2Hz | |
| ✅ GoalManager — 黑名单 + 状态机 | |

### Phase 2: ROS Navigation 接入
| Risk: ★★★★ | move_base Adapter 替换自研算法 |
|---|---|
| ✅ MoveBaseAdapter — 封装 action client | |
| ✅ NavigationManager — 双轨切换 | |
| ✅ costmap_2d 配置 (global + local) | |
| ✅ DWA + navfn 参数调优 | |
| ✅ transform_tolerance 修复 | |
| 🔄 TF 外推容差测试 | |

### Phase 3: 定位切换
| Risk: ★★ | Cartographer / VINS / GT |
|---|---|
| ✅ URDF 加相机 + IMU | |
| ✅ VINS topic relay 就绪 | |

---

## TODO ⬜

### Demo 稳定性
| Risk: ★★★★★ |
|---|
| ⬜ sim_time → WallTime 全部替换 |
| ⬜ INITIAL_SCAN 旋转确认 |
| ⬜ Exploration 端到端测试 |
| ⬜ Recovery 触发验证 |

### 实验数据 (Week 5)
| Risk: ★★ |
|---|
| ⬜ 10 次完整探索 |
| ⬜ Coverage vs Time 图表 |
| ⬜ Planner 对比 (navfn vs A*) |
| ⬜ Controller 对比 (DWA vs 自研) |
| ⬜ Recovery 对比 (ROS vs 自研) |
| ⬜ 论文级图表 |

---

## 每周 Exit Criteria

### Week 1 ✅ — 框架搭建
- [x] Mission FSM + EventBus + MotionController
- [x] 58+ tests passing

### Week 2 ✅ — 导航栈
- [x] Costmap + Interfaces + NavigationManager
- [x] RecoveryManager + CoverageMonitor

### Week 3 ✅ — 自主探索
- [x] Frontier + GoalSelector + GoalManager
- [x] 72 tests, 14-node Demo

### Week 4 🔄 — 算法替换
- [x] ROS Navigation Stack 接入
- [ ] Demo 端到端稳定运行

### Week 5 ⬜ — 实验
- [ ] 数据采集 + 论文图表
