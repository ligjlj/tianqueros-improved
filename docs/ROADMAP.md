# 仓库自主探索框架 — 开发路线图（看板）

> 每天开发流程：打开本文件 → 完成一个 Task → 测试 → Demo → Commit → 打勾 → 结束

## DONE ✅

### PR1 MotionController
| Risk: ★★ | DoD: `/cmd_vel` 唯一发布者为 motion_controller |
|---|---|
| ✅ 创建 warehouse_motion 包 | |
| ✅ 优先级复用：recovery > nav > stop | |
| ✅ Demo launch remap 旧节点到 /cmd_vel_nav, /cmd_vel_recovery | |
| ✅ ADR-001 落地 | |

### PR2 MissionManager
| Risk: ★★★ | DoD: BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION→FINISHED 全链路走通 |
|---|---|
| ✅ 创建 warehouse_mission 包 | |
| ✅ MissionFSM 类（可测试，回调注入） | |
| ✅ mission_node（薄 ROS wrapper） | |
| ✅ 9 项状态转移单元测试 | |
| ✅ while(ros::ok()) + spinOnce + rate.sleep() 主循环 | |
| ✅ WallTime 时钟（不依赖 sim_time） | |

### PR3 EventBus
| Risk: ★★★ | DoD: 事件类型完整定义，MissionManager 由事件驱动 |
|---|---|
| ✅ EventBus 类（线程安全，进程内 pub/sub） | |
| ✅ 12 种事件类型 | |
| ✅ MissionManager mapCb 发送 MAP_READY 事件 | |
| ✅ FSM 转移由事件触发，不轮询 | |

---

## TODO ⬜

### PR4 CostmapManager
| Risk: ★★★★★ | DoD: 四层 costmap 可独立查询，不共享全局膨胀 |
|---|---|
| ⬜ Day1: RawCostmap — OccupancyGrid → isFree/isOccupied/isUnknown | |
| ⬜ Day1: PlannerCostmap — Raw + 0.3m 膨胀 | |
| ⬜ Day2: FrontierCostmap — Raw，无膨胀（Frontier 专用） | |
| ⬜ Day2: DynamicCostmap — Planner + 实时激光叠加 | |
| ⬜ Day3: CostmapManager 统一接口 getCost(layer, x, y) | |
| ⬜ Day3: 各层单元测试 | |
| ⬜ Day3: 替换 ReachabilityChecker 使用 RawCostmap | |
| ⬜ Day3: 替换 A* 使用 PlannerCostmap | |

### PR5 PlannerInterface
| Risk: ★★★★ | DoD: A* 通过抽象接口调用，YAML 可切换实现 |
|---|---|
| ⬜ 定义 GlobalPlannerPlugin 抽象类 | |
| ⬜ AStarPlanner 实现接口 | |
| ⬜ YAML 参数 global_planner: "AStarPlanner" | |
| ⬜ NavigationManager 通过接口调用（不直接 new AStarPlanner） | |
| ⬜ 接口单元测试（mock planner） | |

### PR6 ControllerInterface
| Risk: ★★★★ | DoD: DWA 通过抽象接口调用，YAML 可切换 |
|---|---|
| ⬜ 定义 LocalPlannerPlugin 抽象类 | |
| ⬜ DWAPlanner 实现接口 | |
| ⬜ YAML 参数 local_planner: "DWAPlanner" | |
| ⬜ MotionController 通过接口调用 DWA | |
| ⬜ 接口单元测试 | |

### PR7 NavigationManager
| Risk: ★★★★★ | DoD: 接收 Goal → 调 Planner → 跟踪 Path → 报告状态 |
|---|---|
| ⬜ 编排：Goal → plan() → followPath() → nav_status | |
| ⬜ 发布 /nav_status（IDLE/PLANNING/FOLLOWING/STUCK/RECOVERY） | |
| ⬜ 不直接发布 cmd_vel（委托 LocalPlanner → MotionController） | |

### PR8 RecoveryManager
| Risk: ★★★ | DoD: 卡住自动后退+旋转，不影响正常导航 |
|---|---|
| ⬜ 恢复策略：BACKUP、ROTATE、SKIP_GOAL | |
| ⬜ 由 NavigationMonitor 触发 | |
| ⬜ Recovery 通过 MotionController 请求运动（不直接发 cmd_vel） | |
| ⬜ 恢复转移单元测试 | |

### PR9 CoverageMonitor
| Risk: ★★ | DoD: 后台 2Hz 统计覆盖率，独立线程 |
|---|---|
| ⬜ CoverageWorker 类（Job Queue 模式） | |
| ⬜ 发布 /coverage (Float32) | |
| ⬜ 不再在 FSM tick 里算覆盖率 | |

### PR10 GoalManager 重构
| Risk: ★★★ | DoD: 黑名单自动过期，评分包含失败次数 |
|---|---|
| ⬜ Goal 状态：UNTRIED→TRIED→FAILED→BLACKLISTED | |
| ⬜ 黑名单 60s 自动过期 | |
| ⬜ 单元测试：黑名单、过期、去重 | |

### PR11 FrontierDetector 重构
| Risk: ★★★ | DoD: Frontier 使用 FrontierCostmap（无膨胀） |
|---|---|
| ⬜ FrontierDetector 读 FrontierCostmap | |
| ⬜ 不再受全局膨胀影响 | |
| ⬜ 单元测试更新 | |

### PR12 VINS-Fusion 接入
| Risk: ★★ | DoD: VINS / Cartographer / GT 可切换定位源 |
|---|---|
| ⬜ ground_robot.urdf 加 RGB 相机 + IMU（噪声） | |
| ⬜ LocalizationManager：多源切换 | |
| ⬜ ATE 对比实验 | |

---

## 每周 Exit Criteria

### Week 1（已完成 ✅）
- [x] Demo 能启动
- [x] Mission FSM 正常切换
- [x] 无模块直接 publish `/cmd_vel`（全部通过 MotionController）
- [x] EventBus 工作
- [x] 所有测试通过

### Week 2
- [ ] CostmapManager 四层可独立查询
- [ ] Planner/Controller 通过抽象接口调用
- [ ] 机器人能从 A 点可靠导航到 B 点

### Week 3
- [ ] 机器人自主完成迷宫探索建图
- [ ] Recovery 在卡住时自动触发
- [ ] Coverage 后台独立统计

### Week 4
- [ ] VINS / Cartographer / GT 可切换
- [ ] ATE 对比数据产出

### Week 5
- [ ] 10 次完整探索实验数据
- [ ] 论文级图表
