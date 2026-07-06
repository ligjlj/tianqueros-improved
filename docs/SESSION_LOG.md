# 开发日志

## 2026-07-06 — Demo 验证 + wall_h1 删除

**Demo 运行验证:**
- 14 节点全部在线，MissionFSM 全链路走通: BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION（3.2s）
- INITIAL_SCAN 偏航角验证正常：3s 完成 360° 旋转
- /cmd_vel 唯一发布者 = motion_controller（ADR-001 ✓）
- hector_mapping 正常发布 /map

**已知问题:**
- move_base TF extrapolation 错误（sim_time + hector 的 odom→map TF 同步问题）
- 机器人静止时 move_base 无法规划路径

**修复:**
- warehouse.world: 删除 wall_h1（y=-3 横墙），机器人 spawn 位置向南畅通

## 2026-07-06 — v0.8: 稳定性提升 (sim_time→WallTime, INITIAL_SCAN 旋转确认)

**稳定性提升 (4项全部完成):**
1. sim_time → WallTime 全部模块替换:
   - exploration_fsm.hpp/cpp: 所有 ros::Time → ros::WallTime（NavigationMonitor, MapProgressMonitor, FSM 成员变量）
   - dwa_controller_node.cpp: ros::Timer → ros::WallTimer
   - mission_fsm.hpp/cpp 已经在用 WallTime
2. move_base TF transform_tolerance: 2.0 → 0.3
3. INITIAL_SCAN 旋转确认: MissionFSM 现在基于实际 odom 偏航角累积（≥315°）+ 最短 8s，14s 安全超时
4. Exploration 端到端自动化测试脚本: tests/exploration_e2e.py

**额外修复:**
- test_goal.cpp 预存问题: weight_reachability 不存在 → 移除; min_goal_distance_m=2.0 在小网格跳过全部目标 → 测试中设为 0.0
- 单元测试: 37/37 全部通过（warehouse_utils 19 + warehouse_exploration 18）

**编译验证:** warehouse_mission, warehouse_exploration, warehouse_controller 三包零 error

## 2026-07-06 — v0.7: 五层架构重构

**架构决策：** 从"自研所有算法"转变为"构建自主探索框架"。算法是插件，不是核心。

**五层：**
1. Mission Layer — 要不要探索
2. Exploration Layer — 去哪
3. Navigation Layer — 怎么导航（Adapter 模式）
4. Navigation Backend — ROS Navigation Stack
5. Localization Layer — 在哪

**关键决策：**
- ADR-009: 算法层使用 ROS Navigation Stack（navfn + DWA + costmap_2d）
- 自研 A*/DWA 保留双轨，config 切换
- MotionController 保留为速度仲裁器
- EventBus 保留为跨模块通信
- Recovery 委托给 move_base，Mission 处理失败

## 2026-07-06 — v0.6: ROS Navigation 接入
- MoveBaseAdapter 封装 action client
- NavigationManager 双轨运行
- move_base + navfn + DWA + costmap_2d 配置完成
- Demo 14节点运行，move_base 产出规划

## 2026-07-06 — v0.5: PR12 + 三级测试
- Kanban Multi-Agent 验证：4/4 tasks
- 三级测试体系：Unit(72) + Integration + System + Regression(8)
- URDF 相机+IMU

## 2026-07-06 — v0.0~v0.4
- v0.4: Costmap + Interfaces + Navigation + Recovery
- v0.3: EventBus + MissionFSM
- v0.2: MissionManager
- v0.1: MotionController
- v0.0: Baseline (GridMap + A* + DWA, 58 tests)

## 调试记录

### sim_time 问题（贯穿始终）
- ros::Timer 在 sim_time 下不稳定 → 全部改为 WallTimer/WallTime
- MotionController 未改 WallTimer → 修复后 /cmd_vel 链路正常
- rostopic echo 在 sim_time 下超时 → 改用直接日志观察

### 其他修复
- coverage 每 tick 计算 O(2048²) → 缓存每 2s 一次
- inflation_radius=1.0m 堵死迷宫 → 减为 0.3m
- warehouse.world SDF 缺少 name 属性 → 修复后 Gazebo 正常渲染
- ReachabilityChecker BFS 过严 → 暂时跳过，A* 自行验证
- GoalSelector 重复选紧邻目标 → 加 min_goal_distance=2.0m
