# 开发日志

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
