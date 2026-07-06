# 仓库自主探索框架 — 开发路线图

## 第一周：系统重构

### 1.1 包结构
- [x] 创建 `warehouse_mission` 包
- [x] 创建 `warehouse_motion` 包（MotionController）
- [ ] 创建 `warehouse_navigation` 包
- [ ] 创建 `warehouse_diagnostics` 包
- [ ] 将现有包迁移到新结构
- [ ] 更新所有 CMakeLists 的包间依赖

### 1.2 接口定义
- [x] 定义所有 Topic 名称和消息类型（参照 INTERFACES.md）
- [ ] 按需创建 `warehouse_msgs` 自定义消息
- [x] 文档化每个 Topic 的发布者、订阅者和频率

### 1.3 MissionManager
- [x] 实现 FSM：BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED
- [x] 发布 `/mission_state` 和 `/exploration_enable`
- [x] 主循环 `while(ros::ok()) { spinOnce; fsm.update(); rate.sleep(); }`
- [x] 基于系统时钟（不依赖 sim_time）
- [ ] 状态转移单元测试

### 1.4 事件机制
- [ ] 定义事件类型：MAP_READY、SCAN_COMPLETE、EXPLORATION_DONE、ERROR
- [ ] 事件总线：进程内发布/订阅（不走 ROS Topic，零开销）
- [ ] FSM 由事件驱动转移，不轮询

### 1.5 诊断模块
- [ ] 实现 ResourceMonitor：CPU、tick 耗时、地图更新延迟
- [ ] 在 `warehouse_diagnostics` 中实现 CsvLogger
- [ ] 自适应频率：CPU > 80% 时降速
- [ ] 全模块输出 CSV 用于实验数据采集

**第一周目标：** 系统能启动、进入各生命周期状态，rviz 显示状态转移。暂不探索。

---

## 第二周：导航重构

### 2.1 CostmapManager
- [ ] 四层代价地图：Raw / Inflated (0.3m) / Frontier / Dynamic (laser)
- [ ] `getCost(layer, x, y)` 统一接口
- [ ] ReachabilityChecker 使用 Raw 层
- [ ] A* 使用 Planner 层
- [ ] DWA 使用 Dynamic 层
- [ ] 每层单元测试

### 2.2 NavigationManager
- [ ] 编排：接收目标 → 规划 → 执行 → 报告
- [ ] 不发布 cmd_vel（委托给 LocalPlanner）
- [ ] 发布 `/nav_status`（IDLE / PLANNING / FOLLOWING / STUCK / RECOVERY）

### 2.3 RecoveryManager
- [ ] 恢复策略：BACKUP、ROTATE、REPLAN、SKIP_GOAL
- [ ] 由 NavigationMonitor 触发
- [ ] Recovery 只在活跃恢复阶段发布 cmd_vel
- [ ] 恢复转移单元测试

### 2.4 Planner 接口
- [ ] 抽象 `GlobalPlanner` 基类
- [ ] A* 实现遵循接口
- [ ] `plan(map, start, goal) → Path`
- [ ] 以后可加 RRT* 而不改 NavigationManager

### 2.5 Controller 接口
- [ ] 抽象 `LocalPlanner` 基类
- [ ] DWA 实现遵循接口
- [ ] `computeVelocity(map, state, path) → (v, w)`
- [ ] 以后可加 TEB/MPC

**第二周目标：** 机器人能可靠地从 A 点导航到 B 点（rviz "2D Nav Goal"）。

---

## 第三周：自主探索

### 3.1 FrontierDetector
- [ ] O(N) 扫描 OccupancyGrid，找自由格子邻接未知
- [ ] 输出：Frontier GridCell 列表
- [ ] 单元测试：空地图、全地图、边界情况

### 3.2 FrontierClusterer
- [ ] BFS 连通分量聚类（8 邻域）
- [ ] 输出：聚类（含中心、大小、包围盒）
- [ ] 单元测试：单聚类、多聚类、小聚类

### 3.3 GoalManager
- [ ] 目标状态跟踪：未尝试 → 已尝试 → 失败 → 黑名单
- [ ] 黑名单 60s 冷却，自动过期
- [ ] 就近去重（0.5m 半径）
- [ ] 单元测试：黑名单、过期、去重

### 3.4 ExplorationFSM
- [ ] 状态：DETECT → SELECT → PLAN → FOLLOW → RECOVERY → UPDATE
- [ ] 事件驱动转移（不轮询）
- [ ] 可达性过滤：只评分与机器人 BFS 连通的聚类
- [ ] GoalSelector：四维加权评分 + 失败次数惩罚
- [ ] 单元测试：状态转移、超时、重试

### 3.5 CoverageMonitor
- [ ] 后台 2Hz 线程，独立于 FSM
- [ ] 订阅 `/map`，计算覆盖率百分比
- [ ] 发布 `/coverage`（Float32）
- [ ] 覆盖率停滞时触发 FINISHED 信号

**第三周目标：** 机器人无需人工干预，自主探索并完成迷宫建图。

---

## 第四周：视觉惯性融合

### 4.1 Gazebo 传感器
- [ ] 给 ground_robot.urdf 加 RGB 相机
- [ ] 给 ground_robot.urdf 加 IMU（含噪声模型）
- [ ] 验证相机发布 `/camera/image_raw`
- [ ] 验证 IMU 发布 `/imu`

### 4.2 IMU 噪声模型
- [ ] 配置真实噪声参数（陀螺仪、加速度计偏置）
- [ ] 验证噪声在 `/imu` 数据中可见

### 4.3 VINS-Fusion 接入
- [ ] 用相机 + IMU Topic 启动 VINS-Fusion
- [ ] 验证 VINS 发布 `/vins_estimator/odometry`
- [ ] 对比 VINS 轨迹 vs 真值 P3D

### 4.4 LocalizationManager
- [ ] 支持多定位源：Cartographer、VINS、Ground Truth
- [ ] 发布统一 `/localization_pose`
- [ ] 通过参数或 Service 运行时切换
- [ ] 切换单元测试

### 4.5 定位对比
- [ ] 同一路径采集 GT、Cartographer、VINS 轨迹
- [ ] 计算各方法 ATE（绝对轨迹误差）
- [ ] 生成对比图

**第四周目标：** 定量对比不同定位方法对建图和探索的影响。

---

## 第五周：优化与实验

### 5.1 Frontier 评分优化
- [ ] 调参：距离、信息、面积、失败次数权重
- [ ] A/B 测试不同评分策略
- [ ] 指标：总探索时间、覆盖率增速、失败率

### 5.2 Recovery 策略优化
- [ ] 对比恢复策略：纯后退、纯旋转、组合
- [ ] 指标：恢复成功率和时间开销
- [ ] 优化恢复参数（后退距离、旋转角度）

### 5.3 参数调优
- [ ] 网格搜索/手动调优：inflation_radius、goal_tolerance、超时时间
- [ ] DWA 参数：v_samples、w_samples、alpha/beta/gamma
- [ ] 探索参数：min_cluster_size、blacklist_duration

### 5.4 实验数据采集
- [ ] 每种配置跑 10 次完整探索
- [ ] 记录：coverage_vs_time、path_length、planning_time、recovery_count
- [ ] 导出 CSV 到 `~/catkin_ws/experiments/`

### 5.5 论文图表
- [ ] 覆盖率随时间变化图（对比不同配置）
- [ ] 轨迹叠加在地图上的俯视图
- [ ] 规划时间直方图
- [ ] Recovery 事件时间线
- [ ] 定位误差对比图（Cartographer vs VINS vs GT）

**第五周目标：** 完整实验数据集 + 发表级图表。
