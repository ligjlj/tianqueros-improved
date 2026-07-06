# Regression Tests — TianqueROS-Improved

> 每次重构后必须确认以下问题没有重新出现。

| 编号 | 问题 | 验证方式 | 状态 |
|---|---|---|---|
| REG-001 | FSM 不受 `/clock` 卡顿影响 | `ros::WallTimer` 替代 `ros::Timer`；Gazebo 暂停/恢复后 FSM 继续 | ✅ v0.3 |
| REG-002 | Coverage 不阻塞导航 | CoverageMonitor 独立线程 2Hz；FSM tick 不再计算覆盖率 | ✅ v0.4 |
| REG-003 | A* 能通过标准迷宫 | `inflation_radius=0.3m`；maze.world 中至少产出 1 条有效路径 | ✅ v0.2 |
| REG-004 | INITIAL_SCAN 后机器人停止 | 扫描结束后 MissionManager 发 stop cmd；1s 内 angular.z=0 | ✅ v0.3 |
| REG-005 | 只有 MotionController 发布 `/cmd_vel` | `rostopic info /cmd_vel` 显示唯一发布者 = motion_controller | ✅ v0.1 |
| REG-006 | GoalManager 黑名单 3 次失败后生效 | `markFailed × 3 → isBlacklisted() = true`；60s 后自动过期 | ✅ v0.4 |
| REG-007 | MissionManager 状态转移正确 | BOOT→WAIT_MAP→INITIAL_SCAN→EXPLORATION→FINISHED 全链路走通 | ✅ v0.3 |
| REG-008 | 全包编译零 error | `catkin_make` 10 packages clean | ⬜ 每次 PR 后验证 |

## 验证流程

每次 PR 完成后：

```bash
# Level 1: Unit Tests
catkin_make run_tests

# Level 2: Integration Tests
cd ~/catkin_ws && ./tests/run_integration.sh

# Level 3: System Test
roslaunch warehouse_utils demo_gazebo.launch  # 目视验证

# Regression Check
cat docs/REGRESSION.md  # 确认 REG-001~008 全部 ✅
```
