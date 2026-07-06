#ifndef WAREHOUSE_INTERFACES_CONTROLLER_INTERFACE_HPP_
#define WAREHOUSE_INTERFACES_CONTROLLER_INTERFACE_HPP_

#include "warehouse_costmap/costmap_layer.hpp"
#include "warehouse_interfaces/planner_interface.hpp"

namespace warehouse_interfaces {

struct RobotState { double x, y, theta, v, w; };
struct Twist { double v, w; };

/// Abstract local planner plugin (ADR-005).
/// YAML-switchable: "DWAPlanner", "TEBPlanner", "MPCPlanner".
class LocalPlannerPlugin {
public:
  virtual Twist computeVelocity(const warehouse_costmap::CostmapLayer& costmap,
                                 const RobotState& state,
                                 const Path& path) = 0;
  virtual bool isGoalReached(const RobotState& state, WorldPoint goal) const = 0;
  virtual ~LocalPlannerPlugin() = default;
};

}  // namespace warehouse_interfaces
#endif
