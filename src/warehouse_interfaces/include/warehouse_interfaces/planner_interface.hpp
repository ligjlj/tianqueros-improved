#ifndef WAREHOUSE_INTERFACES_PLANNER_INTERFACE_HPP_
#define WAREHOUSE_INTERFACES_PLANNER_INTERFACE_HPP_

#include <vector>
#include "warehouse_costmap/costmap_layer.hpp"

namespace warehouse_interfaces {

struct WorldPoint { double x, y; };
struct Path { std::vector<WorldPoint> points; double length_m=0; int nodes_expanded=0; double time_ms=0; bool success=false; };

/// Abstract global planner plugin (ADR-005).
/// YAML-switchable: "AStarPlanner", "RRTStar", "HybridAStar".
class GlobalPlannerPlugin {
public:
  virtual Path plan(const warehouse_costmap::CostmapLayer& costmap,
                    WorldPoint start, WorldPoint goal) = 0;
  virtual ~GlobalPlannerPlugin() = default;
};

}  // namespace warehouse_interfaces
#endif
