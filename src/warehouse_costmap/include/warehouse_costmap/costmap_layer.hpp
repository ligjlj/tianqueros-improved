#ifndef WAREHOUSE_COSTMAP_COSTMAP_LAYER_HPP_
#define WAREHOUSE_COSTMAP_COSTMAP_LAYER_HPP_

#include <vector>
#include <cstdint>
#include <nav_msgs/OccupancyGrid.h>

namespace warehouse_costmap {

/// Abstract costmap layer (ADR-002).
/// Each layer provides a grid of cost values. Subclasses implement
/// how costs are generated.
class CostmapLayer {
public:
  virtual ~CostmapLayer() = default;

  virtual double getCost(int row, int col) const = 0;
  virtual int    width()     const = 0;
  virtual int    height()    const = 0;
  virtual double resolution() const = 0;
  virtual double originX()   const = 0;
  virtual double originY()   const = 0;

  /// Raw occupancy value at (row,col). -1=unknown, 0=free, 100=occupied.
  virtual int8_t getRaw(int row, int col) const { return 0; }
};

/// Raw costmap: direct occupancy values from OccupancyGrid.
/// Cost: 0=free, 254=occupied, -1=unknown.
class RawCostmap : public CostmapLayer {
public:
  void loadFromMsg(const nav_msgs::OccupancyGrid& msg);

  double getCost(int row, int col) const override;
  int    width()     const override { return w_; }
  int    height()    const override { return h_; }
  double resolution() const override { return res_; }
  double originX()   const override { return ox_; }
  double originY()   const override { return oy_; }
  int8_t getRaw(int row, int col) const override;

  bool isValid(int r, int c) const { return r>=0 && r<h_ && c>=0 && c<w_; }

private:
  int w_=0, h_=0;
  double res_=0.05, ox_=0, oy_=0;
  std::vector<int8_t> data_;
};

/// Inflated costmap: obstacle distance field.
/// Built from a RawCostmap, applies quadratic decay up to inflation_radius.
class InflatedCostmap : public CostmapLayer {
public:
  void buildFrom(const RawCostmap& raw, double inflation_radius_m);

  double getCost(int row, int col) const override;
  int    width()     const override { return w_; }
  int    height()    const override { return h_; }
  double resolution() const override { return res_; }
  double originX()   const override { return ox_; }
  double originY()   const override { return oy_; }

private:
  int w_=0, h_=0;
  double res_=0.05, ox_=0, oy_=0;
  std::vector<double> cost_;
};

}  // namespace warehouse_costmap
#endif
