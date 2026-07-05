/// @file astar_planner_node.cpp
/// ROS node wrapper for AStarPlanner.
///
/// Subscribes:
///   /map                   — nav_msgs::OccupancyGrid
///   /move_base_simple/goal — geometry_msgs::PoseStamped (from rviz "2D Nav Goal")
///   /tf                    — robot pose (map → base_link)
///
/// Publishes:
///   /global_path           — nav_msgs::Path
///   /global_path_marker    — visualization_msgs::Marker (SPHERE_LIST waypoints)

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "warehouse_utils/grid_map.hpp"
#include "warehouse_planner/astar_planner.hpp"

class AStarPlannerNode {
public:
  AStarPlannerNode()
    : nh_("~")
    , tf_listener_(tf_buffer_)
  {
    // ── Load config ────────────────────────────────────────
    warehouse_planner::AStarPlanner::Config cfg;
    nh_.param("heuristic_weight",   cfg.heuristic_weight,   1.0);
    nh_.param("collision_margin_m", cfg.collision_margin_m, 0.1);

    // GridMap config (shared with warehouse_utils defaults).
    warehouse_utils::GridMap::Config grid_cfg;
    nh_.param("inflation_radius_m", grid_cfg.inflation_radius_m, 1.0);
    nh_.param("robot_radius_m",     grid_cfg.robot_radius_m,     0.5);

    grid_   = warehouse_utils::GridMap(grid_cfg);
    planner_ = warehouse_planner::AStarPlanner(cfg);

    // ── Subscribers ───────────────────────────────────────
    map_sub_  = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "/map", 1, &AStarPlannerNode::mapCallback, this);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, &AStarPlannerNode::goalCallback, this);
    expl_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/exploration_goal", 1, &AStarPlannerNode::goalCallback, this);

    // ── Publishers ────────────────────────────────────────
    path_pub_   = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/global_path_marker", 1);

    ROS_INFO("A* Planner Node ready. Use '2D Nav Goal' in rviz to set goal.");
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    try {
      grid_.loadFromMsg(*msg);
      has_map_ = true;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to load map: " << e.what());
    }
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!has_map_) {
      ROS_WARN("No map received yet. Cannot plan.");
      return;
    }

    // ── Get robot pose from TF with longer timeout for sim time ──
    warehouse_utils::WorldPoint start;
    bool have_pose = false;

    try {
      geometry_msgs::TransformStamped tf =
        tf_buffer_.lookupTransform("map", "base_link",
                                    ros::Time(0), ros::Duration(10.0));
      start.x = tf.transform.translation.x;
      start.y = tf.transform.translation.y;
      have_pose = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM("TF lookup failed: " << ex.what()
                      << ". Waiting for valid TF...");
      // Don't fallback to map center — return empty path to signal failure.
      publishEmptyPath("map");
      return;
    }

    // ── Goal in world coords ──────────────────────────────
    warehouse_utils::WorldPoint goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;

    ROS_INFO_STREAM("Planning from (" << start.x << "," << start.y
                    << ") to (" << goal.x << "," << goal.y << ")");

    // ── Run planner ───────────────────────────────────────
    auto result = planner_.plan(grid_, start, goal);

    if (result.success) {
      publishPath(result);
    } else {
      publishEmptyPath(msg->header.frame_id);
    }
  }

private:
  void publishPath(const warehouse_planner::AStarPlanner::Result& result) {
    // ── Path message ──────────────────────────────────────
    nav_msgs::Path path_msg;
    path_msg.header.stamp    = ros::Time::now();
    path_msg.header.frame_id = "map";

    for (const auto& wp : result.path) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    path_pub_.publish(path_msg);

    // ── Marker (waypoints) ────────────────────────────────
    visualization_msgs::Marker marker;
    marker.header      = path_msg.header;
    marker.ns          = "astar_waypoints";
    marker.id          = 0;
    marker.type        = visualization_msgs::Marker::SPHERE_LIST;
    marker.action      = visualization_msgs::Marker::ADD;
    marker.scale.x     = 0.1;
    marker.scale.y     = 0.1;
    marker.scale.z     = 0.1;
    marker.color.r     = 0.0;
    marker.color.g     = 1.0;
    marker.color.b     = 0.0;
    marker.color.a     = 0.8;

    // Decimate for visualization: max ~200 markers.
    const size_t step = std::max<size_t>(1, result.path.size() / 200);
    for (size_t i = 0; i < result.path.size(); i += step) {
      geometry_msgs::Point p;
      p.x = result.path[i].x;
      p.y = result.path[i].y;
      p.z = 0.05;
      marker.points.push_back(p);
    }
    marker_pub_.publish(marker);

    ROS_INFO_STREAM("Published path: " << result.path.size()
                    << " waypoints, " << result.path_length_m << " m");
  }

  void publishEmptyPath(const std::string& frame_id) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp    = ros::Time::now();
    path_msg.header.frame_id = frame_id;
    path_pub_.publish(path_msg);

    // Clear marker.
    visualization_msgs::Marker marker;
    marker.header  = path_msg.header;
    marker.ns      = "astar_waypoints";
    marker.id      = 0;
    marker.action  = visualization_msgs::Marker::DELETE;
    marker_pub_.publish(marker);
  }

  // ── ROS handles ─────────────────────────────────────────
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber expl_goal_sub_;
  ros::Publisher  path_pub_;
  ros::Publisher  marker_pub_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── Core ────────────────────────────────────────────────
  warehouse_utils::GridMap            grid_;
  warehouse_planner::AStarPlanner     planner_;
  bool has_map_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar_planner_node");
  AStarPlannerNode node;
  ros::spin();
  return 0;
}
