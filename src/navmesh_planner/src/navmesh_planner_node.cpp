#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "navmesh_planner/navmesh_loader.hpp"
#include "navmesh_planner/coordinate_utils.hpp"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#include <cmath>
#include <vector>
#include <string>

class NavMeshPlannerNode : public rclcpp::Node {
public:
  NavMeshPlannerNode() : Node("navmesh_planner_node") {
    // Parameters
    this->declare_parameter<std::string>("navmesh_path", "");
    this->declare_parameter<std::string>("pose_topic", "/pcl_pose");
    this->declare_parameter<double>("poly_search_extent_x", 2.0);
    this->declare_parameter<double>("poly_search_extent_y", 4.0);
    this->declare_parameter<double>("poly_search_extent_z", 2.0);
    this->declare_parameter<int>("max_path_polys", 2048);
    this->declare_parameter<int>("max_straight_path", 256);
    this->declare_parameter<double>("waypoint_spacing", 0.0);

    std::string navmesh_path = this->get_parameter("navmesh_path").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    max_path_polys_ = this->get_parameter("max_path_polys").as_int();
    max_straight_path_ = this->get_parameter("max_straight_path").as_int();
    waypoint_spacing_ = this->get_parameter("waypoint_spacing").as_double();

    // Detour search extents (in Detour Y-UP coords)
    double ext_x = this->get_parameter("poly_search_extent_x").as_double();
    double ext_y = this->get_parameter("poly_search_extent_y").as_double();
    double ext_z = this->get_parameter("poly_search_extent_z").as_double();
    extents_[0] = static_cast<float>(ext_x);
    extents_[1] = static_cast<float>(ext_y);  // Y-UP height
    extents_[2] = static_cast<float>(ext_z);

    // --------------------------------------------------------
    // Load NavMesh
    // --------------------------------------------------------
    if (navmesh_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "navmesh_path parameter is required!");
      return;
    }

    nav_mesh_ = loadNavMesh(navmesh_path);
    if (!nav_mesh_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load navmesh: %s", navmesh_path.c_str());
      return;
    }

    // Count tiles
    int tile_count = 0;
    for (int i = 0; i < nav_mesh_->getMaxTiles(); ++i) {
      const dtNavMesh* const_mesh = nav_mesh_;
      const dtMeshTile* tile = const_mesh->getTile(i);
      if (tile && tile->header && tile->dataSize > 0) tile_count++;
    }
    RCLCPP_INFO(this->get_logger(), "NavMesh loaded: %d tiles from %s",
      tile_count, navmesh_path.c_str());

    // Init query
    nav_query_ = dtAllocNavMeshQuery();
    if (dtStatusFailed(nav_query_->init(nav_mesh_, 2048))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init dtNavMeshQuery");
      return;
    }

    // --------------------------------------------------------
    // Subscribers
    // --------------------------------------------------------
    // Support both PoseStamped (NDT /pcl_pose) and PoseWithCovarianceStamped (AMCL)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&NavMeshPlannerNode::pose_callback, this, std::placeholders::_1));

    pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, 10,
      std::bind(&NavMeshPlannerNode::pose_cov_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&NavMeshPlannerNode::goal_callback, this, std::placeholders::_1));

    replan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/replan_request", 10,
      std::bind(&NavMeshPlannerNode::replan_callback, this, std::placeholders::_1));

    // --------------------------------------------------------
    // Publishers
    // --------------------------------------------------------
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/planned_path_markers", 10);

    RCLCPP_INFO(this->get_logger(),
      "NavMesh planner ready. Pose topic: '%s', waiting for /goal_pose...",
      pose_topic.c_str());
  }

  ~NavMeshPlannerNode() {
    if (nav_query_) dtFreeNavMeshQuery(nav_query_);
    if (nav_mesh_) dtFreeNavMesh(nav_mesh_);
  }

private:
  // Detour objects
  dtNavMesh* nav_mesh_ = nullptr;
  dtNavMeshQuery* nav_query_ = nullptr;
  dtQueryFilter filter_;
  float extents_[3];

  // Parameters
  int max_path_polys_;
  int max_straight_path_;
  double waypoint_spacing_;

  // Current pose
  bool pose_received_ = false;
  double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0;

  // Active goal
  bool goal_active_ = false;
  double goal_x_ = 0, goal_y_ = 0, goal_z_ = 0;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;

  // --------------------------------------------------------
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_z_ = msg->pose.position.z;
    pose_received_ = true;
  }

  void pose_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_z_ = msg->pose.pose.position.z;
    pose_received_ = true;
  }

  // --------------------------------------------------------
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!pose_received_) {
      RCLCPP_WARN(this->get_logger(), "No pose received yet, ignoring goal");
      return;
    }

    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_z_ = msg->pose.position.z;
    goal_active_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Goal received: (%.2f, %.2f, %.2f) -> planning from (%.2f, %.2f, %.2f)",
      goal_x_, goal_y_, goal_z_, pose_x_, pose_y_, pose_z_);

    plan_and_publish();
  }

  // --------------------------------------------------------
  void replan_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;

    if (!pose_received_) {
      RCLCPP_WARN(this->get_logger(), "Replan requested but no pose received yet");
      return;
    }
    if (!goal_active_) {
      RCLCPP_WARN(this->get_logger(), "Replan requested but no active goal");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Replan requested: replanning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
      pose_x_, pose_y_, pose_z_, goal_x_, goal_y_, goal_z_);

    plan_and_publish();
  }

  // --------------------------------------------------------
  // Main planning function
  // --------------------------------------------------------
  void plan_and_publish() {
    if (!nav_mesh_ || !nav_query_) return;

    // Convert ROS coords to Detour (Y-UP)
    float start_dt[3], goal_dt[3];
    rosToDetour(pose_x_, pose_y_, pose_z_, start_dt);
    rosToDetour(goal_x_, goal_y_, goal_z_, goal_dt);

    // Find nearest polygons
    dtPolyRef start_ref = 0, goal_ref = 0;
    float start_nearest[3], goal_nearest[3];

    nav_query_->findNearestPoly(start_dt, extents_, &filter_, &start_ref, start_nearest);
    if (start_ref == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "findNearestPoly failed for start (%.2f, %.2f, %.2f). Check extents or position.",
        pose_x_, pose_y_, pose_z_);
      return;
    }

    nav_query_->findNearestPoly(goal_dt, extents_, &filter_, &goal_ref, goal_nearest);
    if (goal_ref == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "findNearestPoly failed for goal (%.2f, %.2f, %.2f). Check extents or position.",
        goal_x_, goal_y_, goal_z_);
      return;
    }

    // Find path (polygon corridor)
    std::vector<dtPolyRef> poly_path(max_path_polys_);
    int poly_count = 0;
    dtStatus status = nav_query_->findPath(
      start_ref, goal_ref, start_nearest, goal_nearest,
      &filter_, poly_path.data(), &poly_count, max_path_polys_);

    if (dtStatusFailed(status) || poly_count == 0) {
      RCLCPP_ERROR(this->get_logger(), "findPath failed or returned 0 polygons");
      return;
    }

    // Convert to straight path (waypoints)
    std::vector<float> straight_path(max_straight_path_ * 3);
    int straight_count = 0;
    nav_query_->findStraightPath(
      start_nearest, goal_nearest, poly_path.data(), poly_count,
      straight_path.data(), nullptr, nullptr, &straight_count,
      max_straight_path_, DT_STRAIGHTPATH_ALL_CROSSINGS);

    if (straight_count == 0) {
      RCLCPP_ERROR(this->get_logger(), "findStraightPath returned 0 waypoints");
      return;
    }

    // Convert waypoints from Detour to ROS coords
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    for (int i = 0; i < straight_count; ++i) {
      double rx, ry, rz;
      detourToRos(&straight_path[i * 3], rx, ry, rz);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = this->now();
      ps.pose.position.x = rx;
      ps.pose.position.y = ry;
      ps.pose.position.z = rz;
      ps.pose.orientation.w = 1.0;
      waypoints.push_back(ps);
    }

    // Optional: interpolate waypoints for smoother driving
    if (waypoint_spacing_ > 0.0 && waypoints.size() >= 2) {
      waypoints = interpolate_waypoints(waypoints, waypoint_spacing_);
    }

    // Publish path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    path_msg.poses = waypoints;
    path_pub_->publish(path_msg);

    // Publish visualization marker
    publish_path_marker(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "Path planned: %d polys -> %d waypoints (published %zu poses)",
      poly_count, straight_count, waypoints.size());
  }

  // --------------------------------------------------------
  // Interpolate waypoints for smoother driving
  // --------------------------------------------------------
  std::vector<geometry_msgs::msg::PoseStamped> interpolate_waypoints(
      const std::vector<geometry_msgs::msg::PoseStamped>& input, double spacing) {
    std::vector<geometry_msgs::msg::PoseStamped> result;
    result.push_back(input[0]);

    for (size_t i = 1; i < input.size(); ++i) {
      double dx = input[i].pose.position.x - input[i-1].pose.position.x;
      double dy = input[i].pose.position.y - input[i-1].pose.position.y;
      double dz = input[i].pose.position.z - input[i-1].pose.position.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      if (dist > spacing) {
        int n = static_cast<int>(std::ceil(dist / spacing));
        for (int j = 1; j <= n; ++j) {
          double t = static_cast<double>(j) / n;
          geometry_msgs::msg::PoseStamped ps;
          ps.header = input[i].header;
          ps.pose.position.x = input[i-1].pose.position.x + t * dx;
          ps.pose.position.y = input[i-1].pose.position.y + t * dy;
          ps.pose.position.z = input[i-1].pose.position.z + t * dz;
          ps.pose.orientation.w = 1.0;
          result.push_back(ps);
        }
      } else {
        result.push_back(input[i]);
      }
    }
    return result;
  }

  // --------------------------------------------------------
  // Publish path as LINE_STRIP marker for RViz
  // --------------------------------------------------------
  void publish_path_marker(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;  // line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& wp : waypoints) {
      geometry_msgs::msg::Point p;
      p.x = wp.pose.position.x;
      p.y = wp.pose.position.y;
      p.z = wp.pose.position.z;
      marker.points.push_back(p);
    }

    path_marker_pub_->publish(marker);
  }
};

// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavMeshPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
