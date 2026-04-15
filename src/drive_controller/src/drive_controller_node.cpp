#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

class DriveControllerNode : public rclcpp::Node {
public:
  DriveControllerNode() : Node("drive_controller_node") {
    // Parameters
    this->declare_parameter<std::string>("pose_topic", "/pcl_pose");
    this->declare_parameter<double>("lookahead_distance", 1.5);
    this->declare_parameter<double>("max_linear_speed", 0.5);
    this->declare_parameter<double>("min_linear_speed", 0.1);
    this->declare_parameter<double>("max_steering_angle", 0.5);  // radians (~28.6 deg)
    this->declare_parameter<double>("wheelbase", 0.32);           // meters (typical rover)
    this->declare_parameter<double>("goal_tolerance", 0.3);       // meters
    this->declare_parameter<double>("control_rate", 20.0);        // Hz
    this->declare_parameter<double>("speed_reduction_angle", 0.3); // rad - reduce speed when steering hard
    this->declare_parameter<bool>("enable_obstacle_stop", true);
    this->declare_parameter<double>("max_yaw_rate", 0.8);                 // rad/s cap on angular.z
    this->declare_parameter<double>("forward_cone_length", 1.5);          // stop if point within this many m ahead
    this->declare_parameter<double>("forward_cone_half_width", 0.6);      // and within this many m laterally
    this->declare_parameter<double>("replan_trigger_radius", 3.0);        // request replan if obstacle within this radius
    this->declare_parameter<double>("replan_cooldown", 1.0);              // seconds between replan requests
    this->declare_parameter<double>("obstacle_cloud_max_age", 0.5);       // ignore stale clouds older than this

    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    lookahead_dist_ = this->get_parameter("lookahead_distance").as_double();
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    min_linear_speed_ = this->get_parameter("min_linear_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    double control_rate = this->get_parameter("control_rate").as_double();
    speed_reduction_angle_ = this->get_parameter("speed_reduction_angle").as_double();
    enable_obstacle_stop_ = this->get_parameter("enable_obstacle_stop").as_bool();
    max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    forward_cone_length_ = this->get_parameter("forward_cone_length").as_double();
    forward_cone_half_width_ = this->get_parameter("forward_cone_half_width").as_double();
    replan_trigger_radius_ = this->get_parameter("replan_trigger_radius").as_double();
    replan_cooldown_ = this->get_parameter("replan_cooldown").as_double();
    obstacle_cloud_max_age_ = this->get_parameter("obstacle_cloud_max_age").as_double();

    // Subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&DriveControllerNode::pose_callback, this, std::placeholders::_1));

    pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, 10,
      std::bind(&DriveControllerNode::pose_cov_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&DriveControllerNode::path_callback, this, std::placeholders::_1));

    // Subscribe to dynamic obstacles cloud — if non-empty, obstacles are present
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/dynamic_obstacles_cloud", 10,
      std::bind(&DriveControllerNode::obstacle_callback, this, std::placeholders::_1));

    // Publishers
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_request", 10);
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/lookahead_marker", 10);

    // Control loop timer
    double dt = 1.0 / control_rate;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&DriveControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
      "Drive controller ready. Pose: '%s', lookahead: %.2fm, speed: %.2f m/s, wheelbase: %.2fm",
      pose_topic.c_str(), lookahead_dist_, max_linear_speed_, wheelbase_);
  }

private:
  // Parameters
  double lookahead_dist_;
  double max_linear_speed_;
  double min_linear_speed_;
  double max_steering_angle_;
  double wheelbase_;
  double goal_tolerance_;
  double speed_reduction_angle_;
  bool enable_obstacle_stop_;
  double max_yaw_rate_;
  double forward_cone_length_;
  double forward_cone_half_width_;
  double replan_trigger_radius_;
  double replan_cooldown_;
  double obstacle_cloud_max_age_;

  // State
  bool pose_received_ = false;
  double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0, pose_yaw_ = 0;

  bool path_active_ = false;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t closest_idx_ = 0;

  // Latest dynamic-obstacle cloud, cached as (x, y) in map frame.
  std::vector<std::pair<double, double>> obstacle_xy_;
  rclcpp::Time obstacle_cloud_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_request_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // --------------------------------------------------------
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_z_ = msg->pose.position.z;
    pose_yaw_ = tf2::getYaw(msg->pose.orientation);
    pose_received_ = true;
  }

  void pose_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_z_ = msg->pose.pose.position.z;
    pose_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    pose_received_ = true;
  }

  // --------------------------------------------------------
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path");
      path_active_ = false;
      return;
    }

    path_ = msg->poses;
    path_active_ = true;
    closest_idx_ = 0;

    RCLCPP_INFO(this->get_logger(), "New path received: %zu waypoints", path_.size());
  }

  // --------------------------------------------------------
  // Cache the latest obstacle cloud as (x, y) map-frame points.
  // The decision to stop or replan is made in the control loop against pose.
  // --------------------------------------------------------
  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Obstacle cloud frame '%s' != 'map' — ignoring.",
        msg->header.frame_id.c_str());
      obstacle_xy_.clear();
      return;
    }

    obstacle_xy_.clear();
    obstacle_xy_.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    for (; it_x != it_x.end(); ++it_x, ++it_y) {
      obstacle_xy_.emplace_back(*it_x, *it_y);
    }
    obstacle_cloud_stamp_ = msg->header.stamp;
  }

  // --------------------------------------------------------
  // Scan cached obstacle points against rover pose:
  //   block_stop    = any point in a forward rectangle (immediate threat)
  //   trigger_replan = any point within replan_trigger_radius
  // --------------------------------------------------------
  void check_obstacles(bool& block_stop, bool& trigger_replan) {
    block_stop = false;
    trigger_replan = false;
    if (!enable_obstacle_stop_ || obstacle_xy_.empty()) return;

    double age = (this->now() - obstacle_cloud_stamp_).seconds();
    if (age > obstacle_cloud_max_age_) return;

    const double cos_y = std::cos(-pose_yaw_);
    const double sin_y = std::sin(-pose_yaw_);
    const double replan_r_sq = replan_trigger_radius_ * replan_trigger_radius_;

    for (const auto& p : obstacle_xy_) {
      double dx = p.first - pose_x_;
      double dy = p.second - pose_y_;

      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < replan_r_sq) trigger_replan = true;

      // Transform into rover-local frame (x forward, y left)
      double lx = dx * cos_y - dy * sin_y;
      double ly = dx * sin_y + dy * cos_y;
      if (lx > 0.0 && lx < forward_cone_length_ &&
          std::abs(ly) < forward_cone_half_width_) {
        block_stop = true;
      }

      if (block_stop && trigger_replan) break;
    }
  }

  // --------------------------------------------------------
  // Main control loop — pure pursuit
  // --------------------------------------------------------
  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    if (!pose_received_ || !path_active_ || path_.empty()) {
      cmd_pub_->publish(cmd);  // zero velocity
      return;
    }

    // Check if we reached the goal
    double dx_goal = path_.back().pose.position.x - pose_x_;
    double dy_goal = path_.back().pose.position.y - pose_y_;
    double dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (dist_to_goal < goal_tolerance_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Goal reached!");
      path_active_ = false;
      cmd_pub_->publish(cmd);  // stop
      return;
    }

    // Obstacle handling: stop only if something is directly ahead,
    // request a replan (throttled) if something is nearby.
    bool block_stop = false, trigger_replan = false;
    check_obstacles(block_stop, trigger_replan);

    if (trigger_replan) {
      double since = (this->now() - last_replan_request_time_).seconds();
      if (since > replan_cooldown_) {
        std_msgs::msg::Bool replan_msg;
        replan_msg.data = true;
        replan_pub_->publish(replan_msg);
        last_replan_request_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Replan requested (obstacle within %.1fm)",
          replan_trigger_radius_);
      }
    }

    if (block_stop) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Obstacle blocking forward cone — stopped, waiting for new path.");
      cmd_pub_->publish(cmd);  // zero velocity
      return;
    }

    // Find closest point on path (advance only forward)
    update_closest_index();

    // Find lookahead point
    double lx, ly;
    if (!find_lookahead_point(lx, ly)) {
      // No lookahead found — slow approach to end of path
      lx = path_.back().pose.position.x;
      ly = path_.back().pose.position.y;
    }

    // Publish lookahead marker for visualization
    publish_lookahead_marker(lx, ly);

    // Pure pursuit geometry
    // Transform lookahead point to robot frame
    double dx = lx - pose_x_;
    double dy = ly - pose_y_;

    // Rotate into robot's local frame
    double local_x = dx * std::cos(-pose_yaw_) - dy * std::sin(-pose_yaw_);
    double local_y = dx * std::sin(-pose_yaw_) + dy * std::cos(-pose_yaw_);

    double ld_sq = local_x * local_x + local_y * local_y;
    double ld = std::sqrt(ld_sq);

    if (ld < 0.01) {
      cmd_pub_->publish(cmd);
      return;
    }

    // Curvature = 2 * y / L^2
    double curvature = 2.0 * local_y / ld_sq;

    // Steering angle (Ackermann): delta = atan(curvature * wheelbase)
    double steering_angle = std::atan(curvature * wheelbase_);
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    // Speed: reduce when steering hard
    double abs_steering = std::abs(steering_angle);
    double speed = max_linear_speed_;
    if (abs_steering > speed_reduction_angle_) {
      double ratio = 1.0 - (abs_steering - speed_reduction_angle_) /
                     (max_steering_angle_ - speed_reduction_angle_ + 1e-6);
      ratio = std::clamp(ratio, 0.0, 1.0);
      speed = min_linear_speed_ + ratio * (max_linear_speed_ - min_linear_speed_);
    }

    // Slow down near goal
    if (dist_to_goal < lookahead_dist_) {
      speed = std::min(speed, min_linear_speed_ + (dist_to_goal / lookahead_dist_) *
              (max_linear_speed_ - min_linear_speed_));
    }

    // Innok Heros consumes standard Twist: angular.z = yaw rate (rad/s).
    // yaw_rate = curvature * linear_speed (equivalent to speed*tan(delta)/L).
    double yaw_rate = std::tan(steering_angle) * speed / wheelbase_;
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);

    cmd.linear.x = speed;
    cmd.angular.z = yaw_rate;

    cmd_pub_->publish(cmd);
  }

  // --------------------------------------------------------
  void update_closest_index() {
    double min_dist = std::numeric_limits<double>::max();
    size_t best = closest_idx_;

    // Search forward from current closest (don't go backwards on path)
    size_t search_end = std::min(closest_idx_ + 50, path_.size());
    for (size_t i = closest_idx_; i < search_end; ++i) {
      double dx = path_[i].pose.position.x - pose_x_;
      double dy = path_[i].pose.position.y - pose_y_;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        best = i;
      }
    }
    closest_idx_ = best;
  }

  // --------------------------------------------------------
  bool find_lookahead_point(double& lx, double& ly) {
    // Walk along path from closest_idx_ until we find a point >= lookahead_dist away
    for (size_t i = closest_idx_; i < path_.size(); ++i) {
      double dx = path_[i].pose.position.x - pose_x_;
      double dy = path_[i].pose.position.y - pose_y_;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist >= lookahead_dist_) {
        // Interpolate between i-1 and i for smoother tracking
        if (i > 0) {
          double dx_prev = path_[i-1].pose.position.x - pose_x_;
          double dy_prev = path_[i-1].pose.position.y - pose_y_;
          double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev);

          if (dist_prev < lookahead_dist_) {
            double seg_dx = path_[i].pose.position.x - path_[i-1].pose.position.x;
            double seg_dy = path_[i].pose.position.y - path_[i-1].pose.position.y;
            double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

            if (seg_len > 1e-6) {
              double t = (lookahead_dist_ - dist_prev) / (dist - dist_prev);
              t = std::clamp(t, 0.0, 1.0);
              lx = path_[i-1].pose.position.x + t * seg_dx;
              ly = path_[i-1].pose.position.y + t * seg_dy;
              return true;
            }
          }
        }

        lx = path_[i].pose.position.x;
        ly = path_[i].pose.position.y;
        return true;
      }
    }
    return false;
  }

  // --------------------------------------------------------
  void publish_lookahead_marker(double lx, double ly) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = lx;
    marker.pose.position.y = ly;
    marker.pose.position.z = pose_z_ + 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    lookahead_marker_pub_->publish(marker);
  }
};

// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
