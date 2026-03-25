#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
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
    this->declare_parameter<double>("obstacle_stop_timeout", 2.0); // seconds before requesting replan

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
    obstacle_stop_timeout_ = this->get_parameter("obstacle_stop_timeout").as_double();

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
  double obstacle_stop_timeout_;

  // State
  bool pose_received_ = false;
  double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0, pose_yaw_ = 0;

  bool path_active_ = false;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t closest_idx_ = 0;

  bool obstacle_detected_ = false;
  rclcpp::Time last_obstacle_time_;
  bool replan_requested_ = false;

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
    replan_requested_ = false;

    RCLCPP_INFO(this->get_logger(), "New path received: %zu waypoints", path_.size());
  }

  // --------------------------------------------------------
  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Check if cloud has any points
    bool has_obstacles = (msg->width * msg->height > 0);

    if (has_obstacles) {
      obstacle_detected_ = true;
      last_obstacle_time_ = this->now();
    } else {
      obstacle_detected_ = false;
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

    // Obstacle handling
    if (enable_obstacle_stop_ && obstacle_detected_) {
      // Stop the rover
      cmd_pub_->publish(cmd);

      // Check if we should request a replan
      double elapsed = (this->now() - last_obstacle_time_).seconds();
      if (elapsed < 0.5 && !replan_requested_) {
        // Obstacles still active — after timeout, request replan
        if (obstacle_stop_timeout_ > 0) {
          // We'll request replan after seeing obstacles continuously
          // For now, just stop and wait
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Obstacle detected — stopped. Waiting...");

          // Request replan
          std_msgs::msg::Bool replan_msg;
          replan_msg.data = true;
          replan_pub_->publish(replan_msg);
          replan_requested_ = true;
          RCLCPP_INFO(this->get_logger(), "Replan requested due to obstacle");
        }
      }
      return;
    }

    // If obstacle cleared, reset replan flag
    if (!obstacle_detected_) {
      replan_requested_ = false;
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

    // For Ackermann: linear.x = speed, angular.z = steering angle
    // Many ROS Ackermann bridges expect angular.z as steering angle
    cmd.linear.x = speed;
    cmd.angular.z = steering_angle;

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
