#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class DriveControllerNode : public rclcpp::Node {
public:
  DriveControllerNode() : Node("drive_controller_node") {
    // Parameters
    this->declare_parameter<std::string>("pose_topic", "/pcl_pose");
    this->declare_parameter<std::string>("message_typ", "PoseWithCovariance");
    this->declare_parameter<double>("lookahead_distance", 1.5);
    this->declare_parameter<double>("max_linear_speed", 0.5);
    this->declare_parameter<double>("min_linear_speed", 0.1);
    this->declare_parameter<double>("max_angular_velocity", 1.0);  // rad/s — max rotation rate
    this->declare_parameter<double>("goal_tolerance", 0.3);        // meters
    this->declare_parameter<double>("control_rate", 20.0);         // Hz
    this->declare_parameter<double>("speed_reduction_omega", 0.5); // rad/s — reduce speed above this rotation rate
    this->declare_parameter<bool>("enable_obstacle_stop", true);
    this->declare_parameter<double>("obstacle_stop_timeout", 2.0);
    this->declare_parameter<double>("obstacle_stop_cooldown", 1.0);
    this->declare_parameter<double>("in_place_rotation_threshold", 1.0);
    this->declare_parameter<double>("in_place_kp", 2.0);
    this->declare_parameter<double>("obstacle_slowdown_dist", 2.5); // meters — start reducing speed
    this->declare_parameter<double>("obstacle_min_speed_dist", 0.5); // meters — drive at min_linear_speed

    // Safety bubble: hard 360-deg stop. If ANY dynamic obstacle is within
    // safety_stop_radius of the rover (any direction), publish zero velocity and
    // hold until it leaves. The planner keeps replanning independently — this only
    // gates /cmd_vel. safety_stop_min_dist ignores points closer than this (the
    // rover's own body when no detector self-filter is active); 0 = disabled.
    this->declare_parameter<bool>("safety_stop_enabled", true);
    this->declare_parameter<double>("safety_stop_radius", 1.0);
    this->declare_parameter<double>("safety_stop_min_dist", 0.0);

    // Smooth tracking. Pure pursuit's steering gain is 2*v/L — a short fixed
    // lookahead makes the rover weave on straights (gain too high for the
    // pose->actuation loop delay), a long one cuts corners on detours. So L
    // scales with commanded speed: L = clamp(lookahead_time * v,
    // lookahead_distance, lookahead_max). Fast straight -> long L (stable);
    // slowed near obstacles -> short L (tight tracking).
    this->declare_parameter<double>("lookahead_time", 1.8);
    this->declare_parameter<double>("lookahead_max", 1.2);
    // First-order low-pass on omega: filters the 10 Hz pose jitter out of the
    // 50 Hz steering command.
    this->declare_parameter<double>("omega_smoothing_tau", 0.15);
    // The pose in hand is stale by NDT latency + bridge transport, and the
    // command takes ZMQ + ROS1 + motor time to act. Steering computed on a
    // pose this old oscillates; predict the pose forward by this many seconds
    // (using the last commanded twist) before computing steering.
    this->declare_parameter<double>("pose_latency", 0.20);
    // Forward acceleration ramp (m/s^2). Deceleration stays instant (safety).
    this->declare_parameter<double>("max_linear_accel", 0.5);

    // Bubble escape: the 360-deg bubble alone deadlocks when a path passes a
    // static obstacle closer than safety_stop_radius — the obstacle never
    // clears and never blocks the path geometrically, so nobody replans. On
    // bubble entry a replan is requested; if still held after
    // bubble_escape_after seconds, the rover backs away at
    // bubble_escape_speed (only while the rear hemisphere is clear) until
    // safety_stop_radius + bubble_escape_hysteresis, then replans again.
    this->declare_parameter<double>("bubble_escape_after", 3.0);
    this->declare_parameter<double>("bubble_escape_speed", 0.10);
    this->declare_parameter<double>("bubble_escape_hysteresis", 0.10);
    this->declare_parameter<double>("bubble_escape_max_duration", 6.0);

    // Obstacle shy-away: pure pursuit tracks the path with no awareness of how
    // close the path itself runs to an obstacle. Cluster-estimation variance
    // (visible-face centroid bias, persistence filtering, pose error)
    // occasionally plants a planned detour slightly inside the bubble, freezing
    // a run that succeeds nine times out of ten. This layer biases omega away
    // from the nearest MEASURED point on each side once it is closer than
    // repulsion_radius — a true-distance guarantee independent of the planner's
    // cylinder estimate. Left and right biases cancel, so a symmetric corridor
    // is still driven straight through the middle.
    this->declare_parameter<double>("repulsion_radius", 0.70);
    this->declare_parameter<double>("repulsion_gain", 2.0);
    this->declare_parameter<double>("repulsion_max_omega", 0.6);

    // External speed limit (/speed_limit, from the ZED human governor): fresh
    // non-negative values cap the commanded speed. Values older than this
    // timeout — or negative ones — are ignored, so a dead/absent camera node
    // fails OPEN (normal driving); the LiDAR bubble stays the collision guard.
    this->declare_parameter<double>("speed_limit_timeout", 1.0);

    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    std::string message_typ = this->get_parameter("message_typ").as_string();
    lookahead_dist_ = this->get_parameter("lookahead_distance").as_double();
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    min_linear_speed_ = this->get_parameter("min_linear_speed").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    double control_rate = this->get_parameter("control_rate").as_double();
    speed_reduction_omega_ = this->get_parameter("speed_reduction_omega").as_double();
    enable_obstacle_stop_ = this->get_parameter("enable_obstacle_stop").as_bool();
    obstacle_stop_timeout_ = this->get_parameter("obstacle_stop_timeout").as_double();
    obstacle_stop_cooldown_ = this->get_parameter("obstacle_stop_cooldown").as_double();
    in_place_rotation_threshold_ = this->get_parameter("in_place_rotation_threshold").as_double();
    in_place_kp_ = this->get_parameter("in_place_kp").as_double();
    obstacle_slowdown_dist_ = this->get_parameter("obstacle_slowdown_dist").as_double();
    obstacle_min_speed_dist_ = this->get_parameter("obstacle_min_speed_dist").as_double();
    safety_stop_enabled_ = this->get_parameter("safety_stop_enabled").as_bool();
    safety_stop_radius_ = this->get_parameter("safety_stop_radius").as_double();
    safety_stop_min_dist_ = this->get_parameter("safety_stop_min_dist").as_double();
    lookahead_time_ = this->get_parameter("lookahead_time").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    omega_smoothing_tau_ = this->get_parameter("omega_smoothing_tau").as_double();
    pose_latency_ = this->get_parameter("pose_latency").as_double();
    max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
    bubble_escape_after_ = this->get_parameter("bubble_escape_after").as_double();
    bubble_escape_speed_ = this->get_parameter("bubble_escape_speed").as_double();
    bubble_escape_hysteresis_ = this->get_parameter("bubble_escape_hysteresis").as_double();
    bubble_escape_max_duration_ = this->get_parameter("bubble_escape_max_duration").as_double();
    repulsion_radius_ = this->get_parameter("repulsion_radius").as_double();
    repulsion_gain_ = this->get_parameter("repulsion_gain").as_double();
    repulsion_max_omega_ = this->get_parameter("repulsion_max_omega").as_double();
    speed_limit_timeout_ = this->get_parameter("speed_limit_timeout").as_double();

    // Subscribers
    if (message_typ == "Odometry") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10,
        std::bind(&DriveControllerNode::odom_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Pose mode: Odometry (Isaac Sim)");
    } else {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10,
        std::bind(&DriveControllerNode::pose_callback, this, std::placeholders::_1));
      pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 10,
        std::bind(&DriveControllerNode::pose_cov_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Pose mode: PoseWithCovariance / PoseStamped");
    }

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&DriveControllerNode::path_callback, this, std::placeholders::_1));

    // Subscribe to dynamic obstacles cloud — if non-empty, obstacles are present
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/dynamic_obstacles_cloud", 10,
      std::bind(&DriveControllerNode::obstacle_callback, this, std::placeholders::_1));

    speed_limit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/speed_limit", 10,
      std::bind(&DriveControllerNode::speed_limit_callback, this, std::placeholders::_1));

    // Publishers
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_request", 10);
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/lookahead_marker", 10);

    // Control loop timer
    double dt = 1.0 / control_rate;
    control_dt_ = dt;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&DriveControllerNode::control_loop, this));

    last_path_received_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Drive controller ready. Pose: '%s', lookahead: %.2fm, speed: %.2f m/s, max_omega: %.2f rad/s",
      pose_topic.c_str(), lookahead_dist_, max_linear_speed_, max_angular_velocity_);
  }

private:
  // Parameters
  double lookahead_dist_;
  double max_linear_speed_;
  double min_linear_speed_;
  double max_angular_velocity_;
  double goal_tolerance_;
  double speed_reduction_omega_;
  bool enable_obstacle_stop_;
  double obstacle_stop_timeout_;
  double obstacle_stop_cooldown_;
  double in_place_rotation_threshold_;
  double in_place_kp_;
  double obstacle_slowdown_dist_;
  double obstacle_min_speed_dist_;
  bool safety_stop_enabled_ = true;
  double safety_stop_radius_ = 1.0;
  double safety_stop_min_dist_ = 0.0;
  double lookahead_time_ = 1.8;
  double lookahead_max_ = 1.2;
  double omega_smoothing_tau_ = 0.15;
  double pose_latency_ = 0.20;
  double max_linear_accel_ = 0.5;
  double bubble_escape_after_ = 3.0;
  double bubble_escape_speed_ = 0.10;
  double bubble_escape_hysteresis_ = 0.10;
  double bubble_escape_max_duration_ = 6.0;
  double repulsion_radius_ = 0.70;
  double repulsion_gain_ = 2.0;
  double repulsion_max_omega_ = 0.6;
  double speed_limit_timeout_ = 1.0;
  double control_dt_ = 0.02;

  // State
  bool pose_received_ = false;
  double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0, pose_yaw_ = 0;
  // Latency-compensated pose used by the steering pipeline this cycle
  double pred_x_ = 0, pred_y_ = 0, pred_yaw_ = 0;
  double last_cmd_speed_ = 0.0;
  double last_cmd_omega_ = 0.0;
  double omega_filt_ = 0.0;
  double cur_lookahead_ = 0.6;

  // Bubble escape state machine
  bool bubble_holding_ = false;
  bool bubble_escaping_ = false;
  rclcpp::Time bubble_hold_start_;
  rclcpp::Time bubble_escape_start_;

  bool path_active_ = false;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t closest_idx_ = 0;

  bool obstacle_detected_ = false;
  rclcpp::Time last_obstacle_time_;
  rclcpp::Time obstacle_start_time_;
  bool replan_requested_ = false;
  rclcpp::Time last_path_received_time_;
  double nearest_obstacle_dist_ = std::numeric_limits<double>::max(); // 2D distance, forward-facing only
  double nearest_obstacle_dist_any_ = std::numeric_limits<double>::max(); // 2D distance, 360 deg (safety bubble)
  double nearest_obstacle_dist_rear_ = std::numeric_limits<double>::max(); // 2D distance, rear hemisphere
  double nearest_any_x_ = 0.0, nearest_any_y_ = 0.0; // map coords of the nearest 360-deg point
  // Shy-away inputs: nearest point per side (abeam or ahead) and how abeam it
  // is (|local_y|/dist, 0 = dead ahead .. 1 = fully beside).
  double repulse_left_dist_ = std::numeric_limits<double>::max();
  double repulse_right_dist_ = std::numeric_limits<double>::max();
  double repulse_left_w_ = 0.0;
  double repulse_right_w_ = 0.0;

  // External speed limit (ZED human governor). Negative = no limit.
  double external_speed_limit_ = -1.0;
  bool speed_limit_received_ = false;
  rclcpp::Time last_speed_limit_time_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // --------------------------------------------------------
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_z_ = msg->pose.pose.position.z;
    pose_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    pose_received_ = true;
  }

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
    last_path_received_time_ = this->now();
    // Do NOT reset replan_requested_ here — if an obstacle is still present,
    // resetting it causes an infinite replan loop (path arrives → reset → replan again).
    // replan_requested_ is only cleared when the obstacle actually clears.

    RCLCPP_INFO(this->get_logger(), "New path received: %zu waypoints", path_.size());
  }

  // --------------------------------------------------------
  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    bool has_obstacles = (msg->width * msg->height > 0);

    if (has_obstacles) {
      if (!obstacle_detected_) {
        obstacle_start_time_ = this->now();
      }
      obstacle_detected_ = true;
      last_obstacle_time_ = this->now();

      // Compute nearest obstacle distance — forward-facing only (ignore obstacles behind rover).
      // Only points in the forward hemisphere (local_x > 0 in robot frame) count.
      // This prevents slowing for obstacles already passed or to the side during avoidance arcs.
      double cos_yaw = std::cos(pose_yaw_);
      double sin_yaw = std::sin(pose_yaw_);
      double min_d2 = std::numeric_limits<double>::max();      // forward-facing
      double min_d2_any = std::numeric_limits<double>::max();  // 360 deg (safety bubble)
      double min_d2_rear = std::numeric_limits<double>::max(); // rear hemisphere (escape reverse)
      double min_d2_left = std::numeric_limits<double>::max(); // shy-away, left side
      double min_d2_right = std::numeric_limits<double>::max();// shy-away, right side
      double left_abs_y = 0.0, right_abs_y = 0.0;
      double inner2 = safety_stop_min_dist_ * safety_stop_min_dist_;
      try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          double dx = *iter_x - pose_x_;
          double dy = *iter_y - pose_y_;
          double d2 = dx * dx + dy * dy;
          // 360-deg nearest for the safety bubble. Ignore points inside the inner
          // floor (treated as the rover's own body when no self-filter is active).
          if (d2 >= inner2 && d2 < min_d2_any) {
            min_d2_any = d2;
            nearest_any_x_ = *iter_x;
            nearest_any_y_ = *iter_y;
          }
          double local_x = dx * cos_yaw + dy * sin_yaw;  // positive = ahead of robot
          double local_y = -dx * sin_yaw + dy * cos_yaw; // positive = left of robot
          // Shy-away bookkeeping: nearest point per side among points abeam or
          // ahead (slightly behind the axle still steers the tail clear).
          if (local_x > -0.3 && d2 >= inner2) {
            if (local_y >= 0.0) {
              if (d2 < min_d2_left) { min_d2_left = d2; left_abs_y = local_y; }
            } else {
              if (d2 < min_d2_right) { min_d2_right = d2; right_abs_y = -local_y; }
            }
          }
          if (local_x < 0.0) {
            if (d2 >= inner2 && d2 < min_d2_rear) min_d2_rear = d2;
            continue;
          }
          if (d2 < min_d2) min_d2 = d2;
        }
      } catch (...) {}
      nearest_obstacle_dist_ = (min_d2 < std::numeric_limits<double>::max())
                                ? std::sqrt(min_d2)
                                : std::numeric_limits<double>::max();
      nearest_obstacle_dist_any_ = (min_d2_any < std::numeric_limits<double>::max())
                                ? std::sqrt(min_d2_any)
                                : std::numeric_limits<double>::max();
      nearest_obstacle_dist_rear_ = (min_d2_rear < std::numeric_limits<double>::max())
                                ? std::sqrt(min_d2_rear)
                                : std::numeric_limits<double>::max();
      if (min_d2_left < std::numeric_limits<double>::max()) {
        repulse_left_dist_ = std::sqrt(min_d2_left);
        repulse_left_w_ = std::clamp(left_abs_y / std::max(repulse_left_dist_, 1e-6), 0.0, 1.0);
      } else {
        repulse_left_dist_ = std::numeric_limits<double>::max();
        repulse_left_w_ = 0.0;
      }
      if (min_d2_right < std::numeric_limits<double>::max()) {
        repulse_right_dist_ = std::sqrt(min_d2_right);
        repulse_right_w_ = std::clamp(right_abs_y / std::max(repulse_right_dist_, 1e-6), 0.0, 1.0);
      } else {
        repulse_right_dist_ = std::numeric_limits<double>::max();
        repulse_right_w_ = 0.0;
      }
    } else {
      obstacle_detected_ = false;
      replan_requested_ = false;
      nearest_obstacle_dist_ = std::numeric_limits<double>::max();
      nearest_obstacle_dist_any_ = std::numeric_limits<double>::max();
      nearest_obstacle_dist_rear_ = std::numeric_limits<double>::max();
      repulse_left_dist_ = std::numeric_limits<double>::max();
      repulse_right_dist_ = std::numeric_limits<double>::max();
      repulse_left_w_ = 0.0;
      repulse_right_w_ = 0.0;
    }
  }

  // --------------------------------------------------------
  void speed_limit_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (msg->data >= 0.0 && (external_speed_limit_ < 0.0 ||
        !speed_limit_received_ ||
        (this->now() - last_speed_limit_time_).seconds() >= speed_limit_timeout_)) {
      RCLCPP_INFO(this->get_logger(),
        "External speed limit engaged: %.2f m/s", msg->data);
    }
    external_speed_limit_ = msg->data;
    last_speed_limit_time_ = this->now();
    speed_limit_received_ = true;
  }

  // --------------------------------------------------------
  // Publish + record. Steering prediction and the accel ramp need the last
  // commanded twist; the omega filter must not carry state across a stop.
  // --------------------------------------------------------
  void publish_cmd(const geometry_msgs::msg::Twist &cmd) {
    cmd_pub_->publish(cmd);
    last_cmd_speed_ = cmd.linear.x;
    last_cmd_omega_ = cmd.angular.z;
    if (cmd.linear.x == 0.0 && cmd.angular.z == 0.0) omega_filt_ = 0.0;
  }

  // --------------------------------------------------------
  // Safety bubble state machine. Returns true if it published a command
  // (hold or reverse) — the caller must return. Returns false only in the
  // forward-creep escape case: the caller runs pure pursuit with speed capped
  // at bubble_escape_speed_.
  // --------------------------------------------------------
  bool handle_bubble(geometry_msgs::msg::Twist &cmd) {
    auto now_t = this->now();

    if (!bubble_holding_ && !bubble_escaping_) {
      bubble_holding_ = true;
      bubble_hold_start_ = now_t;
      // An obstacle BESIDE the path is inside the bubble without geometrically
      // blocking the path, so the planner would never replan on its own.
      std_msgs::msg::Bool replan_msg;
      replan_msg.data = true;
      replan_pub_->publish(replan_msg);
      RCLCPP_WARN(this->get_logger(),
        "Safety bubble entered (obstacle at %.2f m) — holding, replan requested",
        nearest_obstacle_dist_any_);
    }

    if (!bubble_escaping_ &&
        (now_t - bubble_hold_start_).seconds() > bubble_escape_after_) {
      bubble_escaping_ = true;
      bubble_escape_start_ = now_t;
      RCLCPP_WARN(this->get_logger(),
        "Bubble held %.1fs with no way out — starting escape", bubble_escape_after_);
    }

    if (bubble_escaping_) {
      if ((now_t - bubble_escape_start_).seconds() > bubble_escape_max_duration_) {
        // Escape isn't converging — re-hold and retry after another wait.
        bubble_escaping_ = false;
        bubble_holding_ = true;
        bubble_hold_start_ = now_t;
        RCLCPP_WARN(this->get_logger(), "Escape timed out — holding again");
        publish_cmd(cmd);  // zero
        return true;
      }
      // Bearing of the nearest bubble point in the robot frame. RAW pose, not
      // the latency-predicted one — safety decisions use measured state.
      double dxn = nearest_any_x_ - pose_x_;
      double dyn = nearest_any_y_ - pose_y_;
      double local_xn = dxn * std::cos(pose_yaw_) + dyn * std::sin(pose_yaw_);
      if (local_xn > 0.0) {
        // Blocking point ahead — back straight away if the rear hemisphere is clear.
        if (nearest_obstacle_dist_rear_ > safety_stop_radius_) {
          cmd.linear.x = -bubble_escape_speed_;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Escape: reversing at %.2f m/s (obstacle %.2f m ahead)",
            bubble_escape_speed_, nearest_obstacle_dist_any_);
          publish_cmd(cmd);
          return true;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Boxed in: bubble points ahead AND behind — holding");
        publish_cmd(cmd);  // zero
        return true;
      }
      // Blocking point behind/beside — creeping forward along the path opens
      // the distance. Caller caps speed at bubble_escape_speed_.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Escape: creeping forward (obstacle %.2f m behind/beside)",
        nearest_obstacle_dist_any_);
      return false;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Safety stop: obstacle at %.2f m < %.2f m bubble — holding (escape in %.1fs)",
      nearest_obstacle_dist_any_, safety_stop_radius_,
      bubble_escape_after_ - (now_t - bubble_hold_start_).seconds());
    publish_cmd(cmd);  // zero
    return true;
  }

  // --------------------------------------------------------
  // Main control loop — pure pursuit
  // --------------------------------------------------------
  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    if (!pose_received_ || !path_active_ || path_.empty()) {
      publish_cmd(cmd);  // zero velocity
      return;
    }

    // Latency compensation: the newest pose is stale by NDT + transport time,
    // and the command needs ZMQ + ROS1 + motor time to act. Steer from where
    // the rover WILL be, not where it was measured.
    pred_x_ = pose_x_ + last_cmd_speed_ * std::cos(pose_yaw_) * pose_latency_;
    pred_y_ = pose_y_ + last_cmd_speed_ * std::sin(pose_yaw_) * pose_latency_;
    pred_yaw_ = pose_yaw_ + last_cmd_omega_ * pose_latency_;

    // Safety bubble with escape. Hysteresis: once escaping, stay in the state
    // machine until clear of radius + hysteresis so the boundary doesn't chatter.
    double escape_speed_cap = std::numeric_limits<double>::max();
    if (safety_stop_enabled_) {
      bool inside = nearest_obstacle_dist_any_ < safety_stop_radius_;
      bool cleared = nearest_obstacle_dist_any_ >
                     safety_stop_radius_ + bubble_escape_hysteresis_;
      if (inside || (bubble_escaping_ && !cleared)) {
        if (handle_bubble(cmd)) return;
        // Forward-creep escape: pure pursuit continues, slowly. In-place
        // rotation stays forbidden below (an obstacle within the bubble can
        // sit inside the chassis corners' swing circle).
        escape_speed_cap = bubble_escape_speed_;
      } else {
        if (bubble_escaping_) {
          // Escaped to a safe distance — replan from here; the pre-escape path
          // still hugs the obstacle and would lead straight back in.
          std_msgs::msg::Bool replan_msg;
          replan_msg.data = true;
          replan_pub_->publish(replan_msg);
          RCLCPP_INFO(this->get_logger(),
            "Escape complete (obstacle at %.2f m) — replanning from safe position",
            nearest_obstacle_dist_any_);
        }
        bubble_holding_ = false;
        bubble_escaping_ = false;
      }
    }

    // Check if we reached the goal
    double dx_goal = path_.back().pose.position.x - pred_x_;
    double dy_goal = path_.back().pose.position.y - pred_y_;
    double dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (dist_to_goal < goal_tolerance_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Goal reached!");
      path_active_ = false;
      publish_cmd(cmd);  // stop
      return;
    }

    // Obstacle handling with cooldown: after a fresh replanned path arrives the rover
    // is allowed to drive on it for obstacle_stop_cooldown_ seconds even while the
    // obstacle is still reported. The planner republishes every obstacle_replan_interval
    // while obstacles exist, so last_path_received_time_ keeps refreshing and the rover
    // drives freely around the obstacle. The stop only re-engages if replanning fails.
    double path_age = (this->now() - last_path_received_time_).seconds();
    bool in_cooldown = (path_age < obstacle_stop_cooldown_);
    bool obstacle_stop_active = enable_obstacle_stop_ && obstacle_detected_ && !in_cooldown;

    if (obstacle_stop_active) {
      publish_cmd(cmd);  // stop

      if (!replan_requested_) {
        double obstacle_duration = (this->now() - obstacle_start_time_).seconds();
        if (obstacle_duration > obstacle_stop_timeout_) {
          // Obstacle has persisted long enough without a fresh path — request one
          std_msgs::msg::Bool replan_msg;
          replan_msg.data = true;
          replan_pub_->publish(replan_msg);
          replan_requested_ = true;
          RCLCPP_INFO(this->get_logger(),
            "Obstacle persisted %.1fs without fresh path — replan requested", obstacle_duration);
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Obstacle detected — stopped (path_age=%.1fs > cooldown=%.1fs, %.1fs / %.1fs before replan)",
            path_age, obstacle_stop_cooldown_, obstacle_duration, obstacle_stop_timeout_);
        }
      }
      return;
    }

    // Speed-scaled lookahead: long on fast straights (low steering gain — no
    // weave), short at low speed (tight tracking through detours, where the
    // obstacle slowdown has already reduced speed).
    cur_lookahead_ = std::clamp(lookahead_time_ * std::max(last_cmd_speed_, 0.0),
                                lookahead_dist_, lookahead_max_);

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
    // Transform lookahead point to robot frame (latency-compensated pose)
    double dx = lx - pred_x_;
    double dy = ly - pred_y_;

    // Rotate into robot's local frame
    double local_x = dx * std::cos(-pred_yaw_) - dy * std::sin(-pred_yaw_);
    double local_y = dx * std::sin(-pred_yaw_) + dy * std::cos(-pred_yaw_);

    double ld_sq = local_x * local_x + local_y * local_y;
    double ld = std::sqrt(ld_sq);

    if (ld < 0.01) {
      publish_cmd(cmd);
      return;
    }

    // Heading error to the lookahead point in robot frame
    double heading_error = std::atan2(local_y, local_x);

    // Rotate in place when the heading error is too large to track cleanly with
    // forward motion — this produces tight pivots instead of wide arcs.
    if (std::abs(heading_error) > in_place_rotation_threshold_) {
      if (escape_speed_cap < max_linear_speed_) {
        // During bubble escape an obstacle sits within the chassis corners'
        // swing circle — pivoting could clip it. Hold and wait for the
        // replanned path instead.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Escape: pivot needed but obstacle too close to rotate — holding");
        publish_cmd(cmd);  // zero
        return;
      }
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(
        in_place_kp_ * heading_error,
        -max_angular_velocity_, max_angular_velocity_);
      omega_filt_ = cmd.angular.z;  // keep the filter continuous across modes
      publish_cmd(cmd);
      return;
    }

    // Curvature = 2 * y / L^2  (pure pursuit)
    double curvature = 2.0 * local_y / ld_sq;

    // Differential drive: angular velocity = curvature * linear speed
    // (NOT Ackermann atan formula — Nova Carter uses diff drive /cmd_vel)
    double speed = max_linear_speed_;

    // Slow down near goal
    if (dist_to_goal < cur_lookahead_) {
      speed = std::min(speed, min_linear_speed_ + (dist_to_goal / cur_lookahead_) *
              (max_linear_speed_ - min_linear_speed_));
    }

    // Distance-proportional speed reduction: scale speed linearly between
    // obstacle_slowdown_dist (full speed) and obstacle_min_speed_dist (min speed).
    // Forward-facing filter in obstacle_callback ensures this only fires for
    // obstacles ahead of the rover, not ones it's already passing.
    if (obstacle_detected_ && nearest_obstacle_dist_ < obstacle_slowdown_dist_) {
      double t = (nearest_obstacle_dist_ - obstacle_min_speed_dist_) /
                 (obstacle_slowdown_dist_ - obstacle_min_speed_dist_);
      t = std::clamp(t, 0.0, 1.0);
      double obstacle_speed = min_linear_speed_ + t * (max_linear_speed_ - min_linear_speed_);
      speed = std::min(speed, obstacle_speed);
      RCLCPP_DEBUG(this->get_logger(),
        "Obstacle proximity: dist=%.2fm → speed capped at %.2f m/s", nearest_obstacle_dist_, speed);
    }

    // Forward-creep escape cap (inactive outside the bubble state machine)
    speed = std::min(speed, escape_speed_cap);

    double omega = curvature * speed;
    omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

    // Reduce linear speed when rotating hard
    double abs_omega = std::abs(omega);
    if (abs_omega > speed_reduction_omega_) {
      double ratio = 1.0 - (abs_omega - speed_reduction_omega_) /
                     (max_angular_velocity_ - speed_reduction_omega_ + 1e-6);
      ratio = std::clamp(ratio, 0.0, 1.0);
      speed = min_linear_speed_ + ratio * (max_linear_speed_ - min_linear_speed_);
      speed = std::min(speed, escape_speed_cap);
      // Recompute omega with reduced speed
      omega = curvature * speed;
      omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);
    }

    // External speed limit (ZED human governor): cap the commanded speed while
    // a fresh, non-negative /speed_limit is in effect. Applied AFTER the
    // hard-rotation block above — that block SETS speed (it can raise it back
    // above an earlier cap), so the external limit must come last to be
    // authoritative. Omega is recomputed so the arc stays the same. Stale
    // (> speed_limit_timeout) or negative values are ignored (fail-open).
    if (speed_limit_received_ && external_speed_limit_ >= 0.0 &&
        (this->now() - last_speed_limit_time_).seconds() < speed_limit_timeout_ &&
        speed > external_speed_limit_) {
      speed = external_speed_limit_;
      omega = std::clamp(curvature * speed, -max_angular_velocity_, max_angular_velocity_);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "External speed limit active: %.2f m/s", external_speed_limit_);
    }

    // Obstacle shy-away: bias steering away from the nearest measured point on
    // each side once it intrudes inside repulsion_radius. Weighted by how abeam
    // the point is — a dead-ahead point gets no lateral push (the slowdown /
    // replan / bubble own that case), which also prevents sign-flip dither.
    // Opposite sides cancel, so a symmetric gap is driven straight through.
    double repulse = 0.0;
    if (repulse_right_dist_ < repulsion_radius_)
      repulse += repulsion_gain_ * repulse_right_w_ * (repulsion_radius_ - repulse_right_dist_);
    if (repulse_left_dist_ < repulsion_radius_)
      repulse -= repulsion_gain_ * repulse_left_w_ * (repulsion_radius_ - repulse_left_dist_);
    if (repulse != 0.0) {
      repulse = std::clamp(repulse, -repulsion_max_omega_, repulsion_max_omega_);
      omega = std::clamp(omega + repulse, -max_angular_velocity_, max_angular_velocity_);
    }

    // Low-pass the steering command: the pose updates at 10 Hz with cm-level
    // jitter while this loop reruns at 50 Hz — unfiltered, that jitter goes
    // straight to the wheels as left/right dither.
    double alpha = control_dt_ / (omega_smoothing_tau_ + control_dt_);
    omega_filt_ += alpha * (omega - omega_filt_);
    omega = std::clamp(omega_filt_, -max_angular_velocity_, max_angular_velocity_);

    // Forward acceleration ramp; deceleration stays instant (safety).
    double max_step = max_linear_accel_ * control_dt_;
    double prev_speed = std::max(last_cmd_speed_, 0.0);
    if (speed > prev_speed + max_step) speed = prev_speed + max_step;

    cmd.linear.x = speed;
    cmd.angular.z = omega;

    publish_cmd(cmd);
  }

  // --------------------------------------------------------
  void update_closest_index() {
    double min_dist = std::numeric_limits<double>::max();
    size_t best = closest_idx_;

    // Search forward from current closest (don't go backwards on path)
    size_t search_end = std::min(closest_idx_ + 50, path_.size());
    for (size_t i = closest_idx_; i < search_end; ++i) {
      double dx = path_[i].pose.position.x - pred_x_;
      double dy = path_[i].pose.position.y - pred_y_;
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
    // Walk along path from closest_idx_ until we find a point >= cur_lookahead_ away
    for (size_t i = closest_idx_; i < path_.size(); ++i) {
      double dx = path_[i].pose.position.x - pred_x_;
      double dy = path_[i].pose.position.y - pred_y_;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist >= cur_lookahead_) {
        // Interpolate between i-1 and i for smoother tracking
        if (i > 0) {
          double dx_prev = path_[i-1].pose.position.x - pred_x_;
          double dy_prev = path_[i-1].pose.position.y - pred_y_;
          double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev);

          if (dist_prev < cur_lookahead_) {
            double seg_dx = path_[i].pose.position.x - path_[i-1].pose.position.x;
            double seg_dy = path_[i].pose.position.y - path_[i-1].pose.position.y;
            double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

            if (seg_len > 1e-6) {
              double t = (cur_lookahead_ - dist_prev) / (dist - dist_prev);
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
