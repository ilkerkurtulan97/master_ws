#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>

#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <string>
#include <deque>
#include <limits>

// ============================================================
// Voxel key: integer 3D index into the voxel grid
// ============================================================
struct VoxelKey {
  int x, y, z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &k) const {
    std::size_t h = 0;
    h ^= std::hash<int>()(k.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

// ============================================================
// Convert a 3D point to its voxel key
// ============================================================
inline VoxelKey point_to_voxel(float x, float y, float z, double voxel_size) {
  return {
    static_cast<int>(std::floor(x / voxel_size)),
    static_cast<int>(std::floor(y / voxel_size)),
    static_cast<int>(std::floor(z / voxel_size))
  };
}

// ============================================================
// Main ROS2 Node
// ============================================================
class VoxelSubtractionNode : public rclcpp::Node {
public:
  VoxelSubtractionNode() : Node("voxel_subtraction_node") {
    // Declare parameters
    this->declare_parameter<std::string>("static_map_path", "");
    this->declare_parameter<double>("voxel_size", 0.2);
    this->declare_parameter<std::string>("cloud_topic", "/cloud");
    this->declare_parameter<std::string>("pose_topic", "/amcl_pose");
    this->declare_parameter<double>("activation_distance", 1.0);
    this->declare_parameter<double>("fov_angle_deg", 120.0);
    this->declare_parameter<double>("min_z", -0.3);
    this->declare_parameter<double>("max_z", 2.5);
    this->declare_parameter<int>("inflation_radius", 1);
    // Vertical inflation, separate from XY. NDT error is dominated by heading
    // (XY displacement grows with range) while z is well constrained by the
    // ground plane — but the FLOOR is itself a static surface, so vertical
    // inflation eats the bottom (z-radius * voxel) of every object standing on
    // it. At 3 (0.3 m) a 45 cm box survived only as a 1-voxel flickering
    // sliver and was missed half the time.
    this->declare_parameter<int>("inflation_radius_z", 1);
    this->declare_parameter<int>("min_consecutive_scans", 3);
    this->declare_parameter<double>("ground_filter_height", 0.2);
    this->declare_parameter<std::string>("message_typ", "PoseWithCovariance");

    // Number of consecutive empty frames before the obstacle cloud is cleared.
    // Lower = obstacles vanish faster after the real object leaves (more responsive),
    // higher = more robust against momentary detection dropouts. Was hardcoded 20.
    this->declare_parameter<int>("obstacle_clear_frames", 2);

    // Sensor-frame self-filter: drop points that fall inside the rover's own
    // chassis footprint (expressed in the LiDAR/sensor frame, before the map
    // transform). This removes the rover's body — which is "dynamic" because it
    // is not in the static PCD — without a large radial exclusion in the planner,
    // so real objects close to the rover can still be detected. Disabled by
    // default (zero-size box); measure your rover and enable on the real robot.
    this->declare_parameter<bool>("self_filter_enabled", false);
    this->declare_parameter<double>("self_filter_min_x", 0.0);
    this->declare_parameter<double>("self_filter_max_x", 0.0);
    this->declare_parameter<double>("self_filter_min_y", 0.0);
    this->declare_parameter<double>("self_filter_max_y", 0.0);
    this->declare_parameter<double>("self_filter_min_z", 0.0);
    this->declare_parameter<double>("self_filter_max_z", 0.0);

    // Get parameters
    std::string map_path = this->get_parameter("static_map_path").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    activation_distance_ = this->get_parameter("activation_distance").as_double();
    fov_angle_deg_ = this->get_parameter("fov_angle_deg").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_int();
    inflation_radius_z_ = this->get_parameter("inflation_radius_z").as_int();
    min_consecutive_scans_ = this->get_parameter("min_consecutive_scans").as_int();
    ground_filter_height_ = this->get_parameter("ground_filter_height").as_double();
    message_typ_ = this->get_parameter("message_typ").as_string();
    obstacle_clear_frames_ = this->get_parameter("obstacle_clear_frames").as_int();
    self_filter_enabled_ = this->get_parameter("self_filter_enabled").as_bool();
    self_filter_min_x_ = this->get_parameter("self_filter_min_x").as_double();
    self_filter_max_x_ = this->get_parameter("self_filter_max_x").as_double();
    self_filter_min_y_ = this->get_parameter("self_filter_min_y").as_double();
    self_filter_max_y_ = this->get_parameter("self_filter_max_y").as_double();
    self_filter_min_z_ = this->get_parameter("self_filter_min_z").as_double();
    self_filter_max_z_ = this->get_parameter("self_filter_max_z").as_double();

    if (map_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "static_map_path parameter is required!");
      return;
    }

    // --------------------------------------------------------
    // Step 1: Load and voxelize static map (ONCE)
    // --------------------------------------------------------
    load_and_voxelize_static_map(map_path);

    // --------------------------------------------------------
    // Step 2: Inflate static voxels (3x3x3 neighborhood)
    // --------------------------------------------------------
    inflate_static_voxels();

    RCLCPP_INFO(this->get_logger(),
      "Static map: %zu raw voxels -> %zu inflated voxels (voxel_size=%.2f, inflation=%dx%dx%d)",
      static_voxels_raw_count_, inflated_static_voxels_.size(), voxel_size_,
      2*inflation_radius_+1, 2*inflation_radius_+1, 2*inflation_radius_z_+1);

    // --------------------------------------------------------
    // Subscribers
    // --------------------------------------------------------
    std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();

    // depth 1: processing takes longer than the scan period, so a deeper queue
    // makes the node work through a permanent backlog of STALE scans (the pose
    // matched to an old scan lags reality -> rotation smear). Newest scan wins.
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VoxelSubtractionNode::cloud_callback, this, std::placeholders::_1));

    // Subscribe to pose based on message_typ parameter
    if (message_typ_ == "Odometry") {
      // Odometry mode: robotPose gives the lidar's pose in map frame directly
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10,
        std::bind(&VoxelSubtractionNode::odom_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
        "Pose mode: Odometry (pose = lidar position in map)");
    } else {
      // PoseWithCovariance mode (default — for localization packages like AMCL/NDT)
      // depth 50: pose messages must NEVER be dropped — scan-time matching needs
      // the pose stamped at each scan, and poses queue up while a cloud is being
      // processed. 50 msgs = 5 s of history at 10 Hz, matching the pose buffer.
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 50,
        std::bind(&VoxelSubtractionNode::pose_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
        "Pose mode: PoseWithCovariance");
    }

    // --------------------------------------------------------
    // Publishers
    // --------------------------------------------------------
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dynamic_obstacles_markers", 10);

    obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_obstacles_cloud", 10);

    // Live lidar transformed to map frame (for RViz alignment verification)
    lidar_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lidar_map_frame", 10);

    // Pre-temporal-filter dynamic candidates (debug: shows subtraction before persistence check)
    candidates_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_candidates_cloud", 10);

    // Static map as voxelized point cloud (default QoS, republish periodically for RViz)
    static_voxel_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/static_map_voxelized", 10);

    // Publish once at startup + every 5s for RViz
    publish_static_voxel_cloud();
    static_map_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&VoxelSubtractionNode::publish_static_voxel_cloud, this));

    RCLCPP_INFO(this->get_logger(),
      "Node ready. Listening on cloud='%s', pose='%s'",
      cloud_topic.c_str(), pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Activation: distance=%.1fm, FOV=%.0f deg",
      activation_distance_, fov_angle_deg_);
    RCLCPP_INFO(this->get_logger(),
      "Temporal filter: require %d consecutive scans before reporting obstacle",
      min_consecutive_scans_);
    RCLCPP_INFO(this->get_logger(),
      "Ground filter: ignore points below robot_z + %.2fm",
      ground_filter_height_);
    RCLCPP_INFO(this->get_logger(),
      "Transform: pose_topic position applied directly to cloud points");
  }

private:
  // Parameters
  double voxel_size_;
  double activation_distance_;
  double fov_angle_deg_;
  double min_z_;
  double max_z_;
  int inflation_radius_;
  int inflation_radius_z_;
  int min_consecutive_scans_;
  double ground_filter_height_;
  std::string message_typ_;
  bool self_filter_enabled_ = false;
  double self_filter_min_x_ = 0, self_filter_max_x_ = 0;
  double self_filter_min_y_ = 0, self_filter_max_y_ = 0;
  double self_filter_min_z_ = 0, self_filter_max_z_ = 0;

  // Voxel data
  VoxelSet raw_static_voxels_;
  VoxelSet inflated_static_voxels_;
  std::size_t static_voxels_raw_count_ = 0;

  // Temporal filtering: track consecutive voxel appearances
  using VoxelCountMap = std::unordered_map<VoxelKey, int, VoxelKeyHash>;
  VoxelCountMap voxel_persistence_;

  // Current robot/lidar pose
  bool pose_received_ = false;
  Eigen::Isometry3d robot_pose_ = Eigen::Isometry3d::Identity();
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};

  // Stamped pose history for scan-time matching. The NDT localizer stamps its
  // pose with the SCAN's timestamp, so transforming a scan with the pose whose
  // stamp matches removes the rotation smear caused by using the latest pose
  // (one-plus frames stale: at 1 rad/s that is 6+ deg -> static walls leak
  // through subtraction as phantom voxels while turning).
  std::deque<std::pair<rclcpp::Time, Eigen::Isometry3d>> pose_buffer_;
  // Scan waiting for its matching pose to arrive (NDT publishes the pose for
  // scan T shortly AFTER we receive scan T).
  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;

  // Frame counter for clock-independent diagnostic logging
  uint64_t diag_frame_ = 0;

  // Debounce: count consecutive frames with 0 confirmed obstacles before clearing the cloud.
  // Prevents oscillating replanning when detection momentarily drops out (e.g., LiDAR angle).
  // At ~20 Hz LiDAR, 20 frames ≈ 1 second of sustained absence before clearing.
  int consecutive_empty_frames_ = 0;
  int obstacle_clear_frames_ = 20;  // set from parameter in constructor

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidates_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_voxel_cloud_pub_;
  rclcpp::TimerBase::SharedPtr static_map_timer_;

  // --------------------------------------------------------
  // Load PCD and voxelize the static map
  // --------------------------------------------------------
  void load_and_voxelize_static_map(const std::string &path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", path.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded static map: %zu points", cloud->size());

    for (const auto &pt : cloud->points) {
      if (pt.z < min_z_ || pt.z > max_z_) continue;
      raw_static_voxels_.insert(point_to_voxel(pt.x, pt.y, pt.z, voxel_size_));
    }
    static_voxels_raw_count_ = raw_static_voxels_.size();
    inflated_static_voxels_ = raw_static_voxels_;
  }

  // --------------------------------------------------------
  // Inflate each static voxel by 1 in every direction (3x3x3)
  // --------------------------------------------------------
  void inflate_static_voxels() {
    VoxelSet inflated;
    inflated.reserve(inflated_static_voxels_.size() * 27);

    for (const auto &v : inflated_static_voxels_) {
      for (int dx = -inflation_radius_; dx <= inflation_radius_; dx++) {
        for (int dy = -inflation_radius_; dy <= inflation_radius_; dy++) {
          for (int dz = -inflation_radius_z_; dz <= inflation_radius_z_; dz++) {
            inflated.insert({v.x + dx, v.y + dy, v.z + dz});
          }
        }
      }
    }
    inflated_static_voxels_ = std::move(inflated);
  }

  // --------------------------------------------------------
  // Common pose update (shared by both callbacks)
  // --------------------------------------------------------
  void update_robot_pose(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp) {
    robot_pose_ = Eigen::Isometry3d::Identity();

    Eigen::Quaterniond q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z
    );
    robot_pose_.rotate(q);
    robot_pose_.translation() = Eigen::Vector3d(
      pose.position.x,
      pose.position.y,
      pose.position.z
    );

    pose_received_ = true;

    // Buffer for scan-time matching (dedupe repeats of the same stamp — the
    // localizer's publish timer can re-send the last pose).
    if (pose_buffer_.empty() || stamp != pose_buffer_.back().first) {
      pose_buffer_.emplace_back(stamp, robot_pose_);
      while (pose_buffer_.size() > 50) pose_buffer_.pop_front();
    }

    // A pose arrival may complete a scan that was waiting for it.
    if (pending_cloud_) {
      auto cloud = pending_cloud_;
      if (try_process(cloud) != PoseMatch::Defer) pending_cloud_.reset();
    }
  }

  // --------------------------------------------------------
  // Pose callback for PoseWithCovarianceStamped (localization mode)
  // --------------------------------------------------------
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    update_robot_pose(msg->pose.pose, rclcpp::Time(msg->header.stamp));
  }

  // --------------------------------------------------------
  // Odom callback for Odometry (Isaac Sim / direct odom mode)
  // --------------------------------------------------------
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    update_robot_pose(msg->pose.pose, rclcpp::Time(msg->header.stamp));
    last_odom_stamp_ = msg->header.stamp;
  }

  // --------------------------------------------------------
  // Pick the buffered pose closest to the scan's timestamp.
  //   Matched — pose within match_tol of the scan stamp: use it.
  //   Defer   — the matching pose has not arrived yet (pose stream is behind
  //             the scan by <= pose_wait): caller retries when a pose lands.
  //   Skip    — no pose anywhere near this scan's stamp. Processing it with a
  //             stale pose smears the scan during rotation (walls leak through
  //             static subtraction -> phantom obstacles), which is strictly
  //             worse than dropping one frame. Caller discards the scan.
  // --------------------------------------------------------
  enum class PoseMatch { Matched, Defer, Skip };

  PoseMatch select_pose_at(const rclcpp::Time &scan_stamp, Eigen::Isometry3d &out) {
    constexpr double match_tol_s = 0.06;   // "same scan" tolerance (< scan period)
    constexpr double pose_wait_s = 0.30;   // max time to wait for the pose stream

    const rclcpp::Time &newest = pose_buffer_.back().first;
    if (newest < scan_stamp) {
      if ((scan_stamp - newest).seconds() <= pose_wait_s) {
        return PoseMatch::Defer;  // matching pose still in flight
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Pose stream lags scan by %.2fs (> %.2fs wait) — dropping scan instead "
        "of processing with a stale pose",
        (scan_stamp - newest).seconds(), pose_wait_s);
      return PoseMatch::Skip;
    }

    double best_dt = std::numeric_limits<double>::max();
    for (auto it = pose_buffer_.rbegin(); it != pose_buffer_.rend(); ++it) {
      double dt = std::fabs((it->first - scan_stamp).seconds());
      if (dt < best_dt) { best_dt = dt; out = it->second; }
      else break;  // buffer is time-ordered; diffs only grow past the minimum
    }
    if (best_dt > match_tol_s) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No pose within %.2fs of scan stamp (best %.2fs) — dropping scan; "
        "check that localization stamps poses with the scan time",
        match_tol_s, best_dt);
      return PoseMatch::Skip;
    }
    return PoseMatch::Matched;
  }

  // --------------------------------------------------------
  // Point cloud callback — defer until the pose matching this scan's stamp
  // has arrived (NDT publishes it a few tens of ms after the scan), then
  // process with that pose. Costs ~one NDT latency of delay; removes the
  // rotation smear entirely.
  // --------------------------------------------------------
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pose_received_) {
      if (++diag_frame_ % 100 == 0) {
        RCLCPP_WARN(this->get_logger(), "No pose received yet, skipping cloud (frame %lu)", diag_frame_);
      }
      return;
    }
    if (pending_cloud_) {
      // Previous scan never got its pose (stream hiccup) — newest data wins.
      pending_cloud_.reset();
    }
    if (try_process(msg) == PoseMatch::Defer) {
      pending_cloud_ = msg;  // pose still in flight; retried on pose arrival
    }
  }

  PoseMatch try_process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    Eigen::Isometry3d pose_at_scan;
    PoseMatch m = select_pose_at(rclcpp::Time(msg->header.stamp), pose_at_scan);
    if (m == PoseMatch::Matched) {
      process_cloud(msg, pose_at_scan);
    }
    return m;
  }

  // --------------------------------------------------------
  // The main processing pipeline (pose = rover pose AT the scan's timestamp)
  // --------------------------------------------------------
  void process_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                     const Eigen::Isometry3d &pose_at_scan) {
    // Convert ROS msg to PCL (in sensor frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Robot position and forward direction (for activation zone)
    Eigen::Vector3d robot_pos = pose_at_scan.translation();
    Eigen::Matrix3d rot = pose_at_scan.rotation();
    Eigen::Vector3d forward_dir = rot.col(0);
    forward_dir.z() = 0.0;
    forward_dir.normalize();

    double half_fov_rad = (fov_angle_deg_ / 2.0) * M_PI / 180.0;
    double cos_half_fov = std::cos(half_fov_rad);

    // Relative ground threshold: ignore points below this z in map frame
    // Follows the robot's z, so z-drift in localization won't cause ground false positives
    double ground_z_threshold = robot_pos.z() + ground_filter_height_;

    // --------------------------------------------------------
    // For each point (now in map frame):
    //   1. Voxelize
    //   2. Subtract inflated static
    //   3. Check activation zone
    // --------------------------------------------------------
    int n_nan = 0, n_z = 0, n_ground = 0, n_static = 0, n_zone = 0, n_self = 0;

    VoxelSet dynamic_voxels_in_zone;

    // Collect all transformed points for the map-frame lidar topic (RViz debug)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_map->reserve(cloud->size());

    for (const auto &pt : cloud->points) {
      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) { n_nan++; continue; }

      // Self-filter: drop points inside the rover's chassis box (sensor frame,
      // before the map transform). Removes the rover's own body so it isn't
      // reported as a dynamic obstacle, without a large radial exclusion.
      if (self_filter_enabled_ &&
          pt.x >= self_filter_min_x_ && pt.x <= self_filter_max_x_ &&
          pt.y >= self_filter_min_y_ && pt.y <= self_filter_max_y_ &&
          pt.z >= self_filter_min_z_ && pt.z <= self_filter_max_z_) {
        n_self++;
        continue;
      }

      // Transform from lidar-local frame to map using the pose matched to this
      // scan's timestamp (NOT the latest pose — that lags during rotation).
      Eigen::Vector3d p_map = pose_at_scan * Eigen::Vector3d(pt.x, pt.y, pt.z);

      cloud_map->push_back(pcl::PointXYZ(p_map.x(), p_map.y(), p_map.z()));

      // Absolute z filter (safety net)
      if (p_map.z() < min_z_ || p_map.z() > max_z_) { n_z++; continue; }

      // Relative ground filter: skip points near/below robot base (ground)
      if (p_map.z() < ground_z_threshold) { n_ground++; continue; }

      VoxelKey vk = point_to_voxel(p_map.x(), p_map.y(), p_map.z(), voxel_size_);

      // Skip if part of inflated static map
      if (inflated_static_voxels_.count(vk)) { n_static++; continue; }

      // Check activation zone
      Eigen::Vector3d diff = p_map - robot_pos;
      diff.z() = 0.0;
      double dist = diff.norm();

      if (dist > 0.1 && dist < activation_distance_) {
        double dot = forward_dir.dot(diff.normalized());
        if (dot > cos_half_fov) {
          dynamic_voxels_in_zone.insert(vk);
        } else {
          n_zone++;
        }
      } else {
        n_zone++;
      }
    }

    // Publish lidar transformed to map frame (for RViz alignment check)
    {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*cloud_map, out);
      out.header.frame_id = "map";
      out.header.stamp = msg->header.stamp;
      lidar_map_pub_->publish(out);
    }

    // --------------------------------------------------------
    // Temporal filtering: decay-based persistence
    // Seen voxels: count +1 (capped at min_consecutive_scans_)
    // Unseen voxels: count -1; removed when count reaches 0
    // This means a confirmed voxel survives brief detection gaps
    // (e.g., rotating lidar beam missing a voxel for 1-2 scans)
    // without resetting its counter from scratch.
    // --------------------------------------------------------
    VoxelCountMap new_persistence;

    // Boost voxels detected this frame
    for (const auto &vk : dynamic_voxels_in_zone) {
      auto it = voxel_persistence_.find(vk);
      if (it != voxel_persistence_.end()) {
        new_persistence[vk] = std::min(it->second + 1, min_consecutive_scans_);
      } else {
        new_persistence[vk] = 1;
      }
    }

    // Decay voxels NOT detected this frame
    for (const auto &[vk, count] : voxel_persistence_) {
      if (new_persistence.find(vk) == new_persistence.end()) {
        if (count > 1) {
          new_persistence[vk] = count - 1;  // decay
        }
        // count == 1: voxel expires, not added
      }
    }

    voxel_persistence_ = std::move(new_persistence);

    // Only report voxels that persisted for enough consecutive scans
    VoxelSet confirmed_dynamic;
    for (const auto &[vk, count] : voxel_persistence_) {
      if (count >= min_consecutive_scans_) {
        confirmed_dynamic.insert(vk);
      }
    }

    // --------------------------------------------------------
    // Diagnostic log every 100 frames (~2.8s at 36Hz) — clock-independent
    // --------------------------------------------------------
    if (++diag_frame_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "[diag #%lu] total=%zu nan=%d self=%d z_filt=%d gnd_filt=%d static_filt=%d zone_filt=%d | "
        "pre_temporal=%zu confirmed=%zu | robot_z=%.2f gnd_thresh=%.2f",
        diag_frame_, cloud->size(), n_nan, n_self, n_z, n_ground, n_static, n_zone,
        dynamic_voxels_in_zone.size(), confirmed_dynamic.size(),
        robot_pos.z(), ground_z_threshold);
    }

    // Publish pre-temporal-filter candidates (debug: before persistence check)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cand_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cand_cloud->reserve(dynamic_voxels_in_zone.size());
      for (const auto &vk : dynamic_voxels_in_zone) {
        cand_cloud->push_back(pcl::PointXYZ(
          (vk.x + 0.5f) * static_cast<float>(voxel_size_),
          (vk.y + 0.5f) * static_cast<float>(voxel_size_),
          (vk.z + 0.5f) * static_cast<float>(voxel_size_)));
      }
      sensor_msgs::msg::PointCloud2 cand_out;
      pcl::toROSMsg(*cand_cloud, cand_out);
      cand_out.header.frame_id = "map";
      cand_out.header.stamp = msg->header.stamp;
      candidates_cloud_pub_->publish(cand_out);
    }

    // --------------------------------------------------------
    // Publish results
    // --------------------------------------------------------
    if (confirmed_dynamic.empty()) {
      publish_empty_markers(msg->header.stamp);
      consecutive_empty_frames_++;
      // Only publish empty cloud after sustained absence to avoid oscillating replanning.
      // Momentary detection gaps (LiDAR angle, motion) do NOT clear the navmesh obstacle.
      if (consecutive_empty_frames_ >= obstacle_clear_frames_) {
        publish_obstacle_cloud(confirmed_dynamic, msg->header.stamp);
      }
    } else {
      consecutive_empty_frames_ = 0;
      RCLCPP_INFO(this->get_logger(),
        "Dynamic obstacles confirmed: %zu voxels (candidates: %zu)",
        confirmed_dynamic.size(), dynamic_voxels_in_zone.size());
      publish_obstacle_markers(confirmed_dynamic, msg->header.stamp);
      publish_obstacle_cloud(confirmed_dynamic, msg->header.stamp);
    }
  }

  // --------------------------------------------------------
  // Publish dynamic obstacle voxels as MarkerArray (red cubes)
  // --------------------------------------------------------
  void publish_obstacle_markers(const VoxelSet &voxels,
                                 const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (const auto &vk : voxels) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = stamp;
      marker.ns = "dynamic_obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = (vk.x + 0.5) * voxel_size_;
      marker.pose.position.y = (vk.y + 0.5) * voxel_size_;
      marker.pose.position.z = (vk.z + 0.5) * voxel_size_;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = voxel_size_;
      marker.scale.y = voxel_size_;
      marker.scale.z = voxel_size_;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;

      marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(marker);
    }

    obstacle_marker_pub_->publish(marker_array);
  }

  void publish_empty_markers(const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = stamp;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    obstacle_marker_pub_->publish(marker_array);
  }

  void publish_obstacle_cloud(const VoxelSet &voxels,
                               const rclcpp::Time &stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(voxels.size());

    for (const auto &vk : voxels) {
      pcl::PointXYZ pt;
      pt.x = (vk.x + 0.5) * voxel_size_;
      pt.y = (vk.y + 0.5) * voxel_size_;
      pt.z = (vk.z + 0.5) * voxel_size_;
      cloud->push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = stamp;
    obstacle_cloud_pub_->publish(output);
  }

  void publish_static_voxel_cloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(raw_static_voxels_.size());

    for (const auto &vk : raw_static_voxels_) {
      pcl::PointXYZ pt;
      pt.x = (vk.x + 0.5) * voxel_size_;
      pt.y = (vk.y + 0.5) * voxel_size_;
      pt.z = (vk.z + 0.5) * voxel_size_;
      cloud->push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = this->now();
    static_voxel_cloud_pub_->publish(output);

    RCLCPP_INFO_ONCE(this->get_logger(),
      "Published voxelized static map: %zu voxels on /static_map_voxelized",
      raw_static_voxels_.size());
  }
};

// ============================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelSubtractionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
