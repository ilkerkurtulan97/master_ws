#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <string>

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
    this->declare_parameter<int>("min_consecutive_scans", 3);
    this->declare_parameter<double>("ground_filter_height", 0.2);

    // Get parameters
    std::string map_path = this->get_parameter("static_map_path").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    activation_distance_ = this->get_parameter("activation_distance").as_double();
    fov_angle_deg_ = this->get_parameter("fov_angle_deg").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_int();
    min_consecutive_scans_ = this->get_parameter("min_consecutive_scans").as_int();
    ground_filter_height_ = this->get_parameter("ground_filter_height").as_double();

    if (map_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "static_map_path parameter is required!");
      return;
    }

    // --------------------------------------------------------
    // TF2 buffer and listener (uses the full TF tree from bag)
    // --------------------------------------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
      2*inflation_radius_+1, 2*inflation_radius_+1, 2*inflation_radius_+1);

    // --------------------------------------------------------
    // Subscribers
    // --------------------------------------------------------
    std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VoxelSubtractionNode::cloud_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 10,
      std::bind(&VoxelSubtractionNode::pose_callback, this, std::placeholders::_1));

    // --------------------------------------------------------
    // Publishers
    // --------------------------------------------------------
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dynamic_obstacles_markers", 10);

    obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_obstacles_cloud", 10);

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
      "Using TF2 for cloud transformation to 'map' frame");
  }

private:
  // Parameters
  double voxel_size_;
  double activation_distance_;
  double fov_angle_deg_;
  double min_z_;
  double max_z_;
  int inflation_radius_;
  int min_consecutive_scans_;
  double ground_filter_height_;

  // Voxel data
  VoxelSet raw_static_voxels_;
  VoxelSet inflated_static_voxels_;
  std::size_t static_voxels_raw_count_ = 0;

  // Temporal filtering: track consecutive voxel appearances
  using VoxelCountMap = std::unordered_map<VoxelKey, int, VoxelKeyHash>;
  VoxelCountMap voxel_persistence_;

  // Current robot pose (for activation zone check)
  bool pose_received_ = false;
  Eigen::Isometry3d robot_pose_ = Eigen::Isometry3d::Identity();

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
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
          for (int dz = -inflation_radius_; dz <= inflation_radius_; dz++) {
            inflated.insert({v.x + dx, v.y + dy, v.z + dz});
          }
        }
      }
    }
    inflated_static_voxels_ = std::move(inflated);
  }

  // --------------------------------------------------------
  // Pose callback (still used for activation zone direction)
  // --------------------------------------------------------
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = Eigen::Isometry3d::Identity();

    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
    );
    robot_pose_.rotate(q);
    robot_pose_.translation() = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
    );

    pose_received_ = true;
  }

  // --------------------------------------------------------
  // Point cloud callback — the main processing pipeline
  // --------------------------------------------------------
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No pose received yet, skipping cloud");
      return;
    }

    // --------------------------------------------------------
    // Transform the entire cloud to map frame using TF2
    // This uses the full chain: lidar -> base_link -> odom -> map
    // --------------------------------------------------------
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    bool already_in_map = (msg->header.frame_id == "map");

    if (!already_in_map) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          "map", msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(*msg, cloud_in_map, transform);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF2 transform failed: %s", ex.what());
        return;
      }
    } else {
      cloud_in_map = *msg;
    }

    // Convert transformed cloud to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_in_map, *cloud);

    // Robot position and forward direction (for activation zone)
    Eigen::Vector3d robot_pos = robot_pose_.translation();
    Eigen::Matrix3d rot = robot_pose_.rotation();
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
    VoxelSet dynamic_voxels_in_zone;

    for (const auto &pt : cloud->points) {
      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;

      Eigen::Vector3d p_map(pt.x, pt.y, pt.z);

      // Absolute z filter (safety net)
      if (p_map.z() < min_z_ || p_map.z() > max_z_) continue;

      // Relative ground filter: skip points near/below robot base (ground)
      if (p_map.z() < ground_z_threshold) continue;

      VoxelKey vk = point_to_voxel(p_map.x(), p_map.y(), p_map.z(), voxel_size_);

      // Skip if part of inflated static map
      if (inflated_static_voxels_.count(vk)) continue;

      // Check activation zone
      Eigen::Vector3d diff = p_map - robot_pos;
      diff.z() = 0.0;
      double dist = diff.norm();

      if (dist > 0.1 && dist < activation_distance_) {
        double dot = forward_dir.dot(diff.normalized());
        if (dot > cos_half_fov) {
          dynamic_voxels_in_zone.insert(vk);
        }
      }
    }

    // --------------------------------------------------------
    // Temporal filtering: require consecutive scan appearances
    // Voxels that disappear for even one frame get their count reset.
    // --------------------------------------------------------
    VoxelCountMap new_persistence;
    for (const auto &vk : dynamic_voxels_in_zone) {
      auto it = voxel_persistence_.find(vk);
      if (it != voxel_persistence_.end()) {
        new_persistence[vk] = it->second + 1;
      } else {
        new_persistence[vk] = 1;
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
    // Publish results
    // --------------------------------------------------------
    if (confirmed_dynamic.empty()) {
      publish_empty_markers(msg->header.stamp);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
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
