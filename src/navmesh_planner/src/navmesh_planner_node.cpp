#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "navmesh_planner/navmesh_loader.hpp"
#include "navmesh_planner/coordinate_utils.hpp"
#include "navmesh_planner/tilecache_helpers.hpp"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_map>
#include <string>

class NavMeshPlannerNode : public rclcpp::Node {
public:
  NavMeshPlannerNode() : Node("navmesh_planner_node") {
    // --------------------------------------------------------
    // Parameters
    // --------------------------------------------------------
    this->declare_parameter<std::string>("navmesh_path", "");
    this->declare_parameter<std::string>("pose_topic", "/pcl_pose");
    this->declare_parameter<std::string>("message_typ", "PoseWithCovariance");
    this->declare_parameter<double>("poly_search_extent_x", 2.0);
    this->declare_parameter<double>("poly_search_extent_y", 1.0);
    this->declare_parameter<double>("poly_search_extent_z", 2.0);
    this->declare_parameter<double>("snap_vextent", 1.0);
    this->declare_parameter<int>("max_path_polys", 2048);
    this->declare_parameter<int>("max_straight_path", 256);
    this->declare_parameter<double>("waypoint_spacing", 0.0);

    // dtTileCache clustering parameters
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 3);
    this->declare_parameter<int>("max_cluster_size", 5000);
    this->declare_parameter<double>("obstacle_radius_default", 0.3);
    this->declare_parameter<double>("cylinder_padding", 0.5);
    this->declare_parameter<double>("height_padding", 0.55);
    this->declare_parameter<double>("obstacle_height_min", 0.5);
    this->declare_parameter<double>("obstacle_decay_s", 0.5);
    this->declare_parameter<double>("obstacle_replan_interval", 0.2);
    this->declare_parameter<double>("obstacle_floor_extend", 0.5);
    this->declare_parameter<double>("obstacle_position_threshold", 0.15);
    this->declare_parameter<double>("obstacle_radius_shrink_rate", 0.03);
    this->declare_parameter<double>("obstacle_geom_recarve_threshold", 0.08);
    this->declare_parameter<double>("obstacle_min_rover_dist", 1.5);
    this->declare_parameter<double>("obstacle_max_radius", 1.0);
    this->declare_parameter<double>("path_block_clearance", 0.25);

    std::string navmesh_path = this->get_parameter("navmesh_path").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    std::string message_typ = this->get_parameter("message_typ").as_string();
    max_path_polys_ = this->get_parameter("max_path_polys").as_int();
    max_straight_path_ = this->get_parameter("max_straight_path").as_int();
    waypoint_spacing_ = this->get_parameter("waypoint_spacing").as_double();

    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    obstacle_radius_default_ = this->get_parameter("obstacle_radius_default").as_double();
    cylinder_padding_ = this->get_parameter("cylinder_padding").as_double();
    height_padding_ = this->get_parameter("height_padding").as_double();
    obstacle_height_min_ = this->get_parameter("obstacle_height_min").as_double();
    obstacle_decay_s_ = this->get_parameter("obstacle_decay_s").as_double();
    obstacle_replan_interval_ = this->get_parameter("obstacle_replan_interval").as_double();
    floor_extend_ = static_cast<float>(this->get_parameter("obstacle_floor_extend").as_double());
    obstacle_position_threshold_ = this->get_parameter("obstacle_position_threshold").as_double();
    obstacle_radius_shrink_rate_ = this->get_parameter("obstacle_radius_shrink_rate").as_double();
    obstacle_geom_recarve_threshold_ = this->get_parameter("obstacle_geom_recarve_threshold").as_double();
    obstacle_min_rover_dist_ = this->get_parameter("obstacle_min_rover_dist").as_double();
    obstacle_max_radius_ = this->get_parameter("obstacle_max_radius").as_double();
    path_block_clearance_ = this->get_parameter("path_block_clearance").as_double();

    // Detour search extents (Y-UP). snap_vextent overrides the Y component so
    // ramp scenes can use a larger vertical search without changing X/Z.
    double ext_x = this->get_parameter("poly_search_extent_x").as_double();
    double ext_z = this->get_parameter("poly_search_extent_z").as_double();
    double snap_v = this->get_parameter("snap_vextent").as_double();
    extents_[0] = static_cast<float>(ext_x);
    extents_[1] = static_cast<float>(snap_v);
    extents_[2] = static_cast<float>(ext_z);

    // --------------------------------------------------------
    // Load NavMesh (auto-detect MSET vs TSET)
    // --------------------------------------------------------
    if (navmesh_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "navmesh_path parameter is required!");
      return;
    }

    tc_alloc_ = new LinearAllocator(256 * 1024);  // 256 KB; 32 KB caused silent OOM during tile rebuild
    tc_comp_ = new FastLZCompressor();
    tc_proc_ = new MeshProcess();

    NavMeshLoadResult load_result = loadNavMeshAuto(navmesh_path, tc_alloc_, tc_comp_, tc_proc_);
    nav_mesh_ = load_result.navMesh;
    tile_cache_ = load_result.tileCache;

    if (!nav_mesh_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load navmesh: %s", navmesh_path.c_str());
      return;
    }

    int tile_count = 0;
    for (int i = 0; i < nav_mesh_->getMaxTiles(); ++i) {
      const dtNavMesh* const_mesh = nav_mesh_;
      const dtMeshTile* tile = const_mesh->getTile(i);
      if (tile && tile->header && tile->dataSize > 0) tile_count++;
    }

    if (tile_cache_) {
      RCLCPP_INFO(this->get_logger(),
        "TSET NavMesh loaded: %d tiles, dtTileCache active (dynamic obstacles enabled) from %s",
        tile_count, navmesh_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "MSET NavMesh loaded: %d tiles from %s (static only, dynamic obstacles NOT supported)",
        tile_count, navmesh_path.c_str());
    }

    nav_query_ = dtAllocNavMeshQuery();
    if (dtStatusFailed(nav_query_->init(nav_mesh_, 2048))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init dtNavMeshQuery");
      return;
    }

    // --------------------------------------------------------
    // Subscribers
    // --------------------------------------------------------
    if (message_typ == "Odometry") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10,
        std::bind(&NavMeshPlannerNode::odom_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Pose mode: Odometry (Isaac Sim)");
    } else {
      pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 10,
        std::bind(&NavMeshPlannerNode::pose_cov_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Pose mode: PoseWithCovariance");
    }

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&NavMeshPlannerNode::goal_callback, this, std::placeholders::_1));

    replan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/replan_request", 10,
      std::bind(&NavMeshPlannerNode::replan_callback, this, std::placeholders::_1));

    if (tile_cache_) {
      obstacle_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/dynamic_obstacles_cloud", 10,
        std::bind(&NavMeshPlannerNode::obstacle_cloud_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to /dynamic_obstacles_cloud for obstacle injection");
    }

    // --------------------------------------------------------
    // Publishers
    // --------------------------------------------------------
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/planned_path_markers", 10);
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navmesh_obstacles_markers", 10);
    // transient_local = latched: late subscribers (e.g. navmesh_gui) receive the last message
    auto latched_qos = rclcpp::QoS(1).transient_local();
    navmesh_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/navmesh_polys", latched_qos);

    RCLCPP_INFO(this->get_logger(),
      "NavMesh planner ready. Pose topic: '%s', waiting for /goal_pose...",
      pose_topic.c_str());

    // Wall timer: publishes /navmesh_polys every 1 s regardless of sim clock state.
    // Using wall time avoids the use_sim_time=true / clock-not-started race that
    // causes this->now() to return {0,0}, which makes the sim-time rate limiter
    // inside publish_navmesh_marker() block the very first publish forever.
    navmesh_marker_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { if (nav_mesh_) publish_navmesh_marker(); });

    if (tile_cache_) log_tilecache_ybounds_once();
  }

  ~NavMeshPlannerNode() {
    if (nav_query_) dtFreeNavMeshQuery(nav_query_);
    if (tile_cache_) dtFreeTileCache(tile_cache_);
    if (nav_mesh_) dtFreeNavMesh(nav_mesh_);
    delete tc_alloc_;
    delete tc_comp_;
    delete tc_proc_;
  }

private:
  // --------------------------------------------------------
  // Tracked obstacle (for proximity-matched diff)
  // --------------------------------------------------------
  struct TrackedObstacle {
    double cx, cy, cz_center;
    float radius;      // geometry currently CARVED into the tile cache
    float height;
    // Smoothed measurement envelope: grows instantly on larger measurements
    // (safety), shrinks by at most obstacle_radius_shrink_rate_ per update
    // (stability). The carve is only refreshed when the envelope drifts more
    // than obstacle_geom_recarve_threshold_ from the carved geometry — raw
    // per-frame cluster noise (beams catching/missing edges, smear voxels)
    // no longer reaches the navmesh, killing the carve→replan→wiggle churn.
    float sm_radius;
    float sm_height;
    rclcpp::Time last_seen;
    // When this obstacle's carve was last (re)added to the tile cache. The
    // geometric path-block check only considers obstacles carved AFTER the
    // current path was planned — a fresh path is consistent with all existing
    // carves by construction, so older obstacles must not re-block it.
    rclcpp::Time last_carve;
  };

  // Detour objects
  dtNavMesh* nav_mesh_ = nullptr;
  dtNavMeshQuery* nav_query_ = nullptr;
  dtQueryFilter filter_;
  float extents_[3];

  // dtTileCache
  dtTileCache* tile_cache_ = nullptr;
  LinearAllocator* tc_alloc_ = nullptr;
  FastLZCompressor* tc_comp_ = nullptr;
  MeshProcess* tc_proc_ = nullptr;
  std::unordered_map<dtObstacleRef, TrackedObstacle> tracked_obstacles_;

  // Parameters
  int max_path_polys_;
  int max_straight_path_;
  double waypoint_spacing_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double obstacle_radius_default_;
  double cylinder_padding_;
  double height_padding_;
  double obstacle_height_min_;
  double obstacle_decay_s_;
  double obstacle_replan_interval_;
  float floor_extend_;
  double obstacle_position_threshold_;
  double obstacle_min_rover_dist_;
  double obstacle_max_radius_;
  double path_block_clearance_;
  double obstacle_radius_shrink_rate_;
  double obstacle_geom_recarve_threshold_;

  // When the current path was planned (epoch for the geometric block check).
  rclcpp::Time current_path_time_{0, 0, RCL_ROS_TIME};

  // Pose / goal state
  bool pose_received_ = false;
  double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0;
  bool goal_active_ = false;
  double goal_x_ = 0, goal_y_ = 0, goal_z_ = 0;

  // Current published path — stored in both frames.
  // current_path_dt_  : Detour Y-up frame, used by is_path_blocked().
  // current_path_ros_ : ROS message, re-published as heartbeat so the drive
  //                     controller's path_age stays fresh without triggering a
  //                     new plan (which would cause left/right oscillation).
  std::vector<std::array<float, 3>> current_path_dt_;
  nav_msgs::msg::Path current_path_ros_;

  rclcpp::Time last_obstacle_replan_time_{0, 0, RCL_ROS_TIME};
  bool prev_had_obstacles_ = false;
  // True if is_path_blocked() returned true on the previous drain cycle.
  // Used to allow an immediate first-detection replan while throttling
  // subsequent replans for the same continuous blockage.
  bool path_was_blocked_ = false;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr navmesh_marker_pub_;
  rclcpp::TimerBase::SharedPtr navmesh_marker_timer_;

  // --------------------------------------------------------
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_z_ = msg->pose.pose.position.z;
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
    // RViz 2D Goal Pose always sends Z=0; use the robot's current Z so
    // findNearestPoly searches near the actual floor level, not 2m above it.
    goal_z_ = (std::abs(msg->pose.position.z) < 0.01) ? pose_z_ : msg->pose.position.z;
    goal_active_ = true;
    current_path_dt_.clear();
    current_path_ros_.poses.clear();
    path_was_blocked_ = false;

    RCLCPP_INFO(this->get_logger(),
      "Goal received: (%.2f, %.2f, %.2f) -> planning from (%.2f, %.2f, %.2f)",
      goal_x_, goal_y_, goal_z_, pose_x_, pose_y_, pose_z_);

    plan_and_publish();
  }

  void replan_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    if (!pose_received_ || !goal_active_) return;
    current_path_dt_.clear();
    current_path_ros_.poses.clear();
    path_was_blocked_ = false;
    plan_and_publish();
  }

  // --------------------------------------------------------
  // Dynamic obstacle cloud callback
  // --------------------------------------------------------
  void obstacle_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "obstacle_cloud_callback fired: w=%u h=%u tc=%p nm=%p",
      msg->width, msg->height,
      static_cast<void*>(tile_cache_), static_cast<void*>(nav_mesh_));
    if (!tile_cache_ || !nav_mesh_) return;

    rclcpp::Time now_t = this->now();
    bool has_obstacles = (msg->width * msg->height > 0);
    bool obstacle_cleared = !has_obstacles && prev_had_obstacles_;

    double elapsed = (now_t - last_obstacle_replan_time_).seconds();
    bool throttle_ok = (elapsed >= obstacle_replan_interval_);

    // Cluster the cloud into candidate cylinders
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // POINT-LEVEL chassis exclusion (before clustering). The rover's own body
    // returns sit 0.58-0.85 m from the pose point (pose = LiDAR, mounted off-center;
    // measured on the real rover). Dropping whole *clusters* by centroid distance is
    // fragile: when chassis points merge with a nearby object the merged centroid
    // either escapes the exclusion (huge cylinder carved over the rover -> start
    // snaps away) or drags a real obstacle inside it (person on the path silently
    // dropped). Removing the points themselves makes both cases impossible.
    if (pose_received_ && !cloud->empty()) {
      const double excl2 = obstacle_min_rover_dist_ * obstacle_min_rover_dist_;
      size_t kept = 0;
      for (const auto& p : cloud->points) {
        double dx = p.x - pose_x_;
        double dy = p.y - pose_y_;
        if (dx * dx + dy * dy >= excl2) {
          cloud->points[kept++] = p;
        }
      }
      cloud->points.resize(kept);
      cloud->width = static_cast<uint32_t>(kept);
      cloud->height = 1;
      cloud->is_dense = true;
    }

    struct Candidate { double cx, cy, cz_center; float radius, height; };
    std::vector<Candidate> candidates;

    if (!cloud->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      tree->setInputCloud(cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud);
      ec.extract(cluster_indices);

      candidates.reserve(cluster_indices.size());
      for (const auto& indices : cluster_indices) {
        double cx = 0, cy = 0;
        double cz_min = std::numeric_limits<double>::max();
        double cz_max = std::numeric_limits<double>::lowest();
        for (int idx : indices.indices) {
          cx += (*cloud)[idx].x;
          cy += (*cloud)[idx].y;
          double z = (*cloud)[idx].z;
          if (z < cz_min) cz_min = z;
          if (z > cz_max) cz_max = z;
        }
        int n = static_cast<int>(indices.indices.size());
        cx /= n;
        cy /= n;

        // Robust radius: 95th-percentile point distance instead of the maximum.
        // The enclosing (max) radius is dominated by single outlier voxels (pose
        // smear, edge returns), which made the cylinder radius oscillate frame to
        // frame. The half-voxel term compensates for the detector's 0.1 m grid.
        std::vector<double> r2s;
        r2s.reserve(indices.indices.size());
        for (int idx : indices.indices) {
          double dx = (*cloud)[idx].x - cx;
          double dy = (*cloud)[idx].y - cy;
          r2s.push_back(dx*dx + dy*dy);
        }
        size_t k = static_cast<size_t>(0.95 * (r2s.size() - 1));
        std::nth_element(r2s.begin(), r2s.begin() + k, r2s.end());
        double encl_r = std::sqrt(r2s[k]) + 0.05;

        Candidate c;
        c.cx = cx;
        c.cy = cy;
        c.cz_center = 0.5 * (cz_min + cz_max);
        c.radius = static_cast<float>(std::max(encl_r + cylinder_padding_, obstacle_radius_default_));
        // Cap the carved radius: a mis-merged cluster (e.g. person + chair bridged
        // by cluster_tolerance) must never carve a hole bigger than a real object.
        // Better to under-carve one frame than to eat the whole corridor.
        if (c.radius > obstacle_max_radius_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Cluster radius %.2f m clamped to %.2f m (likely merged clusters — "
            "consider lowering cluster_tolerance)", c.radius, obstacle_max_radius_);
          c.radius = static_cast<float>(obstacle_max_radius_);
        }
        c.height = static_cast<float>(std::max(cz_max - cz_min + height_padding_, obstacle_height_min_));
        candidates.push_back(c);
      }
    }

    // NOTE: chassis exclusion now happens at POINT level before clustering (above).
    // No centroid-based candidate dropping here — a cluster built purely from
    // points outside the exclusion disc is a real obstacle even if its centroid
    // happens to fall near the rover.

    // Proximity-matched diff against tracked obstacles
    int adds = 0, removes = 0, refresh = 0;
    std::vector<bool> matched(candidates.size(), false);

    for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end(); ) {
      int best = -1;
      double best_d2 = std::numeric_limits<double>::max();
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (matched[i]) continue;
        double dx = candidates[i].cx - it->second.cx;
        double dy = candidates[i].cy - it->second.cy;
        double d2 = dx*dx + dy*dy;
        double tol = std::max<double>(candidates[i].radius, it->second.radius);
        if (d2 < tol * tol && d2 < best_d2) {
          best_d2 = d2;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        TrackedObstacle& tr = it->second;
        // Asymmetric geometry smoothing: adopt LARGER measurements instantly
        // (never under-carve a growing/approaching obstacle), but let the envelope
        // shrink only obstacle_radius_shrink_rate_ per update. Raw cluster noise
        // (±20-30% per frame from 16-beam sampling + smear voxels) collapses onto
        // a stable near-max envelope instead of re-carving the navmesh every frame.
        const float shrink = static_cast<float>(obstacle_radius_shrink_rate_);
        const Candidate& c = candidates[best];
        tr.sm_radius = (c.radius >= tr.sm_radius)
          ? c.radius : std::max(c.radius, tr.sm_radius - shrink);
        tr.sm_height = (c.height >= tr.sm_height)
          ? c.height : std::max(c.height, tr.sm_height - shrink);

        // Re-carve only when the SMOOTHED envelope has drifted from what is
        // actually carved, or the obstacle really moved. The carve is the
        // expensive, path-invalidating operation — starve it of noise.
        double d_xy = std::sqrt(best_d2);
        const float geom_thresh = static_cast<float>(obstacle_geom_recarve_threshold_);
        bool similar =
          d_xy < obstacle_position_threshold_ &&
          std::fabs(tr.sm_radius - tr.radius) < geom_thresh &&
          std::fabs(tr.sm_height - tr.height) < 2.0f * geom_thresh &&
          std::fabs(c.cz_center - tr.cz_center) < 0.3;
        matched[best] = true;
        if (similar) {
          tr.last_seen = now_t;
          ++refresh;
          ++it;
          continue;
        }
        // Real drift — re-carve below using the smoothed geometry (not the raw
        // measurement), so the new carve inherits the stable envelope.
        candidates[best].radius = tr.sm_radius;
        candidates[best].height = tr.sm_height;
        tile_cache_->removeObstacle(it->first);
        ++removes;
        it = tracked_obstacles_.erase(it);
        // Mark candidate as still pending add
        matched[best] = false;
        continue;
      }

      // Unmatched tracked obstacle: decay if stale
      double age = (now_t - it->second.last_seen).seconds();
      if (age > obstacle_decay_s_) {
        tile_cache_->removeObstacle(it->first);
        ++removes;
        it = tracked_obstacles_.erase(it);
      } else {
        ++it;
      }
    }

    // Add unmatched candidates
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (matched[i]) continue;
      const Candidate& c = candidates[i];
      float pos[3];
      rosToDetour(c.cx, c.cy, c.cz_center, pos);
      pos[1] -= c.height * 0.5f;  // cylinder BASE = cz_min - height_padding/2 in Detour Y-UP

      // The tile bmax[1] (navmesh floor in Detour Y) may be fractionally BELOW the computed
      // base, causing dtOverlapBounds to return false → ntouched=0 → no carving.
      // Extend the base downward by floor_extend_ and add the same amount to the height
      // so the obstacle top stays at the same position.
      float obs_height = c.height + floor_extend_;
      pos[1] -= floor_extend_;

      RCLCPP_INFO(this->get_logger(),
        "addObstacle: ROS(%.2f,%.2f,%.2f) → Detour base(%.2f,%.2f,%.2f) r=%.2f h=%.2f",
        c.cx, c.cy, c.cz_center, pos[0], pos[1], pos[2], c.radius, obs_height);

      dtObstacleRef ref = 0;
      dtStatus status = tile_cache_->addObstacle(pos, c.radius, obs_height, &ref);
      if (!dtStatusFailed(status) && ref != 0) {
        TrackedObstacle t{c.cx, c.cy, c.cz_center, c.radius, c.height,
                          c.radius, c.height, now_t, now_t};
        tracked_obstacles_.emplace(ref, t);
        ++adds;
        RCLCPP_INFO(this->get_logger(), "addObstacle OK: ref=%u", (unsigned)ref);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "addObstacle FAILED (status=0x%x ref=%u) — check maxObstacles in TSET bake params",
          status, (unsigned)ref);
      }
    }

    bool rebuilt = (adds > 0 || removes > 0);

    if (adds || removes) {
      RCLCPP_INFO(this->get_logger(),
        "Obstacles: +%d -%d ~%d (tracked=%zu, candidates=%zu)",
        adds, removes, refresh, tracked_obstacles_.size(), candidates.size());
    }

    // Sync the Detour navmesh whenever the tile cache changed. Drain is NOT
    // throttled: a slow drag still moves the carving every frame, so the navmesh
    // must stay in sync with the physical cylinder position.
    if (rebuilt || obstacle_cleared) {
      if (!update_tile_cache()) {
        RCLCPP_WARN(this->get_logger(),
          "TileCache failed to drain (not upToDate after %d iters) — skipping replan", 256);
        prev_had_obstacles_ = has_obstacles;
        return;
      }
      if (rebuilt) publish_navmesh_marker();
    }

    publish_obstacle_markers();

    // Validate the committed path against the (now up-to-date) navmesh on EVERY
    // obstacle update — not only when the tile cache changed. A stationary
    // obstacle that settles onto the path produces no further add/remove events,
    // so a rebuilt-gated check would never re-evaluate the path. Sampling here
    // every callback keeps "object stops on the path" responsive.
    bool replanned = false;
    if (goal_active_) {
      bool path_blocked = is_path_blocked();
      // Replan immediately the first time the path becomes blocked so a suddenly
      // appearing obstacle is handled at once. While the path stays continuously
      // blocked, throttle to obstacle_replan_interval_ so the rover commits to a
      // side instead of oscillating.
      if (path_blocked && (!path_was_blocked_ || throttle_ok)) {
        RCLCPP_INFO(this->get_logger(),
          "Path blocked — replanning (tracked=%zu, newly_blocked=%s)",
          tracked_obstacles_.size(), path_was_blocked_ ? "no" : "yes");
        plan_and_publish();
        last_obstacle_replan_time_ = now_t;
        replanned = true;
      }
      path_was_blocked_ = path_blocked;
    }

    // Heartbeat: if we didn't just replan, re-publish the current path with a
    // fresh stamp (throttled) so the drive controller's path_age stays fresh and
    // it keeps driving the committed route instead of triggering an obstacle stop.
    if (!replanned && throttle_ok && goal_active_ && !current_path_ros_.poses.empty()) {
      current_path_ros_.header.stamp = this->now();
      path_pub_->publish(current_path_ros_);
      publish_path_marker(current_path_ros_.poses);
      last_obstacle_replan_time_ = now_t;
    }

    prev_had_obstacles_ = has_obstacles;
  }

  // --------------------------------------------------------
  // Drain tile cache rebuild queue. Returns true if upToDate.
  // --------------------------------------------------------
  bool update_tile_cache() {
    if (!tile_cache_ || !nav_mesh_) return true;
    const float dt = 1.0f / 60.0f;
    const int max_iters = 256;
    bool upToDate = false;
    int iters_used = 0;
    for (int iter = 0; iter < max_iters && !upToDate; ++iter) {
      dtStatus status = tile_cache_->update(dt, nav_mesh_, &upToDate);
      iters_used = iter + 1;
      if (dtStatusFailed(status)) {
        RCLCPP_WARN(this->get_logger(), "dtTileCache::update() failed at iter %d", iter);
        return false;
      }
    }
    // iters_used==1 && upToDate==true means no tiles were rebuilt (ntouched=0)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TileCache drain: %d iters (1 = no tiles touched by obstacle)", iters_used);
    return upToDate;
  }

  // --------------------------------------------------------
  // Print tile cache Y-bounds for one call (diagnostic).
  // --------------------------------------------------------
  void log_tilecache_ybounds_once() {
    if (!tile_cache_) return;
    int count = tile_cache_->getTileCount();
    RCLCPP_INFO(this->get_logger(), "TileCache: maxTiles=%d, scanning for bmin/bmax Y...", count);
    int printed = 0;
    for (int i = 0; i < count && printed < 5; ++i) {
      const dtCompressedTile* t = tile_cache_->getTile(i);
      if (!t || !t->header || t->dataSize <= 0) continue;
      RCLCPP_INFO(this->get_logger(),
        "  tile[%d] tx=%d ty=%d bmin=(%.2f,%.2f,%.2f) bmax=(%.2f,%.2f,%.2f)",
        i, t->header->tx, t->header->ty,
        t->header->bmin[0], t->header->bmin[1], t->header->bmin[2],
        t->header->bmax[0], t->header->bmax[1], t->header->bmax[2]);
      ++printed;
    }
  }

  // --------------------------------------------------------
  // Returns true if the current committed path has any point inside an
  // obstacle carving (findNearestPoly fails with tight extents → no walkable
  // poly at that point). Samples every 0.4 m along each segment so a cylinder
  // placed between polygon-corner waypoints is caught. An empty path is treated
  // as blocked so the first plan always fires.
  // --------------------------------------------------------
  bool is_path_blocked() {
    if (current_path_dt_.empty()) return true;
    if (!nav_query_) return true;

    // 1) GEOMETRIC check: does any tracked obstacle cylinder overlap the path
    // corridor? Exact 2D point-to-segment distance in the ROS frame — immune to
    // tile-cache rasterization, carve resolution, and sample spacing. The navmesh
    // query below can miss a person whose ~0.45 m carved hole (shrunk by cell
    // rasterization) still leaves a walkable poly within the 0.35 m search box;
    // this check cannot.
    if (!tracked_obstacles_.empty() && !current_path_ros_.poses.empty()) {
      const auto& pp = current_path_ros_.poses;
      for (const auto& kv : tracked_obstacles_) {
        const TrackedObstacle& o = kv.second;
        // Only obstacles carved AFTER the current path was planned can block it.
        // The path was planned around every carve that existed at plan time; the
        // rasterized hole is slightly SMALLER than o.radius, so without this
        // epoch test the fresh detour that hugs the hole re-triggers "blocked"
        // forever (3 Hz replan loop -> left/right path flapping -> wiggle).
        if (o.last_carve <= current_path_time_) continue;
        const double block_r = static_cast<double>(o.radius) + path_block_clearance_;
        const double br2 = block_r * block_r;
        for (size_t i = 0; i + 1 < pp.size(); ++i) {
          const double ax = pp[i].pose.position.x,   ay = pp[i].pose.position.y;
          const double bx = pp[i+1].pose.position.x, by = pp[i+1].pose.position.y;
          const double vx = bx - ax, vy = by - ay;
          const double wx = o.cx - ax, wy = o.cy - ay;
          const double vv = vx * vx + vy * vy;
          const double t = (vv > 1e-9) ? std::clamp((wx * vx + wy * vy) / vv, 0.0, 1.0) : 0.0;
          const double dx = wx - t * vx, dy = wy - t * vy;
          if (dx * dx + dy * dy < br2) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "Path blocked (geometric): obstacle at (%.2f, %.2f) r=%.2f within %.2f m of path",
              o.cx, o.cy, o.radius, block_r);
            return true;
          }
        }
      }
    }

    // 2) Navmesh query check: catches carved regions along the path from anything
    // not covered above (e.g. an obstacle removed from tracking whose carve is
    // still draining, or start/goal areas losing their polys).
    const float check_ext[3] = {0.35f, 0.5f, 0.35f};
    const float sample_step  = 0.4f;
    dtPolyRef ref = 0;
    float nearest[3];

    for (size_t i = 0; i < current_path_dt_.size(); ++i) {
      nav_query_->findNearestPoly(current_path_dt_[i].data(), check_ext, &filter_, &ref, nearest);
      if (ref == 0) return true;

      if (i + 1 < current_path_dt_.size()) {
        float ax = current_path_dt_[i][0];
        float ay = current_path_dt_[i][1];
        float az = current_path_dt_[i][2];
        float bx = current_path_dt_[i+1][0];
        float by = current_path_dt_[i+1][1];
        float bz = current_path_dt_[i+1][2];
        float dx = bx - ax, dy = by - ay, dz = bz - az;
        float seg_len = std::sqrt(dx*dx + dy*dy + dz*dz);
        int steps = static_cast<int>(std::ceil(seg_len / sample_step));
        for (int s = 1; s < steps; ++s) {
          float t = static_cast<float>(s) / static_cast<float>(steps);
          float pt[3] = {ax + t*dx, ay + t*dy, az + t*dz};
          dtPolyRef rref = 0;
          nav_query_->findNearestPoly(pt, check_ext, &filter_, &rref, nearest);
          if (rref == 0) return true;
        }
      }
    }
    return false;
  }

  // --------------------------------------------------------
  // Main planning function
  // --------------------------------------------------------
  void plan_and_publish() {
    if (!nav_mesh_ || !nav_query_) return;

    float start_dt[3], goal_dt[3];
    rosToDetour(pose_x_, pose_y_, pose_z_, start_dt);
    rosToDetour(goal_x_, goal_y_, goal_z_, goal_dt);

    dtPolyRef start_ref = 0, goal_ref = 0;
    float start_nearest[3], goal_nearest[3];

    nav_query_->findNearestPoly(start_dt, extents_, &filter_, &start_ref, start_nearest);
    if (start_ref == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "findNearestPoly failed for start (%.2f, %.2f, %.2f)",
        pose_x_, pose_y_, pose_z_);
      return;
    }

    nav_query_->findNearestPoly(goal_dt, extents_, &filter_, &goal_ref, goal_nearest);
    if (goal_ref == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "findNearestPoly failed for goal (%.2f, %.2f, %.2f)",
        goal_x_, goal_y_, goal_z_);
      return;
    }

    std::vector<dtPolyRef> poly_path(max_path_polys_);
    int poly_count = 0;
    dtStatus status = nav_query_->findPath(
      start_ref, goal_ref, start_nearest, goal_nearest,
      &filter_, poly_path.data(), &poly_count, max_path_polys_);

    if (dtStatusFailed(status) || poly_count == 0) {
      RCLCPP_ERROR(this->get_logger(), "findPath failed or returned 0 polygons");
      return;
    }

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

    // Persist path in Detour frame so is_path_blocked() can validate it on
    // subsequent obstacle updates without re-running findPath.
    current_path_dt_.clear();
    current_path_dt_.reserve(straight_count);
    for (int i = 0; i < straight_count; ++i) {
      current_path_dt_.push_back({
        straight_path[i * 3 + 0],
        straight_path[i * 3 + 1],
        straight_path[i * 3 + 2]
      });
    }

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

    if (waypoint_spacing_ > 0.0 && waypoints.size() >= 2) {
      waypoints = interpolate_waypoints(waypoints, waypoint_spacing_);
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    path_msg.poses = waypoints;
    path_pub_->publish(path_msg);
    current_path_ros_ = path_msg;  // cache for heartbeat re-publish
    current_path_time_ = this->now();  // re-arms the geometric block check

    publish_path_marker(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "Path planned: %d polys -> %d waypoints", poly_count, straight_count);
  }

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
  void publish_path_marker(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
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

  // --------------------------------------------------------
  // Debug overlay: each tracked obstacle as a translucent red cylinder at the
  // ROS centroid, plus a yellow point at the Detour→ROS round-trip position so
  // any axis-mapping bug shows up as a visible offset in RViz.
  // --------------------------------------------------------
  void publish_obstacle_markers() {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.ns = "navmesh_obstacles";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto& kv : tracked_obstacles_) {
      const TrackedObstacle& o = kv.second;

      visualization_msgs::msg::Marker cyl;
      cyl.header.frame_id = "map";
      cyl.header.stamp = this->now();
      cyl.ns = "navmesh_obstacles";
      cyl.id = id++;
      cyl.type = visualization_msgs::msg::Marker::CYLINDER;
      cyl.action = visualization_msgs::msg::Marker::ADD;
      cyl.pose.position.x = o.cx;
      cyl.pose.position.y = o.cy;
      cyl.pose.position.z = o.cz_center;
      cyl.pose.orientation.w = 1.0;
      cyl.scale.x = 2.0 * o.radius;
      cyl.scale.y = 2.0 * o.radius;
      cyl.scale.z = o.height;
      cyl.color.r = 1.0;
      cyl.color.g = 0.2;
      cyl.color.b = 0.2;
      cyl.color.a = 0.4;
      arr.markers.push_back(cyl);

      // Round-trip check: ROS -> Detour (with base offset) -> ROS
      float pos_dt[3];
      rosToDetour(o.cx, o.cy, o.cz_center, pos_dt);
      pos_dt[1] -= o.height * 0.5f;
      double rx, ry, rz;
      detourToRos(pos_dt, rx, ry, rz);

      visualization_msgs::msg::Marker pt;
      pt.header.frame_id = "map";
      pt.header.stamp = this->now();
      pt.ns = "navmesh_obstacles";
      pt.id = id++;
      pt.type = visualization_msgs::msg::Marker::SPHERE;
      pt.action = visualization_msgs::msg::Marker::ADD;
      pt.pose.position.x = rx;
      pt.pose.position.y = ry;
      pt.pose.position.z = rz;  // expected: cz_center - height/2 (the cylinder base)
      pt.pose.orientation.w = 1.0;
      pt.scale.x = pt.scale.y = pt.scale.z = 0.15;
      pt.color.r = 1.0;
      pt.color.g = 1.0;
      pt.color.b = 0.0;
      pt.color.a = 1.0;
      arr.markers.push_back(pt);
    }

    obstacle_marker_pub_->publish(arr);
  }

  // --------------------------------------------------------
  // Publish navmesh detail mesh as a TRIANGLE_LIST so the carved cylinder hole
  // is visible in RViz.
  //
  // WHY throttle: each message is a full TRIANGLE_LIST (thousands of points).
  // RViz deletes the old geometry and recreates it on every message — during
  // that brief delete→add window the affected tiles appear absent ("white dent").
  // Sending this at obstacle-update rate (up to ~10 Hz due to centroid jitter
  // triggering frequent remove+re-add cycles) makes these gaps persistent and
  // visible. Publishing at ≤1 Hz is more than enough to show carving results.
  // --------------------------------------------------------
  void publish_navmesh_marker() {
    if (!nav_mesh_ || !navmesh_marker_pub_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "navmesh";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.r = 0.2;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.35;

    // Fan-triangulate each polygon using its base vertices.
    // Detail mesh (tile->detailMeshes) is absent in TSET tilecache navmeshes
    // baked by bake_tcbin.py, so we cannot use it — triCount is always 0.
    const dtNavMesh* mesh = nav_mesh_;
    for (int t = 0; t < mesh->getMaxTiles(); ++t) {
      const dtMeshTile* tile = mesh->getTile(t);
      if (!tile || !tile->header) continue;

      for (int i = 0; i < tile->header->polyCount; ++i) {
        const dtPoly& p = tile->polys[i];
        if (p.getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
        if (p.vertCount < 3) continue;

        // Fan triangulation: (v0,v1,v2), (v0,v2,v3), ..., (v0,vN-2,vN-1)
        const float* v0 = &tile->verts[p.verts[0] * 3];
        double r0x, r0y, r0z;
        detourToRos(v0, r0x, r0y, r0z);

        for (int k = 1; k + 1 < p.vertCount; ++k) {
          const float* va = &tile->verts[p.verts[k]     * 3];
          const float* vb = &tile->verts[p.verts[k + 1] * 3];
          double rax, ray, raz, rbx, rby, rbz;
          detourToRos(va, rax, ray, raz);
          detourToRos(vb, rbx, rby, rbz);

          geometry_msgs::msg::Point p0, pa, pb;
          p0.x = r0x; p0.y = r0y; p0.z = r0z;
          pa.x = rax; pa.y = ray; pa.z = raz;
          pb.x = rbx; pb.y = rby; pb.z = rbz;
          marker.points.push_back(p0);
          marker.points.push_back(pa);
          marker.points.push_back(pb);
        }
      }
    }

    navmesh_marker_pub_->publish(marker);
    RCLCPP_INFO_ONCE(this->get_logger(),
      "navmesh_polys published: %zu triangles", marker.points.size() / 3);
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
