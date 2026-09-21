#!/usr/bin/env python3
"""
replan_trigger: reroute the rover around dynamic obstacles WITHOUT relying on MBF's
(broken) continuous replanning.

WHY: in this MBF build, continuous replanning is off by default and runs unthrottled
(~1600 Hz) when armed, thrashing mbf. So we keep PLAN-ONCE (smooth driving) and trigger
a single fresh re-plan only when it's actually needed: when a dynamic-obstacle point
(from /dynamic_obstacles_cloud) lies within `block_dist` of the current planned path,
AHEAD of the rover. We then re-publish the active goal on /goal_pose; goal_bridge turns
that into a new MoveBase goal -> a fresh plan-once that routes around the obstacle (whose
mesh faces the ObstacleLayer has marked lethal). Throttled by `min_replan_period` so it
cannot thrash; self-limiting because once the path avoids the obstacle, nothing blocks it.

Run (folded into the launch): python3 replan_trigger.py --ros-args -p use_sim_time:=true
"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros


class ReplanTrigger(Node):
    def __init__(self):
        super().__init__('replan_trigger')
        self.block_dist = self.declare_parameter('block_dist', 0.6).value
        self.min_replan_period = self.declare_parameter('min_replan_period', 1.5).value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        self.buf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self.buf, self)
        self.goal = None     # active goal PoseStamped
        self.path = None     # Nx2 path (xy)
        self.obs = None      # Mx2 obstacle points (xy)
        self.last_replan = self.get_clock().now()
        self._reemit = False  # guard so we don't capture our own re-publish as a new goal

        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        latched = QoSProfile(depth=5)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Path, '/mbf_mesh_nav/path', self._on_path, latched)
        self.create_subscription(PointCloud2, '/dynamic_obstacles_cloud', self._on_obs, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "replan_trigger ready: reroute when an obstacle is within %.2f m of the path "
            "ahead of the rover (min %.1f s between replans)" % (self.block_dist, self.min_replan_period))

    def _on_goal(self, msg: PoseStamped):
        if self._reemit:           # ignore our own re-emit
            self._reemit = False
            return
        self.goal = msg
        self.get_logger().info("Active goal captured: (%.2f, %.2f)" % (msg.pose.position.x, msg.pose.position.y))

    def _on_path(self, msg: Path):
        self.path = (np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
                     if msg.poses else None)

    def _on_obs(self, msg: PointCloud2):
        a = np.array([[x, y] for x, y, z in pc2.read_points(msg, ('x', 'y', 'z'), skip_nans=True)], dtype=float)
        self.obs = a if len(a) else None

    def _tick(self):
        if self.goal is None or self.path is None or self.obs is None or len(self.path) < 2:
            return
        if (self.get_clock().now() - self.last_replan).nanoseconds * 1e-9 < self.min_replan_period:
            return
        try:
            tf = self.buf.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            rp = np.array([tf.transform.translation.x, tf.transform.translation.y])
        except Exception:
            return
        P, O = self.path, self.obs
        # for each obstacle: nearest path point distance + index
        diff = O[:, None, :] - P[None, :, :]          # M x N x 2
        d2 = (diff ** 2).sum(-1)                       # M x N
        nearest_idx = d2.argmin(1)
        nearest_dist = np.sqrt(d2.min(1))
        # robot's position along the path
        ri = int(((P - rp) ** 2).sum(1).argmin())
        # blocking = obstacle close to the path AND on the portion ahead of the rover
        blocking = (nearest_dist < self.block_dist) & (nearest_idx > ri + 1)
        if not blocking.any():
            return
        self.last_replan = self.get_clock().now()
        self._reemit = True
        g = PoseStamped()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose = self.goal.pose
        self.pub.publish(g)
        self.get_logger().info("Obstacle blocking path ahead (%d pts within %.2f m) -> re-issued goal to reroute"
                               % (int(blocking.sum()), self.block_dist))


def main():
    rclpy.init()
    node = ReplanTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
