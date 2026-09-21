#!/usr/bin/env python3
"""
Anchor the spawn-relative Isaac TF tree to the absolute /chassis/odom pose.

PROBLEM: fix_odom_definitive.py makes the /chassis/odom *topic* report the
front_3d_lidar's ABSOLUTE Bitlab world pose, but Isaac's *TF tree*
(World -> odom -> base_link) is still SPAWN-RELATIVE. mesh_nav reads the robot
pose from TF (map -> base_link), so a single static map->World offset can't
align it - it misses the spawn YAW, which rotates the mesh relative to the
lidar (ramp looks displaced, lidar doesn't overlay).

FIX: publish map->World = (map_T_lidar) . inv(World_T_lidar), where
  map_T_lidar  = the absolute lidar pose from /chassis/odom (frame 'map')
  World_T_lidar = the spawn-relative lidar pose from Isaac's TF.
Using the SAME frame (front_3d_lidar) on both sides cancels the spawn
translation AND rotation exactly, so the whole chain
map -> World -> odom -> base_link -> sensors becomes absolute and overlays the
mesh. Replaces the static map_to_world_tf node. No master_ws files touched.

Run: python3 map_world_anchor.py --ros-args -p use_sim_time:=true
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    X, Y, Z = x * s, y * s, z * s
    wx, wy, wz = w * X, w * Y, w * Z
    xx, xy, xz = x * X, x * Y, x * Z
    yy, yz, zz = y * Y, y * Z, z * Z
    M = np.eye(4)
    M[0, 0] = 1 - (yy + zz); M[0, 1] = xy - wz;      M[0, 2] = xz + wy
    M[1, 0] = xy + wz;       M[1, 1] = 1 - (xx + zz); M[1, 2] = yz - wx
    M[2, 0] = xz - wy;       M[2, 1] = yz + wx;       M[2, 2] = 1 - (xx + yy)
    return M


def mat_to_quat(M):
    m = M[:3, :3]
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    return x, y, z, w


class MapWorldAnchor(Node):
    def __init__(self):
        super().__init__('map_world_anchor')
        self.lidar_frame = self.declare_parameter('lidar_frame', 'front_3d_lidar').value
        self.world_frame = self.declare_parameter('world_frame', 'World').value
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        # STATIC broadcaster: the spawn anchor is constant, so a time-independent
        # static transform applies at all timestamps. This makes the lidar->mesh
        # overlay immune to /clock-vs-Isaac-TF desync (a dynamic, stamped map->World
        # only matches the lidar at a matching timestamp, so any clock drift shifts
        # the lidar off the mesh). We only (re)broadcast when the computed anchor
        # changes meaningfully, so /tf_static isn't flooded.
        self.br = tf2_ros.StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/chassis/odom', self.cb, 10)
        self._last = None
        self.get_logger().info(
            f"map_world_anchor: anchoring '{self.world_frame}' to absolute /chassis/odom "
            f"via lidar frame '{self.lidar_frame}' (static)")

    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T_ml = quat_to_mat(q.x, q.y, q.z, q.w)          # map_T_lidar (absolute)
        T_ml[:3, 3] = [p.x, p.y, p.z]
        try:
            tf = self.buf.lookup_transform(self.world_frame, self.lidar_frame, rclpy.time.Time())
        except Exception:
            return  # TF not ready yet
        t = tf.transform.translation
        r = tf.transform.rotation
        T_wl = quat_to_mat(r.x, r.y, r.z, r.w)          # World_T_lidar (spawn-relative)
        T_wl[:3, 3] = [t.x, t.y, t.z]
        T_mW = T_ml @ np.linalg.inv(T_wl)               # map_T_World (constant anchor)
        qx, qy, qz, qw = mat_to_quat(T_mW)
        t = (float(T_mW[0, 3]), float(T_mW[1, 3]), float(T_mW[2, 3]))

        # Only (re)broadcast when the anchor changes meaningfully (>1 cm or >0.3 deg)
        # so the static transform isn't republished at the odom rate.
        if self._last is not None:
            dt = np.linalg.norm(np.array(t) - np.array(self._last[0]))
            dq = 1.0 - abs(np.dot(self._last[1], (qx, qy, qz, qw)))
            if dt < 0.01 and dq < 2e-5:
                return
        self._last = (t, (qx, qy, qz, qw))

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id = self.world_frame
        out.transform.translation.x = t[0]
        out.transform.translation.y = t[1]
        out.transform.translation.z = t[2]
        out.transform.rotation.x = float(qx)
        out.transform.rotation.y = float(qy)
        out.transform.rotation.z = float(qz)
        out.transform.rotation.w = float(qw)
        self.br.sendTransform(out)
        yaw = np.degrees(np.arctan2(T_mW[1, 0], T_mW[0, 0]))
        self.get_logger().info(
            f"map->World anchored (static): t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) yaw={yaw:.2f} deg")


def main():
    rclpy.init()
    node = MapWorldAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
