#!/usr/bin/env python3
"""
Relay /chassis/odom → /robotPose with the lidar-to-chassis offset applied.

The dynamic_obstacle_detector expects /robotPose to carry the lidar's world
pose so it can correctly transform lidar points into the map frame.
After the OmniGraph fix, /robotPose carries the chassis_link pose instead.
front_3d_lidar sits at [-0.232, 0, +0.526] m from chassis_link (identity rotation),
so we apply that offset here in the chassis body frame.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

# front_3d_lidar position in chassis_link frame (meters)
LIDAR_IN_CHASSIS = np.array([-0.232, 0.0, 0.526])


def quat_rotate(q, v):
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    return R @ v


class LidarPoseRelay(Node):
    def __init__(self):
        super().__init__('lidar_pose_relay')
        self.sub = self.create_subscription(Odometry, '/chassis/odom', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/robotPose', 10)
        self.get_logger().info('lidar_pose_relay ready: /chassis/odom → /robotPose (+lidar offset)')

    def cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        offset_world = quat_rotate((q.x, q.y, q.z, q.w), LIDAR_IN_CHASSIS)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'World'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = msg.pose.pose.position.x + offset_world[0]
        out.pose.pose.position.y = msg.pose.pose.position.y + offset_world[1]
        out.pose.pose.position.z = msg.pose.pose.position.z + offset_world[2]
        out.pose.pose.orientation = msg.pose.pose.orientation
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LidarPoseRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
