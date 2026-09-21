#!/usr/bin/env python3
"""
cmd_vel type bridge: mbf_mesh_nav -> Isaac Sim Nova Carter.

WHY: move_base_flex publishes the velocity command as geometry_msgs/TwistStamped
on /mbf_mesh_nav/cmd_vel, but Isaac Sim's Nova Carter differential drive (like the
working master_ws drive_controller) subscribes to geometry_msgs/Twist on /cmd_vel.
Mismatched types never connect, so the rover "ignores velocity commands" and MBF
aborts. This node strips the header and republishes Twist on /cmd_vel.

Glue lives here in mesh_nav_isaac_sim, never in master_ws (read-only).

Run:  python3 cmd_vel_relay.py --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            TwistStamped, "/mbf_mesh_nav/cmd_vel", self.cb, 10)
        self.get_logger().info(
            "cmd_vel_relay: /mbf_mesh_nav/cmd_vel (TwistStamped) -> /cmd_vel (Twist)")

    def cb(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
