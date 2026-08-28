#!/usr/bin/env python3
"""
cmd_vel_zmq_sender.py — ROS2 node (runs on the Jetson Orin, ROS2 Humble)

master_ws (recast/Detour) variant. drive_controller_node publishes
geometry_msgs/Twist on /cmd_vel; this node ships that command over ZeroMQ to the
Innok (ROS1), where cmd_vel_zmq_receiver_ros1.py republishes it to the wheels.

This is the reverse of your lidar_zmq_bridge and mirrors its pattern exactly:
ZeroMQ PUB, CONFLATE=1 + SNDHWM=1 (only the latest command matters), NOBLOCK send,
compact little-endian struct payload.

WIRE-COMPATIBLE with the mesh_navigation bridge (same magic/format/port), so the
Innok needs only ONE receiver no matter which nav stack you run on the Jetson:
  magic 4s b"CMDV" | stamp_sec u32 | stamp_nsec u32 | linear xyz f64 | angular xyz f64
  -> "<4s 2I 6d" = 60 bytes.

NOTE: master_ws is read-only — this is a NEW standalone file, it imports/edits nothing
in the existing packages.

Run:
  source /opt/ros/humble/setup.bash
  source /home/ilker/master_ws/install/setup.bash
  python3 cmd_vel_zmq_sender.py --ros-args \
    -p input_topic:=/cmd_vel -p bind_address:=tcp://0.0.0.0:5556

Params:
  ~input_topic   (string, default /cmd_vel)      <- drive_controller publishes plain Twist here
  ~bind_address  (string, default tcp://0.0.0.0:5556)
"""
import struct
import rclpy
import zmq
from rclpy.node import Node
from geometry_msgs.msg import Twist

MAGIC = b"CMDV"
MSG_FMT = "<4s 2I 6d"            # 60 bytes — identical to the mesh_nav bridge
MSG_SIZE = struct.calcsize(MSG_FMT)


class CmdVelZmqSender(Node):
    def __init__(self):
        super().__init__("cmd_vel_zmq_sender")
        self.input_topic = self.declare_parameter("input_topic", "/cmd_vel").value
        bind_address = self.declare_parameter("bind_address", "tcp://0.0.0.0:5556").value

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.bind(bind_address)

        # master_ws drive_controller publishes plain geometry_msgs/Twist (NOT stamped).
        self.create_subscription(Twist, self.input_topic, self.cb, 10)
        self.get_logger().info(
            f"cmd_vel_zmq_sender (master_ws): PUB bound {bind_address}; "
            f"forwarding {self.input_topic} (Twist)")

    def cb(self, tw: Twist):
        now = self.get_clock().now().to_msg()
        payload = struct.pack(
            MSG_FMT, MAGIC, now.sec, now.nanosec,
            tw.linear.x, tw.linear.y, tw.linear.z,
            tw.angular.x, tw.angular.y, tw.angular.z)
        try:
            self.sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass  # no subscriber yet
        except Exception as e:
            self.get_logger().warn(f"send error: {e}", throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self.sock.close(0)
            self.ctx.term()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelZmqSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
