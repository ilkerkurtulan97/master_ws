#!/usr/bin/env python3
"""
cmd_vel_zmq_sender.py — ROS2 node (runs on the Jetson Orin, ROS2 Humble)

The reverse direction of your lidar_zmq_bridge: mesh_navigation computes a velocity
command on the Jetson; this node ships it over ZeroMQ to the Innok (ROS1) where
cmd_vel_zmq_receiver.py republishes it as geometry_msgs/Twist on the robot's /cmd_vel.

Mirrors lidar_zmq_sender.py exactly:
  - ZeroMQ PUB socket, CONFLATE=1 + SNDHWM=1 (only the latest command matters; old
    commands are dropped — correct for control), send with NOBLOCK.
  - Compact little-endian struct payload (no ROS serialization on the wire).

Wire format (little-endian, 60 bytes, MUST match cmd_vel_zmq_receiver.py):
  magic       : 4s     b"CMDV"  (guards against a mis-wired port)
  stamp_sec   : uint32
  stamp_nsec  : uint32
  linear  x,y,z : float64 x3
  angular x,y,z : float64 x3
For a differential-drive rover only linear.x and angular.z are used downstream,
but all six are sent for generality.

MBF publishes geometry_msgs/TwistStamped on /mbf_mesh_nav/cmd_vel; this node
subscribes there directly, so on the real robot it REPLACES cmd_vel_relay.py.

Run (folded into the real-robot launch):
  python3 cmd_vel_zmq_sender.py --ros-args \
    -p bind_address:=tcp://0.0.0.0:5556 \
    -p input_topic:=/mbf_mesh_nav/cmd_vel

Params:
  ~input_topic   (string, default /mbf_mesh_nav/cmd_vel)
  ~input_stamped (bool,   default true   -> TwistStamped; false -> Twist)
  ~bind_address  (string, default tcp://0.0.0.0:5556)
"""
import struct
import rclpy
import zmq
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

MAGIC = b"CMDV"
MSG_FMT = "<4s 2I 6d"            # 4 + 8 + 48 = 60 bytes
MSG_SIZE = struct.calcsize(MSG_FMT)


class CmdVelZmqSender(Node):
    def __init__(self):
        super().__init__("cmd_vel_zmq_sender")
        self.input_topic = self.declare_parameter("input_topic", "/mbf_mesh_nav/cmd_vel").value
        self.input_stamped = self.declare_parameter("input_stamped", True).value
        bind_address = self.declare_parameter("bind_address", "tcp://0.0.0.0:5556").value

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.bind(bind_address)

        msg_type = TwistStamped if self.input_stamped else Twist
        self.create_subscription(msg_type, self.input_topic, self.cb, 10)
        self.get_logger().info(
            f"cmd_vel_zmq_sender: PUB bound {bind_address}; "
            f"forwarding {self.input_topic} ({'TwistStamped' if self.input_stamped else 'Twist'})")

    def cb(self, msg):
        tw = msg.twist if self.input_stamped else msg
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
