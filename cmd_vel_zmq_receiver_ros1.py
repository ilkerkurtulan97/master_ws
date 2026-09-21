#!/usr/bin/env python3
"""
cmd_vel_zmq_receiver.py — ROS1 node (runs on the Innok robot computer)

Receives velocity commands from the Jetson over ZeroMQ (sent by
cmd_vel_zmq_sender.py) and republishes them as geometry_msgs/Twist on the robot's
drive topic. This is the missing return path noted in master_ws README2
("Add cmd_vel ZMQ bridge — same pattern as existing lidar bridge").

Mirrors lidar_zmq_bridge but reversed: here the ROBOT is the ZMQ SUB.
SAFETY: a watchdog publishes a ZERO Twist if no command arrives within
`~timeout` seconds (e.g. Jetson process died, network dropped, mesh_nav aborted)
so the rover does not run away on the last command. ALWAYS keep this enabled.

Wire format (little-endian, 60 bytes — MUST match cmd_vel_zmq_sender.py):
  magic 4s b"CMDV" | stamp_sec u32 | stamp_nsec u32 | linear xyz f64 | angular xyz f64

Place this in your catkin_ws (e.g. lidar_zmq_bridge/scripts/) and run:
  rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py \
    _connect_address:=tcp://<JETSON_IP>:5556 _output_topic:=/cmd_vel _timeout:=0.5

ROS params:
  ~connect_address (string, default tcp://127.0.0.1:5556)  <- set to the JETSON IP
  ~output_topic    (string, default /cmd_vel)
  ~timeout         (float,  default 0.5)   seconds before safety-stop
  ~rate            (float,  default 50.0)  Hz poll/publish loop
"""
import struct
import rospy
import zmq
from geometry_msgs.msg import Twist

MAGIC = b"CMDV"
MSG_FMT = "<4s 2I 6d"
MSG_SIZE = struct.calcsize(MSG_FMT)


class CmdVelZmqReceiver:
    def __init__(self):
        rospy.init_node("cmd_vel_zmq_receiver", anonymous=False)
        connect_address = rospy.get_param("~connect_address", "tcp://127.0.0.1:5556")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.timeout = float(rospy.get_param("~timeout", 0.5))
        self.rate_hz = float(rospy.get_param("~rate", 50.0))

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)            # keep only the latest command
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")   # all messages
        self.sock.connect(connect_address)

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.last_rx = None
        self.stopped = True
        rospy.loginfo("[cmd_vel_zmq_receiver] SUB connected %s -> publishing %s (timeout %.2fs)",
                      connect_address, self.output_topic, self.timeout)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        poller = zmq.Poller()
        poller.register(self.sock, zmq.POLLIN)
        while not rospy.is_shutdown():
            socks = dict(poller.poll(0))
            if self.sock in socks:
                try:
                    payload = self.sock.recv(zmq.NOBLOCK)
                    self._handle(payload)
                except zmq.Again:
                    pass
                except Exception as e:
                    rospy.logwarn_throttle(5.0, "[cmd_vel_zmq_receiver] recv error: %s", e)
            self._watchdog()
            rate.sleep()
        self._publish_zero()  # stop on shutdown

    def _handle(self, payload):
        if len(payload) != MSG_SIZE:
            rospy.logwarn_throttle(5.0, "[cmd_vel_zmq_receiver] bad size %d", len(payload))
            return
        magic, _sec, _nsec, lx, ly, lz, ax, ay, az = struct.unpack(MSG_FMT, payload)
        if magic != MAGIC:
            rospy.logwarn_throttle(5.0, "[cmd_vel_zmq_receiver] bad magic (wrong port?)")
            return
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = lx, ly, lz
        t.angular.x, t.angular.y, t.angular.z = ax, ay, az
        self.pub.publish(t)
        self.last_rx = rospy.Time.now()
        self.stopped = False

    def _watchdog(self):
        if self.last_rx is None:
            return
        if (rospy.Time.now() - self.last_rx).to_sec() > self.timeout and not self.stopped:
            self._publish_zero()
            self.stopped = True
            rospy.logwarn_throttle(2.0, "[cmd_vel_zmq_receiver] command stale -> SAFETY STOP")

    def _publish_zero(self):
        self.pub.publish(Twist())


if __name__ == "__main__":
    try:
        CmdVelZmqReceiver().spin()
    except rospy.ROSInterruptException:
        pass
