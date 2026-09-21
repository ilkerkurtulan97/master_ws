#!/usr/bin/env python3
"""
Goal bridge: /goal_pose (PoseStamped) -> /mbf_mesh_nav/move_base (MoveBase action).

WHY: Move Base Flex is action-only. It has NO /move_base_simple/goal subscriber,
so the topic_tools relay in the launch file forwards goals into a dead topic and
the robot never moves. RViz "2D Nav Goal" and navmesh_gui both publish a
PoseStamped on /goal_pose; this node turns that into a MoveBase action goal
(plan + drive + replan/recover) on /mbf_mesh_nav/move_base.

RUN (in a terminal sourced with ROS2 + the workspace, conda deactivated):
  ros2 run mesh_nav_isaac_sim ...   # (not packaged yet)
  python3 /home/ilker/meshnavigation_ws/goal_bridge.py --ros-args -p use_sim_time:=true

Then set a goal with RViz "2D Nav Goal" (Topic must be /goal_pose) or the
navmesh_gui Drive button. Watch this terminal for accepted/feedback/result.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from mbf_msgs.action import MoveBase
import tf2_ros


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        # Names must match the YAML: planners: ["dijkstra"], controllers: ["mesh_controller"]
        self.planner = self.declare_parameter('planner', 'dijkstra').value
        self.controller = self.declare_parameter('controller', 'mesh_controller').value
        self.action_name = self.declare_parameter('action', '/mbf_mesh_nav/move_base').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        # TF used to snap the goal z to the floor (see _on_goal).
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._client = ActionClient(self, MoveBase, self.action_name)
        self._sub = self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.get_logger().info(
            f"goal_bridge ready: /goal_pose -> {self.action_name} "
            f"(planner='{self.planner}', controller='{self.controller}')")

    def _on_goal(self, msg: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f"Action server '{self.action_name}' not available")
            return
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        # RViz "2D Nav Goal" always publishes z=0, but the mesh floor is ~ -1.2 m.
        # The planner appends the raw goal as the path's last point and the
        # controller uses it as the arrival target, so a z=0 goal makes the path
        # curl 1.2 m up into the air and the rover can never reach it (it orbits
        # the point below). Snap the goal z to the robot's current floor height.
        # (Same-floor only; a 2D goal can't express which floor for multi-floor.)
        try:
            tf = self._tf_buf.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            robot_z = tf.transform.translation.z
            old_z = msg.pose.position.z
            msg.pose.position.z = robot_z
            self.get_logger().info(f"Snapped goal z {old_z:.2f} -> {robot_z:.2f} (robot floor)")
        except Exception as e:
            self.get_logger().warn(f"Could not look up robot z, sending goal z as-is: {e}")

        goal = MoveBase.Goal()
        goal.target_pose = msg
        goal.planner = self.planner
        goal.controller = self.controller
        self.get_logger().info(
            f"Sending goal: frame='{msg.header.frame_id}' "
            f"x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} z={msg.pose.position.z:.2f}")
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_accepted)

    def _on_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("Goal REJECTED by move_base")
            return
        self.get_logger().info("Goal accepted; executing...")
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb):
        f = fb.feedback
        self.get_logger().info(
            f"dist_to_goal={f.dist_to_goal:.2f} angle_to_goal={f.angle_to_goal:.2f} "
            f"outcome={f.outcome}", throttle_duration_sec=1.0)

    def _on_result(self, fut):
        r = fut.result().result
        self.get_logger().info(f"RESULT outcome={r.outcome} message=\"{r.message}\"")


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
