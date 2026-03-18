"""
action commander.

Sends a FollowJointTrajectory action goal to a joint trajectory controller.
Works with fake_controller, URSim, or a real UR robot — set the
'controller_name' parameter to match the active controller:

  simulated_controller:               simulated_joint_controller  (default)
  real robot / URSim (ur_robot_driver): scaled_joint_trajectory_controller

Usage: tweak WAYPOINTS below and run via action_commander.launch.py.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# (positions [rad], time_from_start [sec])
WAYPOINTS = [
    ([0.0,        -math.pi/2,  0.0,  -math.pi/2,  0.0,  0.0], 0.0),
    ([math.pi/4,  -math.pi/2,  0.5,  -math.pi/2,  0.0,  0.0], 3.0),
    ([math.pi/2,  -math.pi/3,  0.8,  -math.pi/2,  0.0,  0.0], 6.0),
    ([0.0,        -math.pi/2,  0.0,  -math.pi/2,  0.0,  0.0], 9.0),
]


def _to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class ActionCommander(Node):
    def __init__(self):
        super().__init__('action_commander')
        controller = self.declare_parameter(
            'controller_name', 'simulated_joint_controller'
        ).get_parameter_value().string_value
        action_name = f'/{controller}/follow_joint_trajectory'
        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self.get_logger().info(f'Waiting for action server: {action_name}')
        self._client.wait_for_server()
        self.get_logger().info('Action server ready — sending trajectory')
        self._send_trajectory()

    def _send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        for positions, t in WAYPOINTS:
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = _to_duration(t)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted — executing trajectory')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback):
        pos = feedback.feedback.actual.positions
        self.get_logger().info(
            'Position: ' + ', '.join(f'{math.degrees(p):.1f}°' for p in pos),
            throttle_duration_sec=0.5,
        )

    def _on_result(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory complete')
        else:
            self.get_logger().error(f'Trajectory failed (error_code={result.error_code})')


def main(args=None):
    rclpy.init(args=args)
    node = ActionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
