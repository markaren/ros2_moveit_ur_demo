"""
topic commander.

Publishes sensor_msgs/JointState to the 'joint_commands' topic.
The fake_controller snaps immediately to the commanded positions and
re-publishes them on 'joint_states' at its configured rate (default 50 Hz).

Usage: tweak WAYPOINTS below and run via topic_commander.launch.py.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Each entry is a list of 6 joint angles in radians.
WAYPOINTS = [
    [0.0,         -math.pi/2,  0.0,  -math.pi/2,  0.0,  0.0],   # home
    [math.pi/4,   -math.pi/2,  0.5,  -math.pi/2,  0.0,  0.0],
    [math.pi/2,   -math.pi/3,  0.8,  -math.pi/2,  0.0,  0.0],
    [0.0,         -math.pi/2,  0.0,  -math.pi/2,  0.0,  0.0],   # back to home
]

DWELL_SEC = 2.0  # seconds to hold each waypoint


class TopicCommander(Node):
    def __init__(self):
        super().__init__('topic_commander')
        self._pub = self.create_publisher(JointState, 'joint_commands', 10)
        self._index = 0
        self._timer = self.create_timer(DWELL_SEC, self._send_next)
        # Send the first waypoint immediately without waiting for the timer.
        self._send_next()

    def _send_next(self):
        positions = WAYPOINTS[self._index % len(WAYPOINTS)]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self._pub.publish(msg)
        self.get_logger().info(
            f'Sent waypoint {self._index % len(WAYPOINTS)}: '
            + ', '.join(f'{math.degrees(p):.1f}°' for p in positions)
        )
        self._index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TopicCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
