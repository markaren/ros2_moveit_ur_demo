"""
Starts the fake_controller stack + topic_commander node.

The topic_commander snaps the robot through a sequence of joint positions
by publishing directly to the 'joint_commands' topic (no interpolation).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='UR robot model (ur3, ur5, ur5e, ur10, ur10e)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ur_bringup'), 'launch', 'fake_controller_stack.launch.py'])
            ]),
            launch_arguments={'ur_type': ur_type}.items(),
        ),

        Node(
            package='kine',
            executable='kine_environment',
            name='kine_environment',
            output='screen',
            parameters=[{'use_sim_time': False, 'goal_planning': False}],
        ),

        Node(
            package='joint_commander',
            executable='topic_commander',
            name='topic_commander',
            output='screen',
        ),
    ])
