"""
Base launch file: robot_state_publisher + fake_controller.
Intended to be included by other launch files, but can also run standalone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur_bringup.launch_utils import robot_description_command

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type').perform(context)
    robot_description_content = robot_description_command(ur_type)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}],
    )

    fake_controller = Node(
        package='fake_controller',
        executable='fake_controller_node',
        name='fake_controller_node',
        output='screen',
        parameters=[{'joint_names': JOINT_NAMES, 'use_sim_time': False}],
    )

    return [rsp, fake_controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='UR robot model (ur3, ur5, ur5e, ur10, ur10e)'),
        OpaqueFunction(function=launch_setup),
    ])
