"""
Starts the action_commander node against either the fake_controller or a real robot / URSim.

  fake_controller:=true  (default) — launches fake_controller stack locally
  fake_controller:=false           — expects ur_robot_driver already running,
                                     uses scaled_joint_trajectory_controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CONTROLLER_NAMES = {
    True:  'fake_ur_manipulator_controller',
    False: 'scaled_joint_trajectory_controller',
}


def launch_setup(context, *args, **kwargs):
    fake_controller = LaunchConfiguration('fake_controller').perform(context).lower() == 'true'
    controller_name = CONTROLLER_NAMES[fake_controller]

    return [
        Node(
            package='joint_commander',
            executable='action_commander',
            name='action_commander',
            output='screen',
            parameters=[{'controller_name': controller_name}],
        ),
    ]


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='UR robot model (ur3, ur5, ur5e, ur10, ur10e)'),
        DeclareLaunchArgument('fake_controller', default_value='true',
                              description='Launch fake_controller stack (false = real robot / URSim)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ur_bringup'), 'launch', 'fake_controller_stack.launch.py'])
            ]),
            launch_arguments={'ur_type': ur_type}.items(),
            condition=IfCondition(LaunchConfiguration('fake_controller')),
        ),

        OpaqueFunction(function=launch_setup),
    ])
