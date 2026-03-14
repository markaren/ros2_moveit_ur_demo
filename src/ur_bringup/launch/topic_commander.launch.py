"""
Starts the fake_controller stack + topic_commander node.

The topic_commander snaps the robot through a sequence of joint positions
by publishing directly to the 'joint_commands' topic (no interpolation).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    ur_config_path = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', ur_type]
    )
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' name:=ur',
        ' ur_type:=', ur_type,
        ' joint_limit_params:=', ur_config_path, '/joint_limits.yaml',
        ' kinematics_params:=', ur_config_path, '/default_kinematics.yaml',
        ' physical_params:=', ur_config_path, '/physical_parameters.yaml',
        ' visual_params:=', ur_config_path, '/visual_parameters.yaml',
        ' safety_limits:=true',
    ])

    fake_controller = Node(
        package='fake_controller',
        executable='fake_controller_node',
        name='fake_controller_node',
        output='screen',
        parameters=[{'joint_names': JOINT_NAMES, 'use_sim_time': False}],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}],
    )

    kine = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'goal_planning': False
        }]
    )

    commander = Node(
        package='joint_commander',
        executable='topic_commander',
        name='topic_commander',
        output='screen',
    )

    return [fake_controller, rsp, commander, kine]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='UR robot model (ur3, ur5, ur5e, ur10, ur10e)'),
        OpaqueFunction(function=launch_setup),
    ])
