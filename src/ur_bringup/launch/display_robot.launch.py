from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ur_bringup.launch_utils import robot_description_command


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type').perform(context)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rsp')),
        parameters=[{'robot_description': robot_description_command(ur_type)}],
    )

    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('launch_jsp')),
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{'use_sim_time': False, 'goal_planning': False}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return [rsp_node, jsp_node, rviz_node, kine_env_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='Which UR robot to display (ur3, ur5, ur5e, ur10, ur10e)'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Whether to launch RViz2'),
        DeclareLaunchArgument('launch_jsp', default_value='false',
                              description='Whether to launch the Joint State Publisher GUI'),
        DeclareLaunchArgument('launch_rsp', default_value='true',
                              description='Whether to launch the Robot State Publisher'),
        OpaqueFunction(function=launch_setup),
    ])
