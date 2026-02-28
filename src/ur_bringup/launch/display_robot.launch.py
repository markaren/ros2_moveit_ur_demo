from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5e",
        description="Which UR robot to display (e.g., ur3, ur5, ur5e, ur10, ur10e)",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz2",
    )

    ur_type = LaunchConfiguration("ur_type")

    ur_config_path = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ", "name:=ur",
            " ", f"ur_type:=", ur_type,
            " ", "joint_limit_params:=", ur_config_path, "/joint_limits.yaml",
            " ", "kinematics_params:=", ur_config_path, "/default_kinematics.yaml",
            " ", "physical_params:=", ur_config_path, "/physical_parameters.yaml",
            " ", "visual_params:=", ur_config_path, "/visual_parameters.yaml",
            " ", "safety_limits:=true",  # Recommended for visualization
        ]
    )

    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_content}]
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_description"),
        "rviz",
        "view_robot.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        ur_type_arg, launch_rviz_arg,
        rsp_node, jsp_node, rviz_node, kine_env_node
    ])
