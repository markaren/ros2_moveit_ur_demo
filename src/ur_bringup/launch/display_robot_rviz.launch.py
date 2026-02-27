
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():

    ur_config_path = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ", "name:=ur",
            " ", "ur_type:=ur5e",
            " ", "joint_limit_params:=", ur_config_path, "/joint_limits.yaml",
            " ", "kinematics_params:=", ur_config_path, "/default_kinematics.yaml",
            " ", "physical_params:=", ur_config_path, "/physical_parameters.yaml",
            " ", "visual_params:=", ur_config_path, "/visual_parameters.yaml",
            " ", "safety_limits:=true", # Recommended for visualization
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
        FindPackageShare("ur_description"), # ur_description often has a basic view config
        "rviz",
        "view_robot.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rsp_node, jsp_node, rviz_node, kine_env_node
    ])
