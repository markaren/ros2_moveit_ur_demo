import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # 1. Path Setup
    ur_description_share = FindPackageShare("ur_description")

    # Manually define the paths to fix the "Nested Xacro" issue on Windows
    ur_config_path = PathJoinSubstitution([ur_description_share, "config", "ur5e"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", PathJoinSubstitution([ur_description_share, "urdf", "ur.urdf.xacro"]),
            " ", "name:=ur",
            " ", "ur_type:=ur5e",
            " ", "joint_limit_params:=", ur_config_path, "/joint_limits.yaml",
            " ", "kinematics_params:=", ur_config_path, "/default_kinematics.yaml",
            " ", "physical_params:=", ur_config_path, "/physical_parameters.yaml",
            " ", "visual_params:=", ur_config_path, "/visual_parameters.yaml",
            " ", "safety_limits:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 1. Process the SRDF via Xacro
    # This resolves the ur.srdf.xacro into valid XML
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "srdf",
                "ur.srdf.xacro"
            ]),
            " ", "name:=ur",
            " ", "ur_type:=ur5e",
        ]
    )

    # 2. Combine into MoveGroup parameters
    # Note: We pass the Command objects directly; MoveIt will resolve them
    move_group_params = [
        {
            "robot_description": robot_description_content,
            "robot_description_semantic": robot_description_semantic_content,
            "planning_pipelines": {"pipelines": ["ompl"]},
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "use_sim_time": False,
        },
        # You also need to find and load the kinematics.yaml
        PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"
        ]),
    ]

    # 3. Launch MoveGroup
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # 4. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5. Static Virtual Joint (Fixes 'world' to 'base_link' transform)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 6. RViz with MoveIt Plugin
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
        ]
    )

    return LaunchDescription([
        static_tf,
        rsp_node,
        run_move_group_node,
        rviz_node,
    ])