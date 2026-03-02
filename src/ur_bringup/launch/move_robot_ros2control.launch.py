import os, yaml
from pathlib import Path

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz2",
    )

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5e",
        description="Which UR robot to display (e.g., ur3, ur5, ur5e, ur10, ur10e)",
    )

    ur_type = LaunchConfiguration("ur_type")

    ur_config_path = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type])

    # Robot description with mock hardware for ros2_control
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur_mocked.urdf.xacro"]),
            " ", "name:=ur",
            " ", "ur_type:=", ur_type,
            " ", "joint_limit_params:=", ur_config_path, "/joint_limits.yaml",
            " ", "kinematics_params:=", ur_config_path, "/default_kinematics.yaml",
            " ", "physical_params:=", ur_config_path, "/physical_parameters.yaml",
            " ", "visual_params:=", ur_config_path, "/visual_parameters.yaml",
            " ", "safety_limits:=true",
            # Enable mock hardware for ros2_control
            " ", "use_mock_hardware:=true",
            " ", "mock_sensor_commands:=false",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur", "ur_type": ur_type})
        .to_moveit_configs()
    )

    # ros2_control controller manager
    ros2_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_bringup"), "config", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
            {"use_sim_time": False},
        ],
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn the joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # MoveIt controller config for ros2_control
    moveit_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_bringup"), "config", "moveit_controllers.yaml"]
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers_yaml,
            {
                "use_sim_time": False,
                "publish_robot_description_semantic": True,
                "moveit_manage_controllers": False,
            },
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    kine_env_node = Node(
        package="kine",
        executable="kine_environment",
        name="kine_environment",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "robot_description": robot_description_content,
            "goal_planning": True,
        }],
    )

    planner_node = Node(
        package="target_planner",
        executable="target_planner",
        name="target_planner",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        ur_type_arg, launch_rviz_arg,
        planner_node,
        static_tf, rsp_node,
        control_node,
        # Delay spawners to let controller_manager start
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[joint_trajectory_controller_spawner]),
        move_group, rviz_node, kine_env_node,
    ])
