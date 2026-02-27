import os, yaml
from pathlib import Path

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_name = "fake_ur_manipulator_controller"

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur", "ur_type": "ur5e"})
        .to_moveit_configs()
    )

    fake_controller_node = Node(
        package="fake_controller",
        executable="fake_controller_node",
        name="fake_controller_node",
        output='screen',
        parameters=[
            {
                "joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "use_sim_time": False
            }
        ]
    )

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
                "publish_robot_description_semantic": False,
                "moveit_manage_controllers": True
            },
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

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
            " ", "safety_limits:=true",  # Recommended for visualization
        ]
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, "use_sim_time": False}]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{
            "use_sim_time": False,
            "robot_description": robot_description_content,
        }],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition("True"),
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
            {
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription([static_tf, rsp_node, fake_controller_node, move_group, rviz_node, kine_env_node])
