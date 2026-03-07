import os, yaml
from pathlib import Path

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def launch_setup(context, *args, **kwargs):
    fake_controller = LaunchConfiguration("fake_controller").perform(context).lower() == "true"
    ur_type = LaunchConfiguration("ur_type").perform(context)

    ur_config_path = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ", "name:=ur",
            " ", "ur_type:=", ur_type,
            " ", "joint_limit_params:=", ur_config_path, "/joint_limits.yaml",
            " ", "kinematics_params:=", ur_config_path, "/default_kinematics.yaml",
            " ", "physical_params:=", ur_config_path, "/physical_parameters.yaml",
            " ", "visual_params:=", ur_config_path, "/visual_parameters.yaml",
            " ", "safety_limits:=true",
        ]
    )

    fake_controller_node = Node(
        package="fake_controller",
        executable="fake_controller_node",
        name="fake_controller_node",
        output='screen',
        condition=IfCondition(LaunchConfiguration("fake_controller")),
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

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration("fake_controller")),
        parameters=[{'robot_description': robot_description_content, "use_sim_time": False}]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{"use_sim_time": False, "goal_planning": True}],
    )

    planner_node = Node(
        package="target_planner",
        executable="target_planner",
        name="target_planner",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    move_group_params = [moveit_config.to_dict()]

    if fake_controller:
        move_group_params.append(PathJoinSubstitution(
            [FindPackageShare("ur_bringup"), "config", "moveit_controllers.yaml"]
        ))

    move_group_params.append({"use_sim_time": False, "publish_robot_description_semantic": True})

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
        )],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return [planner_node, rsp_node, fake_controller_node, move_group, rviz_node, kine_env_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="Which UR robot to load (e.g., ur3, ur5, ur5e, ur10, ur10e)"),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Whether to launch RViz2"),
        DeclareLaunchArgument("fake_controller", default_value="true",
                              description="Whether to launch a fake controller"),
        OpaqueFunction(function=launch_setup),
    ])