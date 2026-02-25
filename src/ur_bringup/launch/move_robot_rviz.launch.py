import os, yaml
from pathlib import Path

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    # 1. Builder Setup - Ensure name matches the URDF 'name:=ur'
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur", "ur_type": "ur5e"})
        .to_moveit_configs()
    )

    # 2. Controller Params - Standard 'Simple' manager using 'Fake' handlers
    fake_controller_params = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["ur_manipulator_controller"],
            "ur_manipulator_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
        },
    }

    # 3. Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            fake_controller_params,
            {
                "use_sim_time": False,
                "publish_robot_description_semantic": True,
                "moveit_manage_controllers": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
        ],
    )


    spawner = TimerAction(
        period=2.0,  # wait 2 seconds
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["ur_manipulator_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        )]
    )

    # 4. Joint State Loopback
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/move_group/fake_controller_joint_states'],
        }]
    )

    # 5. Static TF (Prevents "No transform from [world] to [base_link]" errors)
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
            " ", "safety_limits:=true", # Recommended for visualization
        ]
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,  "use_sim_time": False}]
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

    return LaunchDescription([static_tf, rsp_node, move_group, spawner, rviz_node, joint_state_publisher])
