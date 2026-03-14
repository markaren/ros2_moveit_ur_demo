from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def robot_description_command(ur_type: str):
    """Build the xacro Command substitution for a UR robot description.

    Args:
        ur_type: Resolved robot type string, e.g. 'ur5e'.

    Returns:
        A Command substitution that produces the robot_description XML.
    """
    ur_config_path = PathJoinSubstitution([FindPackageShare('ur_description'), 'config', ur_type])
    return Command([
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
