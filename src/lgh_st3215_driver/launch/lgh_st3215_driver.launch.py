from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

VALID_PROFILES = {'commissioning', 'runtime_safe'}

def _launch_setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in VALID_PROFILES:
        raise RuntimeError(f"Unsupported ST3215 driver profile '{profile}'. Valid: {sorted(VALID_PROFILES)}")
    package_share = Path(get_package_share_directory('lgh_st3215_driver'))
    profile_config = package_share / 'config' / 'profiles' / f'{profile}.yaml'
    return [Node(
        package='lgh_st3215_driver',
        executable='lgh_st3215_driver_node',
        name='lgh_st3215_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            str(profile_config),
            {
                'port': LaunchConfiguration('port'),
                'joint_map_path': LaunchConfiguration('servo_map'),
                'driver_profile': profile,
                'writes_enabled': ParameterValue(LaunchConfiguration('enable_writes'), value_type=bool),
                'default_pose_move_duration_sec': ParameterValue(
                    LaunchConfiguration('default_pose_move_duration_sec'), value_type=float),
            },
        ],
    )]

def generate_launch_description():
    package_share = Path(get_package_share_directory('lgh_st3215_driver'))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='commissioning', description='commissioning or runtime_safe'),
        DeclareLaunchArgument('config', default_value=str(package_share / 'config' / 'servo_driver.yaml')),
        DeclareLaunchArgument('servo_map', default_value=str(package_share / 'config' / 'servo_map.yaml')),
        DeclareLaunchArgument('port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('enable_writes', default_value='false', description='Never implied by profile selection.'),
        DeclareLaunchArgument('default_pose_move_duration_sec', default_value='4.0'),
        OpaqueFunction(function=_launch_setup),
    ])
