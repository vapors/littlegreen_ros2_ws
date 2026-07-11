from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    share = FindPackageShare('littlegreen_biped_pkg')
    return LaunchDescription([
        DeclareLaunchArgument('policy_config', default_value=[share, '/configs/policy_latest.yaml']),
        DeclareLaunchArgument('policy_runtime_config', default_value=[share, '/configs/policy_runtime.yaml']),
        DeclareLaunchArgument('joint_map', default_value=[share, '/configs/joint_map.yaml']),
        DeclareLaunchArgument('onnx_model_path', default_value=''),
        DeclareLaunchArgument('override_imu', default_value='false'),
        Node(
            package='littlegreen_biped_pkg', executable='littlegreen_biped_node',
            name='littlegreen_biped_node', output='screen',
            parameters=[
                LaunchConfiguration('policy_runtime_config'),
                {
                    'policy_config_path': LaunchConfiguration('policy_config'),
                    'joint_map_path': LaunchConfiguration('joint_map'),
                    'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                    'override_imu': ParameterValue(LaunchConfiguration('override_imu'), value_type=bool),
                    'policy_output_mode': 'shadow',
                },
            ],
        ),
    ])
