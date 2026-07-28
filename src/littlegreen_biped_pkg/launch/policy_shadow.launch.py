from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('littlegreen_biped_pkg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_config',
            default_value=[share, '/configs/policy_latest.yaml'],
            description='Paired deployment policy YAML. The ONNX file should be beside it.',
        ),
        DeclareLaunchArgument(
            'policy_runtime_config',
            default_value=[share, '/configs/policy_runtime.yaml'],
            description='Policy freshness, safety, and IMU extrinsic parameters.',
        ),
        DeclareLaunchArgument(
            'joint_map',
            default_value=[share, '/configs/joint_map.yaml'],
            description='Canonical LittleGreen joint order, defaults, and physical bounds.',
        ),
        DeclareLaunchArgument(
            'onnx_model_path',
            default_value='',
            description='Optional explicit ONNX override. Empty uses the model paired with policy_config.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation QoS/data behavior when true.',
        ),
        DeclareLaunchArgument(
            'override_imu',
            default_value='false',
            description='Use nominal IMU values instead of /imu/data.',
        ),
        DeclareLaunchArgument(
            'shadow_desired_position_topic',
            default_value='/policy_shadow/desired_position',
            description='Shadow-only policy target topic.',
        ),
        DeclareLaunchArgument(
            'enable_phase_test_override',
            default_value='false',
            description='Test/replay only. Enable deterministic Stand phase injection in shadow mode.',
        ),
        DeclareLaunchArgument(
            'phase_test_fixed_value',
            default_value='-1.0',
            description='Fixed phase in [0,1) when the test override is enabled; negative disables it.',
        ),
        DeclareLaunchArgument(
            'phase_test_seed',
            default_value='-1',
            description='Deterministic phase seed when the test override is enabled; negative disables it.',
        ),
        Node(
            package='littlegreen_biped_pkg',
            executable='littlegreen_biped_node',
            name='littlegreen_biped_node',
            output='screen',
            parameters=[
                LaunchConfiguration('policy_runtime_config'),
                {
                    'use_sim': ParameterValue(LaunchConfiguration('use_sim'), value_type=bool),
                    'policy_config_path': LaunchConfiguration('policy_config'),
                    'joint_map_path': LaunchConfiguration('joint_map'),
                    'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                    'override_imu': ParameterValue(
                        LaunchConfiguration('override_imu'), value_type=bool
                    ),
                    'policy_output_mode': 'shadow',
                    'shadow_desired_position_topic': LaunchConfiguration(
                        'shadow_desired_position_topic'
                    ),
                    'enable_phase_test_override': ParameterValue(
                        LaunchConfiguration('enable_phase_test_override'), value_type=bool
                    ),
                    'phase_test_fixed_value': ParameterValue(
                        LaunchConfiguration('phase_test_fixed_value'), value_type=float
                    ),
                    'phase_test_seed': ParameterValue(
                        LaunchConfiguration('phase_test_seed'), value_type=int
                    ),
                },
            ],
        ),
    ])
