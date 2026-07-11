from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pd_pkg_share = FindPackageShare('pd_controller_pkg')
    teleop_pkg_share = FindPackageShare('teleop_twist_joy')
    biped_pkg_share = FindPackageShare('littlegreen_biped_pkg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_config',
            default_value=[biped_pkg_share, '/configs/policy_latest.yaml'],
            description='Paired deployment policy YAML. The ONNX file should be beside it.'
        ),
        DeclareLaunchArgument(
            'policy_runtime_config',
            default_value=[biped_pkg_share, '/configs/policy_runtime.yaml'],
            description='Policy-node runtime safety, freshness, and IMU extrinsic parameters.'
        ),
        DeclareLaunchArgument(
            'onnx_model_path',
            default_value='',
            description='Optional explicit ONNX override. Empty uses the model paired with policy_config.'
        ),
        DeclareLaunchArgument(
            'joint_map',
            default_value=[biped_pkg_share, '/configs/joint_map.yaml'],
            description='Path to canonical sim-to-real joint map YAML.'
        ),
        DeclareLaunchArgument(
            'pd_config',
            default_value=[pd_pkg_share, '/config/pd_config.yaml'],
            description='Path to the current downstream command-shaping controller config.'
        ),
        DeclareLaunchArgument(
            'controller_mode',
            default_value='safety_only',
            description='pd_controller_pkg mode: safety_only, outer_pd, or outer_pid.'
        ),
        DeclareLaunchArgument(
            'teleop_config',
            default_value=[teleop_pkg_share, '/config/shanwan.config.yaml'],
            description='Path to teleop_twist_joy YAML config.'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation QoS/data layout behavior when true.'
        ),
        DeclareLaunchArgument(
            'override_imu',
            default_value='false',
            description='Use zero angular velocity and nominal gravity instead of /imu/data.'
        ),
        DeclareLaunchArgument(
            'policy_output_mode',
            default_value='live',
            description='Policy output mode: live, shadow, or disabled.'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[LaunchConfiguration('teleop_config')],
            remappings=[('/cmd_vel', '/command_velocity')],
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
                    'override_imu': ParameterValue(LaunchConfiguration('override_imu'), value_type=bool),
                    'policy_config_path': LaunchConfiguration('policy_config'),
                    'joint_map_path': LaunchConfiguration('joint_map'),
                    'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                    'policy_output_mode': LaunchConfiguration('policy_output_mode'),
                },
            ],
        ),

        Node(
            package='joystick_bridge',
            executable='cmd_vel_to_file',
            name='cmd_vel_to_file_node',
            output='screen'
        ),

        # Safety envelope + reference shaper + configurable velocity-form outer-loop
        # position controller. ROS command topics remain unchanged.
        Node(
            package='pd_controller_pkg',
            executable='pd_controller_node',
            name='pd_controller_node',
            output='screen',
            parameters=[
                LaunchConfiguration('pd_config'),
                {
                    'joint_map_path': LaunchConfiguration('joint_map'),
                    'controller_mode': LaunchConfiguration('controller_mode'),
                },
            ],
        ),
    ])
