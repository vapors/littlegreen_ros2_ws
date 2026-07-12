from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    biped_share = FindPackageShare('littlegreen_biped_pkg')
    pd_share = FindPackageShare('pd_controller_pkg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_config',
            default_value=[biped_share, '/configs/policy_latest.yaml'],
            description='Paired deployment policy YAML. The ONNX file should be beside it.',
        ),
        DeclareLaunchArgument(
            'policy_runtime_config',
            default_value=[biped_share, '/configs/policy_runtime.yaml'],
            description='Policy freshness, safety, and IMU extrinsic parameters.',
        ),
        DeclareLaunchArgument(
            'joint_map',
            default_value=[biped_share, '/configs/joint_map.yaml'],
            description='Canonical LittleGreen joint order, defaults, and physical bounds.',
        ),
        DeclareLaunchArgument(
            'onnx_model_path',
            default_value='',
            description='Optional explicit ONNX override. Empty uses the model paired with policy_config.',
        ),
        DeclareLaunchArgument(
            'pd_config',
            default_value=[pd_share, '/config/pd_config.yaml'],
            description='Downstream safety and command-shaping configuration.',
        ),
        DeclareLaunchArgument(
            'controller_mode',
            default_value='safety_only',
            description='Initial live deployment must use safety_only. outer_pd/outer_pid are experimental.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation QoS/data behavior when true.',
        ),
        DeclareLaunchArgument(
            'override_imu',
            default_value='false',
            description='Use nominal IMU values instead of /imu/data. Not recommended for live hardware.',
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
                    'override_imu': ParameterValue(
                        LaunchConfiguration('override_imu'), value_type=bool
                    ),
                    'policy_config_path': LaunchConfiguration('policy_config'),
                    'joint_map_path': LaunchConfiguration('joint_map'),
                    'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                    'policy_output_mode': 'live',
                },
            ],
        ),
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
