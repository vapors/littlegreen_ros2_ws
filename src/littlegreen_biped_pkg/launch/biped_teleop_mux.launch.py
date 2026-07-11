from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pd_pkg_share = FindPackageShare('pd_controller_pkg')
    teleop_pkg_share = FindPackageShare('teleop_twist_joy')
    biped_pkg_share = FindPackageShare('littlegreen_biped_pkg')

    return LaunchDescription([
        # 🧩 Launch arguments
        DeclareLaunchArgument(
            'pd_config',
            default_value=[pd_pkg_share, '/config/pd_config.yaml'],
            description='Path to the PD controller parameter file'
        ),
        DeclareLaunchArgument(
            'teleop_config',
            default_value=[teleop_pkg_share, '/config/shanwan.config.yaml'],
            description='Path to teleop_twist_joy YAML config'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation data or real hardware'
        ),
        DeclareLaunchArgument(
            'override_imu',
            default_value='false',
            description='Override IMU data with zeros'
        ),

        # 🕹️ Joystick input
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
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        ),

        # ⌨️ Keyboard input (open in new terminal)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # Opens terminal window
            remappings=[('/cmd_vel', '/cmd_vel_keyboard')]
        ),

        # 🔀 twist_mux node (merge keyboard + joystick)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[PathJoinSubstitution([biped_pkg_share, 'configs', 'twist_mux.yaml'])],
            remappings=[('/cmd_vel_out', '/command_velocity')]
        ),

        # 🤖 Biped node
        Node(
            package='littlegreen_biped_pkg',
            executable='littlegreen_biped_node',
            name='littlegreen_biped_node',
            output='screen',
            parameters=[{
                'use_sim': LaunchConfiguration('use_sim'),
                'override_imu': LaunchConfiguration('override_imu')
            }]
        ),

        # 📝 Save velocity
        Node(
            package='joystick_bridge',
            executable='cmd_vel_to_file',
            name='cmd_vel_to_file_node',
            output='screen'
        ),

        # ⚙️ PD controller
        Node(
            package='pd_controller_pkg',
            executable='pd_controller_node',
            name='pd_controller_node',
            output='screen',
            parameters=[LaunchConfiguration('pd_config')]
        )
    ])
