import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindPackageShare
import os

def generate_launch_description():
    # Déclaration des arguments
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode')
    omni_rate_arg = DeclareLaunchArgument('omni_rate', default_value='1000', description='Omni rate')
    config_arg = DeclareLaunchArgument('config', default_value=os.path.join(
        FindPackageShare('ur_to_touch_haptic_teleoperation').find('ur_to_touch_haptic_teleoperation'), 'config', 'servo_config.yaml'),
        description='Path to the config file')

    # Inclusion du launch d'Omni
    omni_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('omni_common'), '/launch/omni_state.launch.py']),
        launch_arguments={'units': 'm', 'publish_rate': LaunchConfiguration('omni_rate')}.items()
    )

    # Lancer le node ur_to_touch_haptic_teleoperation_teleop_node
    ur_teleop_node = Node(
        package='ur_to_touch_haptic_teleoperation',
        executable='ur_to_touch_haptic_teleoperation_teleop_node',
        name='ur_to_touch_haptic_teleoperation_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        debug_arg,
        omni_rate_arg,
        config_arg,
        omni_launch,
        ur_teleop_node
    ])

