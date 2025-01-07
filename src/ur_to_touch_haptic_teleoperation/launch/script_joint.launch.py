import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_robot_driver',
            executable='ur_ros2_control_node',
            name='ur_driver',
            output='screen',
            parameters=[{'robot_ip': '192.168.1.10'}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=['controller.yaml']
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=['controller.yaml']
        ),
    ])
