import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FindExecutable

def generate_launch_description():
    ur3e_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur3e.launch.py'
            ])
        ]),
        launch_arguments={'robot_ip':'192.168.56.2'}.items()
    )
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={'ur_type':'ur3e','moveit_config_package':'ur3e_moveit_config'}.items()
    )
    
    ur3e_node = Node(
        package='ur_control',
        executable='ur3e',
        name='ur3e',
        output='screen'
    )
    
    ur_udp_node = Node(
        package='ur_udp',
        executable='ur_udp',
        name='ur_udp',
        output='screen'
    )
    
    ld =  LaunchDescription([
        ur3e_driver_launch,
        moveit_launch,
        ur3e_node,
        ur_udp_node
    ])
    
    ld.add_action(
        ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "service", "call",
                "/servo_node/start_servo",
                "std_srvs/srv/Trigger"
            ],
            output='screen'
        )
    )
        
    return ld
