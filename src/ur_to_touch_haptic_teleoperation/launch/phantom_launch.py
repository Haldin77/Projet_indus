import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FindExecutable

def generate_launch_description():
    omni_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('omni_common'),
                'launch',
                'omni_state.launch.py'
            ])
        ]),
        launch_arguments={}.items()
    )

    phantom_udp_node = Node(
        package='ur_udp',
        executable='phantom_udp',
        name='phantom_udp',
        output='screen'
    )
    
    ld =  LaunchDescription([
        omni_common_launch,
        phantom_udp_node
    ])
        
    return ld