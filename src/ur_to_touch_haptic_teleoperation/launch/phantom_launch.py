import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FindExecutable

def generate_launch_description():
    phantom_udp_node = Node(
        package='ur_udp',
        executable='phantom_udp',
        name='phantom_udp',
        output='screen'
    )

    haply_inverse3_quaternion_node = Node(
        package='ros2_haply_inverse3_python',
        executable='haply_inverse3_quaternion',
        name='haply_inverse3_quaternion',
        output='screen'
    )

    haply_inverse3_pos_node = Node(
        package='ros2_haply_inverse3_python',
        executable='haply_inverse3_pos',
        name='haply_inverse3_pos',
        output='screen'
    )
    
    ld =  LaunchDescription([
        haply_inverse3_quaternion_node,
        haply_inverse3_pos_node,
        phantom_udp_node
    ])
        
    return ld