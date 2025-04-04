import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Node 1 : haply_inverse3_quaternion
    haply_inverse3_quaternion_node = Node(
        package='ros2_haply_inverse3_python',
        executable='haply_inverse3_quaternion',
        name='haply_inverse3_quaternion',
        output='screen'
    )

    # Node 2 : haply_inverse3_pos (lancé après 3 secondes)
    haply_inverse3_pos_node = Node(
        package='ros2_haply_inverse3_python',
        executable='haply_inverse3_pos',
        name='haply_inverse3_pos',
        output='screen'
    )

    # Node 3 : phantom_udp (lancé en parallèle)
    phantom_udp_node = Node(
        package='ur_udp',
        executable='phantom_udp',
        name='phantom_udp',
        output='screen'
    )

    # Attendre 3 secondes avant de lancer haply_inverse3_pos
    wait_3_seconds = TimerAction(
        period=3.0,  # Attente de 3 secondes
        actions=[haply_inverse3_pos_node]
    )

    return LaunchDescription([
        haply_inverse3_quaternion_node,
        wait_3_seconds,  # Ajouter le délai avant haply_inverse3_pos
        phantom_udp_node
    ])
