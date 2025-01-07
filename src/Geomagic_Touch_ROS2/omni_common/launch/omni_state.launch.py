from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Environment variable setup
    set_gtdd_home = SetEnvironmentVariable(
        name="GTDD_HOME",
        value="/home/$user/.3dsystems"
    )

    # Node definition
    omni_node = Node(
        package="omni_common",
        executable="omni_state",
        output="screen",
        parameters=[
            {"omni_name": "phantom"},
            {"publish_rate": 1000},
            {"reference_frame": "/map"},
            {"units": "mm"}
        ]
    )

    return LaunchDescription([
        set_gtdd_home,
        omni_node
    ])

