from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    srv_node = Node(
        package='ros_acoustics',
        executable='srv_compute_waveforms.py',
    )

    return LaunchDescription([srv_node])

