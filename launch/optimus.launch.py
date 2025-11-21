from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='optimus',
            executable='hardware_node',
            name='hardware_node',
            output='screen'
        ),
        Node(
            package='optimus',
            executable='klipper_bridge',
            name='klipper_bridge',
            output='screen'
        ),
        # Node(
        #     package='optimus',
        #     executable='voice_node',
        #     name='voice_node',
        #     output='screen'
        # )
    ])
