from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_communication',
            executable='service_example',
            name='minimal_service',
            output='screen',
        ),
    ])