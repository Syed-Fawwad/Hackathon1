from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_communication',
            executable='action_example',
            name='minimal_action_server',
            output='screen',
        ),
    ])