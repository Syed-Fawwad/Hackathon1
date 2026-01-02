from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('basic_communication'),
            'params',
            'robot_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all nodes'
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_params_file_cmd,

        Node(
            package='basic_communication',
            executable='publisher_subscriber_example',
            name='minimal_publisher',
            output='screen',
            parameters=[params_file]
        ),
    ])