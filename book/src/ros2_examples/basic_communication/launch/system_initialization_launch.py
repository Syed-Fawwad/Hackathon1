from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

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
        declare_use_sim_time_cmd,
        declare_params_file_cmd,

        # Basic communication nodes
        Node(
            package='basic_communication',
            executable='publisher_subscriber_example',
            name='minimal_publisher',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='basic_communication',
            executable='publisher_subscriber_example',
            name='minimal_subscriber',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='basic_communication',
            executable='service_example',
            name='minimal_service',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='basic_communication',
            executable='action_example',
            name='minimal_action_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])