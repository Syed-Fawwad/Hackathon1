from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Domain randomization framework
    domain_randomization_framework = Node(
        package='ros2_examples',
        executable='domain_randomization_framework',
        name='domain_randomization_framework',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'randomization_enabled': True},
            {'randomization_update_rate': 1.0}
        ],
        output='screen'
    )

    # Domain randomization node
    domain_randomization_node = Node(
        package='ros2_examples',
        executable='domain_randomization_node',
        name='domain_randomization_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'randomization_enabled': True},
            {'randomization_rate': 1.0}
        ],
        output='screen'
    )

    # System identification node
    system_identification_node = Node(
        package='ros2_examples',
        executable='system_identification_node',
        name='system_identification_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'identification_enabled': True},
            {'identification_update_rate': 0.1}
        ],
        output='screen'
    )

    # Parameter adaptation system
    parameter_adaptation_system = Node(
        package='ros2_examples',
        executable='parameter_adaptation_system',
        name='parameter_adaptation_system',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'adaptation_enabled': True},
            {'adaptation_rate': 0.1}
        ],
        output='screen'
    )

    # Transfer topics manager
    transfer_topics_manager = Node(
        package='ros2_examples',
        executable='transfer_topics_manager',
        name='transfer_topics_manager',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'transfer_enabled': True},
            {'sync_rate': 1.0}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        domain_randomization_framework,
        domain_randomization_node,
        system_identification_node,
        parameter_adaptation_system,
        transfer_topics_manager
    ])