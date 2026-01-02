from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ros_ip_address = LaunchConfiguration('ros_ip_address', default='127.0.0.1')
    ros_port = LaunchConfiguration('ros_port', default='9090')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_ros_ip_address_arg = DeclareLaunchArgument(
        'ros_ip_address',
        default_value='127.0.0.1',
        description='IP address for ROS TCP connection'
    )

    declare_ros_port_arg = DeclareLaunchArgument(
        'ros_port',
        default_value='9090',
        description='Port for ROS TCP connection'
    )

    # Unity bridge node
    unity_bridge_node = Node(
        package='unity_scenes',
        executable='unity_bridge_node',
        name='unity_bridge_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'ros_ip_address': ros_ip_address},
            {'ros_port': ros_port}
        ],
        output='screen'
    )

    # Visualization node
    visualization_node = Node(
        package='unity_scenes',
        executable='visualization_node',
        name='visualization_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Teleoperation interface
    teleop_interface = Node(
        package='unity_scenes',
        executable='teleop_interface',
        name='teleop_interface',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_ros_ip_address_arg,
        declare_ros_port_arg,
        unity_bridge_node,
        visualization_node,
        teleop_interface
    ])