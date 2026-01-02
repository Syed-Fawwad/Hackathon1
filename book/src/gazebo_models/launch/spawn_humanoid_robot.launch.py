from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    world_name = LaunchConfiguration('world_name', default='simple_room.world')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot to spawn'
    )

    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='simple_room.world',
        description='Name of the Gazebo world to use'
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('gazebo_models'),
                'worlds',
                world_name
            ])
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(PathJoinSubstitution([
                FindPackageShare('gazebo_models'),
                'models',
                'humanoid_robot',
                'humanoid_robot.urdf'
            ]).perform(None)).read()
        }]
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0', '-y', '0', '-z', '0.5'  # Spawn the robot 0.5m above ground
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,
        declare_world_name_arg,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])