from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleSetState
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Create the lifecycle publisher node
    lifecycle_publisher = Node(
        package='basic_communication',
        executable='lifecycle_node_example',
        name='lifecycle_publisher',
        output='screen',
    )

    # Configure the node after it starts
    configure_node = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_publisher,
            on_start=[
                LifecycleSetState(
                    lifecycle_node_matcher=lifecycle_publisher,
                    lifecycle_state='configuring',
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            ]
        )
    )

    # Activate the node after configuration
    activate_node = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_publisher,
            on_start=[
                LifecycleSetState(
                    lifecycle_node_matcher=lifecycle_publisher,
                    lifecycle_state='activating',
                    transition_id=Transition.TRANSITION_ACTIVATE
                )
            ]
        )
    )

    return LaunchDescription([
        lifecycle_publisher,
        configure_node,
        activate_node,
    ])