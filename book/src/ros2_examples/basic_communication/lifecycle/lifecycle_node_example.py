#!/usr/bin/env python3

"""
Basic ROS 2 Lifecycle Node Example
This demonstrates the lifecycle management pattern in ROS 2.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Node as rclpy_Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


class LifecyclePublisherExample(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_publisher')
        self.get_logger().info('Lifecycle publisher is created in unconfigured state')
        self.pub = None
        self.timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for configuring the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is configuring itself.')

        # Create publisher but do not activate it yet
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)
        self.i = 0

        # Create timer but do not start it yet
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel()  # Cancel timer until activated

        self.get_logger().info(f'{self.get_name()} is configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for activating the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is activating itself.')

        # Activate the publisher and timer
        self.pub.activate()
        self.timer.reset()

        self.get_logger().info(f'{self.get_name()} is activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for deactivating the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is deactivating itself.')

        # Deactivate the publisher and timer
        self.pub.deactivate()
        self.timer.cancel()

        self.get_logger().info(f'{self.get_name()} is deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for cleaning up the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is cleaning up itself.')

        # Destroy publisher and timer
        self.destroy_publisher(self.pub)
        self.destroy_timer(self.timer)

        self.get_logger().info(f'{self.get_name()} is cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for shutting down the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is shutting down.')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Callback for handling errors in the lifecycle node."""
        self.get_logger().info(f'{self.get_name()} is in error state.')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback to publish messages."""
        if self.pub is not None:
            msg = String()
            msg.data = f'Lifecycle message {self.i}'
            self.pub.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1


class LifecycleManagerExample(rclpy_Node):

    def __init__(self):
        super().__init__('lifecycle_manager')
        self.lifecycle_node = None
        self.create_timer(5.0, self.manage_lifecycle)

    def set_lifecycle_node(self, node):
        """Set the lifecycle node to manage."""
        self.lifecycle_node = node

    def manage_lifecycle(self):
        """Manage the lifecycle of the node periodically."""
        if self.lifecycle_node is None:
            return

        current_state = self.lifecycle_node.get_current_state().label
        self.get_logger().info(f'Current state of {self.lifecycle_node.get_name()}: {current_state}')

        # Cycle through states
        if current_state == 'configuring':
            # Wait for configuration to complete
            pass
        elif current_state == 'inactive':
            # Activate the node
            self.lifecycle_node.activate()
        elif current_state == 'active':
            # Deactivate the node after some time
            self.lifecycle_node.deactivate()
        elif current_state == 'unconfigured':
            # Configure the node
            self.lifecycle_node.configure()


def main(args=None):
    rclpy.init(args=args)

    # Create the lifecycle node
    lifecycle_node = LifecyclePublisherExample()

    # Create the manager node
    manager_node = LifecycleManagerExample()
    manager_node.set_lifecycle_node(lifecycle_node)

    # Create an executor and add both nodes
    executor = SingleThreadedExecutor()
    executor.add_node(lifecycle_node)
    executor.add_node(manager_node)

    # Configure the lifecycle node initially
    lifecycle_node.configure()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Properly shut down
        lifecycle_node.deactivate()
        lifecycle_node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()