#!/usr/bin/env python3

"""
Basic ROS 2 Publisher/Subscriber Example
This demonstrates the fundamental communication pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare parameters with default values
        self.declare_parameter('publish_frequency', 2.0)
        self.declare_parameter('message_prefix', 'Hello')
        self.declare_parameter('counter_step', 1)

        # Get parameter values
        publish_frequency = self.get_parameter('publish_frequency').value
        self.message_prefix = self.get_parameter('message_prefix').value
        self.counter_step = self.get_parameter('counter_step').value

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0 / publish_frequency  # Convert frequency to period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_prefix} World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += self.counter_step


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Declare parameters with default values
        self.declare_parameter('subscription_queue_size', 10)
        self.declare_parameter('echo_enabled', True)

        # Get parameter values
        queue_size = self.get_parameter('subscription_queue_size').value
        self.echo_enabled = self.get_parameter('echo_enabled').value

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            queue_size)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if self.echo_enabled:
            self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    # Create an executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()