#!/usr/bin/env python3

"""
Service Client Example for the ROS 2 Communication Framework
This module demonstrates how to use the robot services.
"""

import rclpy
from rclpy.node import Node
from node_framework.services.robot_services import GenericServiceInterface


class RobotServiceClient(Node):
    """
    Client node to demonstrate using the robot services
    """

    def __init__(self):
        super().__init__('robot_service_client')

        # Create clients for the services
        self.get_status_client = self.create_client(
            GenericServiceInterface,
            '/get_robot_status'
        )
        self.set_mode_client = self.create_client(
            GenericServiceInterface,
            '/set_robot_mode'
        )

        # Wait for services to be available
        while not self.get_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Status service not available, waiting again...')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mode service not available, waiting again...')

        # Timer to periodically call services
        self.timer = self.create_timer(5.0, self.call_services)
        self.call_count = 0

        self.get_logger().info('Robot Service Client initialized')

    def call_services(self):
        """Call the robot services periodically"""
        self.get_logger().info(f'Calling services (count: {self.call_count})')

        # Call get robot status
        self.call_get_robot_status()

        # Alternate between different modes
        if self.call_count % 3 == 0:
            self.call_set_robot_mode('idle')
        elif self.call_count % 3 == 1:
            self.call_set_robot_mode('autonomous')
        else:
            self.call_set_robot_mode('manual')

        self.call_count += 1

    def call_get_robot_status(self):
        """Call the get robot status service"""
        request = GenericServiceInterface.Request()
        future = self.get_status_client.call_async(request)

        # In a real implementation, you'd handle the response asynchronously
        # For this example, we'll just make a synchronous call
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f'Robot Status Response: {response.message}')
        else:
            self.get_logger().error('Failed to get robot status')

    def call_set_robot_mode(self, mode):
        """Call the set robot mode service"""
        request = GenericServiceInterface.Request()
        request.mode = mode
        future = self.set_mode_client.call_async(request)

        # In a real implementation, you'd handle the response asynchronously
        # For this example, we'll just make a synchronous call
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f'Set Robot Mode Response: {response.message}')
        else:
            self.get_logger().error(f'Failed to set robot mode to {mode}')


def main(args=None):
    rclpy.init(args=args)
    service_client = RobotServiceClient()

    try:
        rclpy.spin(service_client)
    except KeyboardInterrupt:
        pass
    finally:
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()