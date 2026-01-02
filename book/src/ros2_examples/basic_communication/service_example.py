#!/usr/bin/env python3

"""
Basic ROS 2 Service Example
This demonstrates the request/response communication pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na={request.a}, b={request.b}')
        return response


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()
    minimal_client = MinimalClient()

    # Example client call
    response = minimal_client.send_request(1, 2)
    if response is not None:
        minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    else:
        minimal_client.get_logger().info('Service call failed')

    # Shutdown
    minimal_service.destroy_node()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()