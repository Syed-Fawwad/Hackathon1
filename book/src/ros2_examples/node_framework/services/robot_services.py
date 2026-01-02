#!/usr/bin/env python3

"""
Robot Services for the ROS 2 Communication Framework
This module implements services for robot status and mode management.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from node_framework.communication_framework import CommunicationFrameworkNode


class RobotServicesNode(CommunicationFrameworkNode):
    """
    Node that provides robot services: /get_robot_status and /set_robot_mode
    """

    def __init__(self):
        super().__init__('robot_services_node')

        # Initialize robot state
        self.robot_status = {
            'operational': True,
            'mode': 'idle',  # idle, autonomous, manual, emergency
            'battery_level': 95.0,
            'error_code': 0,
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'active_nodes': ['perception', 'planning', 'control', 'sensor_fusion']
        }

        # Create services
        # For this example, we'll use a generic service interface since custom .srv files
        # would require a more complex build process
        self.get_robot_status_srv = self.create_service(
            GenericServiceInterface,
            '/get_robot_status',
            self.get_robot_status_callback
        )

        self.set_robot_mode_srv = self.create_service(
            GenericServiceInterface,
            '/set_robot_mode',
            self.set_robot_mode_callback
        )

        self.get_logger().info('Robot Services Node initialized with /get_robot_status and /set_robot_mode services')

    def get_robot_status_callback(self, request, response):
        """Callback for get_robot_status service"""
        self.get_logger().info('Received request for robot status')

        # Update timestamp
        self.robot_status['timestamp'] = self.get_clock().now().seconds_nanoseconds()

        # Create response data
        status_info = {
            'operational': self.robot_status['operational'],
            'mode': self.robot_status['mode'],
            'battery_level': self.robot_status['battery_level'],
            'error_code': self.robot_status['error_code'],
            'timestamp': str(self.robot_status['timestamp']),
            'active_nodes': self.robot_status['active_nodes']
        }

        response.success = True
        response.message = f"Robot Status: {status_info}"
        response.data = str(status_info)

        self.get_logger().info(f'Returning robot status: {response.message}')
        return response

    def set_robot_mode_callback(self, request, response):
        """Callback for set_robot_mode service"""
        # Parse the mode from the request (in a real service, this would be a specific field)
        requested_mode = getattr(request, 'mode', None) or getattr(request, 'data', 'idle')

        self.get_logger().info(f'Received request to set robot mode to: {requested_mode}')

        # Validate and set the mode
        valid_modes = ['idle', 'autonomous', 'manual', 'emergency', 'calibration', 'maintenance']

        if requested_mode in valid_modes:
            old_mode = self.robot_status['mode']
            self.robot_status['mode'] = requested_mode

            response.success = True
            response.message = f'Mode changed from {old_mode} to {requested_mode}'
            response.data = self.robot_status['mode']

            self.get_logger().info(f'Mode successfully changed from {old_mode} to {requested_mode}')
        else:
            response.success = False
            response.message = f'Invalid mode: {requested_mode}. Valid modes: {valid_modes}'
            response.data = ''

            self.get_logger().warn(f'Invalid mode requested: {requested_mode}')

        return response


# Define a generic service interface that can work with standard ROS 2 services
# In a real implementation, custom .srv files would be created in a separate package
from rcl_interfaces.srv import SetParameters, GetParameters

# For this example, we'll use a generic service interface that mimics what we need
class GenericServiceInterface:
    class Request:
        def __init__(self):
            self.data = ""
            self.mode = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""
            self.data = ""


def main(args=None):
    rclpy.init(args=args)
    robot_services_node = RobotServicesNode()

    try:
        rclpy.spin(robot_services_node)
    except KeyboardInterrupt:
        # Change to emergency mode on shutdown
        robot_services_node.robot_status['mode'] = 'emergency'
        robot_services_node.get_logger().info('Setting robot to emergency mode on shutdown')
    finally:
        robot_services_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()