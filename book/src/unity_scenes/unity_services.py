#!/usr/bin/env python3

"""
Unity Services for Simulation Control
This node implements services for controlling Unity simulation: reset and view control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.srv import Service
import json
import socket
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String


class UnityServices(Node):
    """
    Unity Services Node
    Implements services: /unity_reset_simulation, /unity_set_view
    """

    def __init__(self):
        super().__init__('unity_services')

        # Initialize connection to Unity (if needed for direct control)
        self.unity_tcp_host = 'localhost'
        self.unity_tcp_port = 9090
        self.unity_connection = None

        # Initialize services
        self.reset_simulation_srv = self.create_service(
            Trigger,
            '/unity_reset_simulation',
            self.reset_simulation_callback
        )

        self.set_view_srv = self.create_service(
            SetBool,  # Using SetBool as a simple service that takes a boolean parameter
            '/unity_set_view',
            self.set_view_callback
        )

        # Additional publisher for Unity commands
        qos_profile = QoSProfile(depth=10)
        self.unity_command_pub = self.create_publisher(
            String, '/unity_commands', qos_profile
        )

        self.get_logger().info('Unity Services initialized')

    def reset_simulation_callback(self, request, response):
        """Callback for reset simulation service"""
        try:
            self.get_logger().info('Reset simulation service called')

            # Send reset command to Unity if connected
            reset_command = {
                'type': 'reset_simulation',
                'timestamp': self.get_clock().now().nanoseconds
            }
            self.send_command_to_unity(reset_command)

            # Publish reset command to Unity via ROS topic as well
            cmd_msg = String()
            cmd_msg.data = 'reset_simulation'
            self.unity_command_pub.publish(cmd_msg)

            response.success = True
            response.message = 'Simulation reset command sent to Unity'

            self.get_logger().info('Simulation reset command sent successfully')

        except Exception as e:
            self.get_logger().error(f'Error in reset simulation service: {e}')
            response.success = False
            response.message = f'Error resetting simulation: {str(e)}'

        return response

    def set_view_callback(self, request, response):
        """Callback for set view service"""
        try:
            view_mode = 'third_person' if request.data else 'first_person'
            self.get_logger().info(f'Set view service called with mode: {view_mode}')

            # Send view change command to Unity
            view_command = {
                'type': 'set_view',
                'view_mode': view_mode,
                'timestamp': self.get_clock().now().nanoseconds
            }
            self.send_command_to_unity(view_command)

            # Publish view command to Unity via ROS topic as well
            cmd_msg = String()
            cmd_msg.data = f'set_view:{view_mode}'
            self.unity_command_pub.publish(cmd_msg)

            response.success = True
            response.message = f'View mode set to {view_mode}'

            self.get_logger().info(f'View mode set to {view_mode} successfully')

        except Exception as e:
            self.get_logger().error(f'Error in set view service: {e}')
            response.success = False
            response.message = f'Error setting view: {str(e)}'

        return response

    def send_command_to_unity(self, command):
        """Send command directly to Unity via TCP (if connection exists)"""
        try:
            # In a real implementation, this would send directly to Unity
            # For now, we'll just log the command
            self.get_logger().debug(f'Sending command to Unity: {command}')
        except Exception as e:
            self.get_logger().warn(f'Could not send command to Unity: {e}')

    def connect_to_unity(self):
        """Establish connection to Unity for direct command sending"""
        try:
            self.unity_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_connection.connect((self.unity_tcp_host, self.unity_tcp_port))
            self.get_logger().info(f'Connected to Unity at {self.unity_tcp_host}:{self.unity_tcp_port}')
        except Exception as e:
            self.get_logger().warn(f'Could not connect to Unity: {e}')
            self.unity_connection = None

    def disconnect_from_unity(self):
        """Close connection to Unity"""
        if self.unity_connection:
            self.unity_connection.close()
            self.unity_connection = None


def main(args=None):
    rclpy.init(args=args)
    services = UnityServices()

    try:
        rclpy.spin(services)
    except KeyboardInterrupt:
        pass
    finally:
        services.disconnect_from_unity()
        services.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()