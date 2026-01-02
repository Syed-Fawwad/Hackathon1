#!/usr/bin/env python3

"""
Domain Randomization Node for Parameter Randomization
This node specifically handles parameter randomization for sim-to-real transfer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from rclpy.parameter import Parameter
import random
import numpy as np
from typing import Dict, List, Any, Optional
import time
from builtin_interfaces.msg import Time


class DomainRandomizationNode(Node):
    """
    Domain Randomization Node
    Handles parameter randomization for sim-to-real transfer.
    """

    def __init__(self):
        super().__init__('domain_randomization_node')

        # Initialize parameters
        self.declare_parameter('randomization_enabled', True)
        self.declare_parameter('randomization_rate', 1.0)  # Hz
        self.declare_parameter('randomization_seed', 12345)
        self.declare_parameter('domain_randomization_topic', '/domain_randomization/parameters')

        # Store randomization state
        self.randomization_enabled = self.get_parameter('randomization_enabled').value
        self.randomization_rate = self.get_parameter('randomization_rate').value
        self.domain_randomization_topic = self.get_parameter('domain_randomization_topic').value

        # Define parameter ranges for different domains
        self.parameter_ranges = {
            # Simulation physics
            'sim_gravity': {'min': -10.8, 'max': -8.8, 'current': -9.81},
            'sim_friction': {'min': 0.1, 'max': 1.0, 'current': 0.5},
            'sim_restitution': {'min': 0.0, 'max': 0.5, 'current': 0.2},
            'sim_damping': {'min': 0.0, 'max': 0.2, 'current': 0.1},

            # Robot dynamics
            'robot_mass_variance': {'min': 0.8, 'max': 1.2, 'current': 1.0},
            'robot_com_offset_x': {'min': -0.05, 'max': 0.05, 'current': 0.0},
            'robot_com_offset_y': {'min': -0.05, 'max': 0.05, 'current': 0.0},
            'robot_com_offset_z': {'min': -0.05, 'max': 0.05, 'current': 0.0},

            # Sensor characteristics
            'sensor_noise_std': {'min': 0.0, 'max': 0.1, 'current': 0.01},
            'sensor_bias': {'min': -0.05, 'max': 0.05, 'current': 0.0},
            'sensor_delay_ms': {'min': 0.0, 'max': 50.0, 'current': 10.0},

            # Visual properties
            'lighting_intensity': {'min': 0.5, 'max': 1.5, 'current': 1.0},
            'texture_roughness': {'min': 0.0, 'max': 1.0, 'current': 0.5},
            'background_color_r': {'min': 0.0, 'max': 1.0, 'current': 0.5},
            'background_color_g': {'min': 0.0, 'max': 1.0, 'current': 0.5},
            'background_color_b': {'min': 0.0, 'max': 1.0, 'current': 0.5},
        }

        # Publishers
        qos_profile = QoSProfile(depth=10)
        self.param_pub = self.create_publisher(
            Float32MultiArray, self.domain_randomization_topic, qos_profile
        )
        self.status_pub = self.create_publisher(
            String, '/domain_randomization/status', qos_profile
        )

        # Subscribers
        self.param_sub = self.create_subscription(
            String, '/domain_randomization/request', self.param_request_callback, qos_profile
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool, '/domain_randomization/enable', self.enable_callback
        )
        self.update_srv = self.create_service(
            Trigger, '/domain_randomization/update', self.update_callback
        )
        self.get_params_srv = self.create_service(
            Trigger, '/domain_randomization/get_params', self.get_params_callback
        )

        # Timer for periodic randomization
        self.randomization_timer = self.create_timer(
            1.0 / self.randomization_rate, self.randomize_parameters
        )

        # Set random seed
        seed = self.get_parameter('randomization_seed').value
        random.seed(seed)
        np.random.seed(seed)

        self.get_logger().info(f'Domain Randomization Node initialized with rate {self.randomization_rate}Hz')

    def randomize_parameters(self):
        """Randomize all parameters within their ranges"""
        if not self.randomization_enabled:
            return

        updated_params = {}

        for param_name, range_info in self.parameter_ranges.items():
            # Randomize the parameter
            new_value = random.uniform(range_info['min'], range_info['max'])
            range_info['current'] = new_value
            updated_params[param_name] = new_value

        # Publish the updated parameters
        self.publish_parameters(updated_params)

        # Log the update
        status_msg = String()
        status_msg.data = f'Domain parameters updated: {len(updated_params)} parameters randomized'
        self.status_pub.publish(status_msg)

        self.get_logger().debug(f'Randomized parameters: {list(updated_params.keys())}')

    def publish_parameters(self, params: Dict[str, float]):
        """Publish parameters to the domain randomization topic"""
        param_msg = Float32MultiArray()

        # Create header
        param_msg.layout.dim = []
        param_msg.layout.data_offset = 0

        # Add parameter values to the message
        param_values = []
        for name, value in params.items():
            param_values.append(value)

        param_msg.data = param_values

        self.param_pub.publish(param_msg)

    def param_request_callback(self, msg: String):
        """Handle parameter requests"""
        request = msg.data
        self.get_logger().info(f'Received parameter request: {request}')

        if request == 'update':
            self.randomize_parameters()
        elif request.startswith('set_range:'):
            # Parse set_range:parameter_name:min:max
            try:
                parts = request[10:].split(':')
                if len(parts) == 3:
                    param_name, min_str, max_str = parts
                    min_val = float(min_str)
                    max_val = float(max_str)
                    if param_name in self.parameter_ranges:
                        self.parameter_ranges[param_name]['min'] = min_val
                        self.parameter_ranges[param_name]['max'] = max_val
                        self.get_logger().info(f'Updated range for {param_name}: [{min_val}, {max_val}]')
            except ValueError:
                self.get_logger().error(f'Invalid set_range format: {request}')

    def enable_callback(self, request, response):
        """Service callback to enable/disable randomization"""
        self.randomization_enabled = request.data
        response.success = True
        response.message = f'Domain randomization {"enabled" if self.randomization_enabled else "disabled"}'

        self.get_logger().info(response.message)
        return response

    def update_callback(self, request, response):
        """Service callback to manually trigger parameter update"""
        if self.randomization_enabled:
            self.randomize_parameters()
            response.success = True
            response.message = 'Domain randomization parameters updated'
        else:
            response.success = False
            response.message = 'Domain randomization is disabled'

        return response

    def get_params_callback(self, request, response):
        """Service callback to get current parameters"""
        try:
            # In a real implementation, we would return a custom message
            # For now, we'll just return success
            response.success = True
            response.message = f'Current parameters: {len(self.parameter_ranges)} total'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting parameters: {str(e)}'
            return response

    def get_parameter(self, param_name: str) -> Optional[float]:
        """Get the current value of a specific parameter"""
        if param_name in self.parameter_ranges:
            return self.parameter_ranges[param_name]['current']
        return None

    def set_parameter_range(self, param_name: str, min_val: float, max_val: float):
        """Set the range for a specific parameter"""
        if param_name in self.parameter_ranges:
            self.parameter_ranges[param_name]['min'] = min_val
            self.parameter_ranges[param_name]['max'] = max_val
            self.get_logger().info(f'Set range for {param_name}: [{min_val}, {max_val}]')
        else:
            self.get_logger().warn(f'Parameter {param_name} not found in ranges')

    def get_all_parameters(self) -> Dict[str, float]:
        """Get all current parameter values"""
        params = {}
        for name, info in self.parameter_ranges.items():
            params[name] = info['current']
        return params

    def enable_domain_randomization(self):
        """Enable domain randomization"""
        self.randomization_enabled = True
        self.get_logger().info('Domain randomization enabled')

    def disable_domain_randomization(self):
        """Disable domain randomization"""
        self.randomization_enabled = False
        self.get_logger().info('Domain randomization disabled')

    def set_randomization_rate(self, rate: float):
        """Set the randomization update rate"""
        self.randomization_rate = rate
        # Update the timer period
        self.randomization_timer.timer_period_ns = int(1e9 / rate)
        self.get_logger().info(f'Domain randomization rate set to {rate}Hz')


def main(args=None):
    rclpy.init(args=args)
    node = DomainRandomizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()