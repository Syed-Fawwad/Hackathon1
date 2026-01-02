#!/usr/bin/env python3

"""
Domain Randomization Framework
This module provides a framework for domain randomization in robotics simulation.
Domain randomization helps bridge the sim-to-real gap by training models on
environments with randomized physical parameters.
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
import yaml


class DomainRandomizationFramework(Node):
    """
    Domain Randomization Framework
    Provides tools for randomizing simulation parameters to improve sim-to-real transfer.
    """

    def __init__(self):
        super().__init__('domain_randomization_framework')

        # Initialize parameters for randomization ranges
        self.declare_parameter('randomization_enabled', True)
        self.declare_parameter('randomization_update_rate', 1.0)  # Hz
        self.declare_parameter('randomization_seed', 42)

        # Define parameter ranges for randomization
        self.param_ranges = {
            # Physical properties
            'gravity': {'min': -9.81 * 1.1, 'max': -9.81 * 0.9},  # ±10%
            'friction_coefficient': {'min': 0.1, 'max': 1.0},
            'restitution_coefficient': {'min': 0.0, 'max': 0.5},
            'mass_variance': {'min': 0.8, 'max': 1.2},  # ±20%

            # Visual properties
            'lighting_intensity': {'min': 0.5, 'max': 1.5},
            'texture_randomization': {'min': 0.0, 'max': 1.0},

            # Sensor properties
            'sensor_noise': {'min': 0.0, 'max': 0.1},
            'sensor_bias': {'min': -0.05, 'max': 0.05},

            # Dynamics properties
            'damping_factor': {'min': 0.0, 'max': 0.1},
            'joint_friction': {'min': 0.0, 'max': 0.1},
        }

        # Current randomized parameters
        self.current_params = {}
        self.randomization_enabled = self.get_parameter('randomization_enabled').value

        # Initialize publishers for parameter updates
        qos_profile = QoSProfile(depth=10)
        self.param_update_pub = self.create_publisher(
            Float32MultiArray, '/domain_randomization/param_updates', qos_profile
        )
        self.status_pub = self.create_publisher(
            String, '/domain_randomization/status', qos_profile
        )

        # Initialize services
        self.enable_srv = self.create_service(
            SetBool, '/domain_randomization/enable', self.enable_callback
        )
        self.update_srv = self.create_service(
            Trigger, '/domain_randomization/update', self.update_callback
        )

        # Timer for periodic randomization updates
        update_rate = self.get_parameter('randomization_update_rate').value
        self.randomization_timer = self.create_timer(
            1.0 / update_rate, self.update_randomization
        )

        # Set random seed
        seed = self.get_parameter('randomization_seed').value
        random.seed(seed)
        np.random.seed(seed)

        # Initialize current parameters
        self.initialize_parameters()

        self.get_logger().info('Domain Randomization Framework initialized')

    def initialize_parameters(self):
        """Initialize current parameters with default values"""
        for param_name, range_info in self.param_ranges.items():
            # Set to midpoint initially
            mid_value = (range_info['min'] + range_info['max']) / 2.0
            self.current_params[param_name] = mid_value

    def update_randomization(self):
        """Update randomization parameters"""
        if not self.randomization_enabled:
            return

        # Randomize each parameter within its range
        for param_name, range_info in self.param_ranges.items():
            if param_name == 'texture_randomization':
                # Special handling for discrete parameters
                self.current_params[param_name] = random.uniform(
                    range_info['min'], range_info['max']
                )
            else:
                # Continuous parameters
                self.current_params[param_name] = random.uniform(
                    range_info['min'], range_info['max']
                )

        # Publish updated parameters
        self.publish_parameter_updates()

        # Log status
        status_msg = String()
        status_msg.data = f'Domain randomization updated: {len(self.current_params)} parameters'
        self.status_pub.publish(status_msg)

        self.get_logger().debug(f'Randomized parameters: {self.current_params}')

    def publish_parameter_updates(self):
        """Publish parameter updates to simulation"""
        # Create message with parameter values
        param_msg = Float32MultiArray()

        # Add all parameter values to the message
        param_values = []
        param_names = []

        for name, value in self.current_params.items():
            param_names.append(name)
            param_values.append(float(value))

        param_msg.data = param_values

        # For more complex parameter structures, we might need a custom message
        # This is a simplified approach
        self.param_update_pub.publish(param_msg)

    def enable_callback(self, request, response):
        """Service callback to enable/disable randomization"""
        self.randomization_enabled = request.data
        response.success = True
        response.message = f'Domain randomization {"enabled" if self.randomization_enabled else "disabled"}'

        self.get_logger().info(f'Domain randomization {response.message}')
        return response

    def update_callback(self, request, response):
        """Service callback to manually trigger parameter update"""
        if self.randomization_enabled:
            self.update_randomization()
            response.success = True
            response.message = 'Domain randomization parameters updated'
        else:
            response.success = False
            response.message = 'Domain randomization is disabled'

        return response

    def get_randomized_parameters(self) -> Dict[str, float]:
        """Get current randomized parameters"""
        return self.current_params.copy()

    def set_parameter_range(self, param_name: str, min_val: float, max_val: float):
        """Set the range for a specific parameter"""
        self.param_ranges[param_name] = {'min': min_val, 'max': max_val}
        self.get_logger().info(f'Set range for {param_name}: [{min_val}, {max_val}]')

    def get_parameter_range(self, param_name: str) -> Optional[Dict[str, float]]:
        """Get the range for a specific parameter"""
        return self.param_ranges.get(param_name)

    def get_current_parameter(self, param_name: str) -> Optional[float]:
        """Get the current value of a specific parameter"""
        return self.current_params.get(param_name)

    def randomize_texture(self) -> str:
        """Generate a randomized texture configuration"""
        # This is a placeholder for more complex texture randomization
        textures = ['metal', 'wood', 'plastic', 'concrete', 'grass', 'sand']
        return random.choice(textures)

    def randomize_lighting(self) -> Dict[str, float]:
        """Generate randomized lighting parameters"""
        return {
            'intensity': random.uniform(0.5, 1.5),
            'direction_x': random.uniform(-1.0, 1.0),
            'direction_y': random.uniform(-1.0, 1.0),
            'direction_z': random.uniform(-1.0, 1.0),
        }

    def randomize_physics(self) -> Dict[str, float]:
        """Generate randomized physics parameters"""
        return {
            'gravity': random.uniform(-10.8, -8.8),  # Around -9.81
            'friction': random.uniform(0.1, 1.0),
            'restitution': random.uniform(0.0, 0.5),
        }

    def save_configuration(self, filepath: str):
        """Save current randomization configuration to file"""
        config = {
            'enabled': self.randomization_enabled,
            'param_ranges': self.param_ranges,
            'current_params': self.current_params
        }

        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        self.get_logger().info(f'Domain randomization configuration saved to {filepath}')

    def load_configuration(self, filepath: str):
        """Load randomization configuration from file"""
        try:
            with open(filepath, 'r') as f:
                config = yaml.safe_load(f)

            if 'enabled' in config:
                self.randomization_enabled = config['enabled']
            if 'param_ranges' in config:
                self.param_ranges = config['param_ranges']
            if 'current_params' in config:
                self.current_params = config['current_params']

            self.get_logger().info(f'Domain randomization configuration loaded from {filepath}')
        except Exception as e:
            self.get_logger().error(f'Error loading configuration: {e}')


def main(args=None):
    rclpy.init(args=args)
    framework = DomainRandomizationFramework()

    try:
        rclpy.spin(framework)
    except KeyboardInterrupt:
        pass
    finally:
        framework.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()