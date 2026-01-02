#!/usr/bin/env python3

"""
Transfer Topics Manager
This node manages topics for sim-to-real transfer: /sim_params, /real_params, /domain_randomization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
from rclpy.parameter import Parameter
import numpy as np
from typing import Dict, List, Any, Optional
import json


class TransferTopicsManager(Node):
    """
    Transfer Topics Manager
    Manages topics for sim-to-real transfer: /sim_params, /real_params, /domain_randomization.
    """

    def __init__(self):
        super().__init__('transfer_topics_manager')

        # Initialize parameters
        self.declare_parameter('transfer_enabled', True)
        self.declare_parameter('sync_rate', 1.0)  # Hz

        self.transfer_enabled = self.get_parameter('transfer_enabled').value
        self.sync_rate = self.get_parameter('sync_rate').value

        # Initialize parameter storage
        self.simulation_params = {}
        self.real_world_params = {}
        self.domain_randomization_params = {}

        # Publishers for transfer topics
        qos_profile = QoSProfile(depth=10)
        self.sim_params_pub = self.create_publisher(
            Float32MultiArray, '/sim_params', qos_profile
        )
        self.real_params_pub = self.create_publisher(
            Float32MultiArray, '/real_params', qos_profile
        )
        self.domain_rand_pub = self.create_publisher(
            Float32MultiArray, '/domain_randomization', qos_profile
        )

        # Subscribers for transfer topics
        self.sim_params_sub = self.create_subscription(
            Float32MultiArray, '/sim_params', self.sim_params_callback, qos_profile
        )
        self.real_params_sub = self.create_subscription(
            Float32MultiArray, '/real_params', self.real_params_callback, qos_profile
        )
        self.domain_rand_sub = self.create_subscription(
            Float32MultiArray, '/domain_randomization', self.domain_rand_callback, qos_profile
        )

        # Services for parameter management
        self.sync_srv = self.create_service(
            Trigger, '/transfer/sync_params', self.sync_params_callback
        )
        self.get_sim_srv = self.create_service(
            Trigger, '/transfer/get_sim_params', self.get_sim_params_callback
        )
        self.get_real_srv = self.create_service(
            Trigger, '/transfer/get_real_params', self.get_real_params_callback
        )
        self.get_domain_srv = self.create_service(
            Trigger, '/transfer/get_domain_params', self.get_domain_params_callback
        )

        # Timer for periodic synchronization
        self.sync_timer = self.create_timer(1.0 / self.sync_rate, self.sync_parameters)

        # Initialize with default parameters
        self.initialize_default_parameters()

        self.get_logger().info('Transfer Topics Manager initialized')

    def initialize_default_parameters(self):
        """Initialize default parameters for simulation and real world"""
        # Default simulation parameters
        self.simulation_params = {
            'gravity': -9.81,
            'friction': 0.5,
            'restitution': 0.2,
            'damping': 0.1,
            'mass_factor': 1.0,
            'inertia_factor': 1.0,
            'com_offset_x': 0.0,
            'com_offset_y': 0.0,
            'com_offset_z': 0.0,
        }

        # Default real world parameters
        self.real_world_params = {
            'gravity': -9.81,
            'friction': 0.5,
            'restitution': 0.2,
            'damping': 0.1,
            'mass_factor': 1.0,
            'inertia_factor': 1.0,
            'com_offset_x': 0.0,
            'com_offset_y': 0.0,
            'com_offset_z': 0.0,
        }

        # Default domain randomization parameters
        self.domain_randomization_params = {
            'randomization_enabled': 1.0,  # 1.0 for true, 0.0 for false
            'randomization_range_min': 0.8,
            'randomization_range_max': 1.2,
            'update_rate': 1.0,
        }

    def sim_params_callback(self, msg: Float32MultiArray):
        """Callback for simulation parameters"""
        # In a real implementation, this would update internal simulation parameters
        self.get_logger().debug(f'Received simulation parameters: {len(msg.data)} values')

    def real_params_callback(self, msg: Float32MultiArray):
        """Callback for real world parameters"""
        # In a real implementation, this would update real robot parameters
        self.get_logger().debug(f'Received real world parameters: {len(msg.data)} values')

    def domain_rand_callback(self, msg: Float32MultiArray):
        """Callback for domain randomization parameters"""
        self.get_logger().debug(f'Received domain randomization parameters: {len(msg.data)} values')

    def sync_parameters(self):
        """Synchronize parameters across topics"""
        if not self.transfer_enabled:
            return

        # Publish simulation parameters
        sim_param_msg = Float32MultiArray()
        sim_values = [float(v) for v in self.simulation_params.values()]
        sim_param_msg.data = sim_values
        self.sim_params_pub.publish(sim_param_msg)

        # Publish real world parameters
        real_param_msg = Float32MultiArray()
        real_values = [float(v) for v in self.real_world_params.values()]
        real_param_msg.data = real_values
        self.real_params_pub.publish(real_param_msg)

        # Publish domain randomization parameters
        domain_param_msg = Float32MultiArray()
        domain_values = [float(v) for v in self.domain_randomization_params.values()]
        domain_param_msg.data = domain_values
        self.domain_rand_pub.publish(domain_param_msg)

    def sync_params_callback(self, request, response):
        """Service callback to manually trigger parameter sync"""
        self.sync_parameters()
        response.success = True
        response.message = 'Parameters synchronized'
        return response

    def get_sim_params_callback(self, request, response):
        """Service callback to get simulation parameters"""
        try:
            response.success = True
            response.message = f'Simulation parameters: {len(self.simulation_params)} total'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting simulation parameters: {str(e)}'
            return response

    def get_real_params_callback(self, request, response):
        """Service callback to get real world parameters"""
        try:
            response.success = True
            response.message = f'Real world parameters: {len(self.real_world_params)} total'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting real world parameters: {str(e)}'
            return response

    def get_domain_params_callback(self, request, response):
        """Service callback to get domain randomization parameters"""
        try:
            response.success = True
            response.message = f'Domain randomization parameters: {len(self.domain_randomization_params)} total'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting domain randomization parameters: {str(e)}'
            return response

    def update_simulation_params(self, new_params: Dict[str, float]):
        """Update simulation parameters"""
        for key, value in new_params.items():
            if key in self.simulation_params:
                self.simulation_params[key] = value
        self.get_logger().info(f'Updated {len(new_params)} simulation parameters')

    def update_real_params(self, new_params: Dict[str, float]):
        """Update real world parameters"""
        for key, value in new_params.items():
            if key in self.real_world_params:
                self.real_world_params[key] = value
        self.get_logger().info(f'Updated {len(new_params)} real world parameters')

    def update_domain_params(self, new_params: Dict[str, float]):
        """Update domain randomization parameters"""
        for key, value in new_params.items():
            if key in self.domain_randomization_params:
                self.domain_randomization_params[key] = value
        self.get_logger().info(f'Updated {len(new_params)} domain randomization parameters')

    def calculate_transfer_metrics(self) -> Dict[str, float]:
        """Calculate metrics for sim-to-real transfer quality"""
        metrics = {}

        # Calculate differences between simulation and real parameters
        differences = []
        for key in self.simulation_params:
            if key in self.real_world_params:
                diff = abs(self.simulation_params[key] - self.real_world_params[key])
                differences.append(diff)

        if differences:
            metrics['avg_param_diff'] = float(np.mean(differences))
            metrics['max_param_diff'] = float(np.max(differences))
            metrics['transfer_score'] = 1.0 / (1.0 + metrics['avg_param_diff'])  # Higher score = better transfer

        return metrics

    def get_current_parameters(self) -> Dict[str, Any]:
        """Get all current parameters"""
        return {
            'simulation': self.simulation_params.copy(),
            'real_world': self.real_world_params.copy(),
            'domain_randomization': self.domain_randomization_params.copy()
        }

    def enable_transfer(self):
        """Enable parameter transfer"""
        self.transfer_enabled = True
        self.get_logger().info('Parameter transfer enabled')

    def disable_transfer(self):
        """Disable parameter transfer"""
        self.transfer_enabled = False
        self.get_logger().info('Parameter transfer disabled')


def main(args=None):
    rclpy.init(args=args)
    manager = TransferTopicsManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()