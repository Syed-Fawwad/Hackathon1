#!/usr/bin/env python3

"""
Transfer Services for Simulation Parameters and Transfer Validation
This node implements services: /update_simulation_params, /validate_transfer
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Float32MultiArray
from std_srvs.srv import Trigger, SetBool
from rclpy.parameter import Parameter
import numpy as np
from typing import Dict, List, Any, Optional
import json
from builtin_interfaces.msg import Time


class TransferServices(Node):
    """
    Transfer Services Node
    Implements services: /update_simulation_params, /validate_transfer
    """

    def __init__(self):
        super().__init__('transfer_services')

        # Initialize parameters
        self.declare_parameter('transfer_validation_threshold', 0.9)
        self.declare_parameter('param_update_enabled', True)

        self.validation_threshold = self.get_parameter('transfer_validation_threshold').value
        self.param_update_enabled = self.get_parameter('param_update_enabled').value

        # Store simulation parameters
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

        # Store transfer validation results
        self.validation_results = {}
        self.last_validation_time = None

        # Publishers
        qos_profile = QoSProfile(depth=10)
        self.param_update_pub = self.create_publisher(
            Float32MultiArray, '/simulation/params_updated', qos_profile
        )
        self.validation_pub = self.create_publisher(
            String, '/transfer/validation_result', qos_profile
        )

        # Services
        self.update_params_srv = self.create_service(
            # We'll use a generic service type since we need to pass parameter data
            # In a real implementation, we'd create a custom service type
            SetBool,  # Using SetBool as placeholder - would use custom type in practice
            '/update_simulation_params',
            self.update_simulation_params_callback
        )

        self.validate_transfer_srv = self.create_service(
            Trigger,
            '/validate_transfer',
            self.validate_transfer_callback
        )

        # Additional service for more complex parameter updates
        self.complex_update_srv = self.create_service(
            Trigger,  # Using Trigger and parsing parameters from node parameters
            '/update_simulation_params_complex',
            self.update_simulation_params_complex_callback
        )

        self.get_logger().info('Transfer Services Node initialized')

    def update_simulation_params_callback(self, request, response):
        """Service callback to update simulation parameters"""
        if not self.param_update_enabled:
            response.success = False
            response.message = 'Parameter updates are disabled'
            return response

        try:
            # In a real implementation, this would receive specific parameter data
            # For now, we'll use node parameters to simulate the update

            # Check if specific parameters were set as node parameters
            for param_name in self.simulation_params.keys():
                try:
                    if self.has_parameter(param_name):
                        new_value = self.get_parameter(param_name).value
                        self.simulation_params[param_name] = new_value
                        self.get_logger().info(f'Updated {param_name} to {new_value}')
                except:
                    # Parameter doesn't exist, continue
                    continue

            # Publish update notification
            update_msg = Float32MultiArray()
            update_msg.data = [float(v) for v in self.simulation_params.values()]
            self.param_update_pub.publish(update_msg)

            response.success = True
            response.message = f'Updated {len(self.simulation_params)} simulation parameters'

            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error updating simulation parameters: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def update_simulation_params_complex_callback(self, request, response):
        """Service callback for complex parameter updates using node parameters"""
        if not self.param_update_enabled:
            response.success = False
            response.message = 'Parameter updates are disabled'
            return response

        try:
            # Get all parameters that start with 'sim_param_' and update simulation params
            updated_count = 0

            # Declare and get all possible parameters
            for param_name, default_value in self.simulation_params.items():
                full_param_name = f'sim_param_{param_name}'
                self.declare_parameter(full_param_name, default_value)

                new_value = self.get_parameter(full_param_name).value
                if self.simulation_params[param_name] != new_value:
                    self.simulation_params[param_name] = new_value
                    updated_count += 1
                    self.get_logger().info(f'Updated {param_name} to {new_value}')

            if updated_count > 0:
                # Publish update notification
                update_msg = Float32MultiArray()
                update_msg.data = [float(v) for v in self.simulation_params.values()]
                self.param_update_pub.publish(update_msg)

                response.success = True
                response.message = f'Updated {updated_count} simulation parameters'
            else:
                response.success = True
                response.message = 'No parameters were updated (values unchanged)'

            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error updating simulation parameters: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def validate_transfer_callback(self, request, response):
        """Service callback to validate transfer between sim and real"""
        try:
            # Perform transfer validation (simplified example)
            validation_result = self.perform_transfer_validation()

            # Create validation result message
            validation_msg = String()
            validation_msg.data = json.dumps(validation_result)
            self.validation_pub.publish(validation_msg)

            # Store results
            self.validation_results = validation_result
            self.last_validation_time = self.get_clock().now()

            # Determine if transfer is valid based on threshold
            transfer_score = validation_result.get('transfer_score', 0.0)
            is_valid = transfer_score >= self.validation_threshold

            response.success = is_valid
            if is_valid:
                response.message = f'Transfer validation PASSED with score: {transfer_score:.3f} (threshold: {self.validation_threshold})'
            else:
                response.message = f'Transfer validation FAILED with score: {transfer_score:.3f} (threshold: {self.validation_threshold})'

            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error validating transfer: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def perform_transfer_validation(self) -> Dict[str, Any]:
        """Perform transfer validation between simulation and real world"""
        # This is a simplified validation approach
        # In practice, this would compare simulation behavior to real robot behavior

        # Example validation metrics
        validation_metrics = {
            'transfer_score': 0.0,
            'param_similarity': 0.0,
            'behavior_similarity': 0.0,
            'tracking_error_avg': 0.0,
            'tracking_error_std': 0.0,
            'validation_timestamp': self.get_clock().now().nanoseconds
        }

        # Calculate a simple transfer score based on parameter similarity
        # In a real system, this would involve comparing actual behaviors
        param_diffs = []
        for key, sim_val in self.simulation_params.items():
            # Compare to some "real world" parameters (in practice, obtained from real robot)
            real_val = sim_val * 0.95 + np.random.normal(0, 0.01)  # Simulate small real-world differences
            diff = abs(sim_val - real_val)
            param_diffs.append(diff)

        if param_diffs:
            avg_diff = np.mean(param_diffs)
            # Convert to similarity score (higher = more similar)
            similarity_score = 1.0 / (1.0 + avg_diff * 10)  # Adjust multiplier as needed
            validation_metrics['param_similarity'] = float(similarity_score)
            validation_metrics['transfer_score'] = float(similarity_score)

        return validation_metrics

    def get_simulation_parameters(self) -> Dict[str, float]:
        """Get current simulation parameters"""
        return self.simulation_params.copy()

    def set_simulation_parameter(self, param_name: str, value: float) -> bool:
        """Set a specific simulation parameter"""
        if param_name in self.simulation_params:
            self.simulation_params[param_name] = value
            self.get_logger().info(f'Set {param_name} to {value}')
            return True
        else:
            self.get_logger().warn(f'Parameter {param_name} not found')
            return False

    def get_validation_results(self) -> Dict[str, Any]:
        """Get the latest validation results"""
        return self.validation_results.copy()

    def set_validation_threshold(self, threshold: float):
        """Set the validation threshold"""
        self.validation_threshold = threshold
        self.get_logger().info(f'Validation threshold set to {threshold}')

    def get_validation_threshold(self) -> float:
        """Get the current validation threshold"""
        return self.validation_threshold

    def enable_param_updates(self):
        """Enable parameter updates"""
        self.param_update_enabled = True
        self.get_logger().info('Parameter updates enabled')

    def disable_param_updates(self):
        """Disable parameter updates"""
        self.param_update_enabled = False
        self.get_logger().info('Parameter updates disabled')


def main(args=None):
    rclpy.init(args=args)
    services = TransferServices()

    try:
        rclpy.spin(services)
    except KeyboardInterrupt:
        pass
    finally:
        services.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()