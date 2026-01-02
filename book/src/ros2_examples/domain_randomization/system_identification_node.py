#!/usr/bin/env python3

"""
System Identification Node for Model Validation
This node performs system identification to validate robot models and improve sim-to-real transfer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger, SetBool
from rclpy.parameter import Parameter
import numpy as np
import scipy.optimize as opt
from typing import Dict, List, Any, Optional, Tuple
import time
import threading
from collections import deque
import matplotlib.pyplot as plt


class SystemIdentificationNode(Node):
    """
    System Identification Node
    Performs system identification to validate robot models and improve sim-to-real transfer.
    """

    def __init__(self):
        super().__init__('system_identification_node')

        # Initialize parameters
        self.declare_parameter('identification_enabled', True)
        self.declare_parameter('identification_window_size', 1000)
        self.declare_parameter('identification_update_rate', 0.1)  # Hz (slow for computation)
        self.declare_parameter('excitation_signal_amplitude', 0.5)
        self.declare_parameter('excitation_signal_frequency', 0.5)

        self.identification_enabled = self.get_parameter('identification_enabled').value
        self.window_size = self.get_parameter('identification_window_size').value
        self.update_rate = self.get_parameter('identification_update_rate').value
        self.excitation_amplitude = self.get_parameter('excitation_signal_amplitude').value
        self.excitation_frequency = self.get_parameter('excitation_signal_frequency').value

        # Data storage for system identification
        self.joint_data = deque(maxlen=self.window_size)
        self.imu_data = deque(maxlen=self.window_size)
        self.command_data = deque(maxlen=self.window_size)
        self.timestamp_data = deque(maxlen=self.window_size)

        # Identified system parameters
        self.system_parameters = {
            'mass': 1.0,  # Identified mass
            'inertia': 0.1,  # Identified inertia
            'friction_coeff': 0.1,  # Identified friction
            'com_offset': [0.0, 0.0, 0.0],  # Center of mass offset
            'sensor_bias': [0.0, 0.0, 0.0],  # Sensor bias
        }

        # Publishers
        qos_profile = QoSProfile(depth=10)
        self.param_pub = self.create_publisher(
            Float32MultiArray, '/system_identification/parameters', qos_profile
        )
        self.status_pub = self.create_publisher(
            String, '/system_identification/status', qos_profile
        )
        self.excitation_pub = self.create_publisher(
            JointState, '/excitation_commands', qos_profile
        )

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_profile
        )
        self.command_sub = self.create_subscription(
            Twist, '/cmd_vel', self.command_callback, qos_profile
        )

        # Services
        self.start_srv = self.create_service(
            Trigger, '/system_identification/start', self.start_identification_callback
        )
        self.stop_srv = self.create_service(
            Trigger, '/system_identification/stop', self.stop_identification_callback
        )
        self.get_params_srv = self.create_service(
            Trigger, '/system_identification/get_params', self.get_params_callback
        )
        self.validate_srv = self.create_service(
            Trigger, '/system_identification/validate', self.validate_callback
        )

        # Timer for periodic identification
        self.identification_timer = self.create_timer(
            1.0 / self.update_rate, self.perform_system_identification
        )

        # Excitation timer to generate input signals
        self.excitation_timer = self.create_timer(
            0.1, self.generate_excitation_signal  # 10 Hz excitation
        )

        # Lock for thread safety
        self.data_lock = threading.Lock()

        self.get_logger().info('System Identification Node initialized')

    def joint_state_callback(self, msg: JointState):
        """Callback for joint state messages"""
        with self.data_lock:
            self.joint_data.append({
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort),
                'timestamp': self.get_clock().now().nanoseconds
            })

    def imu_callback(self, msg: Imu):
        """Callback for IMU messages"""
        with self.data_lock:
            self.imu_data.append({
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'orientation': [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                'timestamp': self.get_clock().now().nanoseconds
            })

    def command_callback(self, msg: Twist):
        """Callback for command messages"""
        with self.data_lock:
            self.command_data.append({
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z,
                'timestamp': self.get_clock().now().nanoseconds
            })

    def generate_excitation_signal(self):
        """Generate excitation signals to excite the system"""
        if not self.identification_enabled:
            return

        # Create joint state message with excitation
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint_1', 'joint_2', 'joint_3']  # Example joint names

        # Generate sinusoidal excitation signals
        current_time = time.time()
        positions = []
        for i, joint_name in enumerate(joint_msg.name):
            # Different frequencies for each joint to excite different modes
            freq = self.excitation_frequency * (i + 1)
            pos = self.excitation_amplitude * np.sin(2 * np.pi * freq * current_time)
            positions.append(pos)

        joint_msg.position = positions
        self.excitation_pub.publish(joint_msg)

    def perform_system_identification(self):
        """Perform system identification using collected data"""
        if not self.identification_enabled:
            return

        with self.data_lock:
            # Check if we have enough data
            if len(self.joint_data) < 10 or len(self.command_data) < 10:
                return

            # Extract data for identification
            joint_positions = []
            joint_velocities = []
            commands = []

            for joint_state in self.joint_data:
                if joint_state['position']:
                    joint_positions.append(joint_state['position'])
                    joint_velocities.append(joint_state['velocity'])

            for cmd in self.command_data:
                commands.append([cmd['linear_x'], cmd['angular_z']])  # Example

        if len(joint_positions) < 10:
            return

        try:
            # Perform system identification (simplified example)
            # In practice, this would involve more complex system identification algorithms
            identified_params = self.identify_system_parameters(
                joint_positions, joint_velocities, commands
            )

            # Update system parameters
            self.system_parameters.update(identified_params)

            # Publish updated parameters
            self.publish_system_parameters()

            # Log status
            status_msg = String()
            status_msg.data = f'System identification completed: {len(identified_params)} parameters updated'
            self.status_pub.publish(status_msg)

            self.get_logger().info(f'System identification updated parameters: {identified_params}')

        except Exception as e:
            self.get_logger().error(f'Error in system identification: {e}')

    def identify_system_parameters(self, positions: List[List[float]],
                                 velocities: List[List[float]],
                                 commands: List[List[float]]) -> Dict[str, float]:
        """Perform system identification to estimate physical parameters"""
        # This is a simplified example - real system identification would be much more complex
        if len(positions) < 2 or len(velocities) < 2 or len(commands) < 2:
            return {}

        # Example: Estimate simple dynamic parameters
        # Calculate average velocity from position differences
        if len(positions) >= 2:
            dt = 0.1  # Assume 10Hz sampling
            vel_estimates = []
            for i in range(1, len(positions)):
                pos_diff = np.array(positions[i]) - np.array(positions[i-1])
                vel = pos_diff / dt
                vel_estimates.append(vel)

            if vel_estimates:
                avg_velocity = np.mean(vel_estimates, axis=0)
                # Example parameter estimation (simplified)
                estimated_mass = float(np.mean(np.abs(avg_velocity)) * 2.0)  # Placeholder calculation
                estimated_friction = float(np.std(avg_velocity) * 0.5)  # Placeholder calculation

                return {
                    'mass': max(0.1, min(10.0, estimated_mass)),  # Clamp to reasonable range
                    'friction_coeff': max(0.0, min(1.0, estimated_friction))
                }

        return {}

    def publish_system_parameters(self):
        """Publish identified system parameters"""
        param_msg = Float32MultiArray()
        param_values = []

        for key, value in self.system_parameters.items():
            if isinstance(value, (int, float)):
                param_values.append(float(value))
            elif isinstance(value, (list, tuple)):
                param_values.extend([float(v) for v in value])

        param_msg.data = param_values
        self.param_pub.publish(param_msg)

    def start_identification_callback(self, request, response):
        """Service callback to start system identification"""
        self.identification_enabled = True
        response.success = True
        response.message = 'System identification started'

        self.get_logger().info('System identification started')
        return response

    def stop_identification_callback(self, request, response):
        """Service callback to stop system identification"""
        self.identification_enabled = False
        response.success = True
        response.message = 'System identification stopped'

        self.get_logger().info('System identification stopped')
        return response

    def get_params_callback(self, request, response):
        """Service callback to get identified parameters"""
        try:
            # In a real implementation, we would return a custom message
            # For now, we'll just return success
            response.success = True
            response.message = f'System parameters: {self.system_parameters}'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting parameters: {str(e)}'
            return response

    def validate_callback(self, request, response):
        """Service callback to validate the identified model"""
        try:
            # Perform validation by comparing predicted vs actual behavior
            validation_result = self.validate_model()
            response.success = validation_result['success']
            response.message = validation_result['message']
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error validating model: {str(e)}'
            return response

    def validate_model(self) -> Dict[str, Any]:
        """Validate the identified model against actual system behavior"""
        with self.data_lock:
            if len(self.joint_data) < 10:
                return {
                    'success': False,
                    'message': 'Not enough data for validation'
                }

            # Compare model predictions with actual measurements
            # This is a simplified validation approach
            # In practice, you would simulate the identified model and compare trajectories

            # Calculate some basic metrics
            position_std = np.std([state['position'] for state in self.joint_data if state['position']])
            velocity_std = np.std([state['velocity'] for state in self.joint_data if state['velocity']])

            # Simple validation: check if std is reasonable
            if np.any(position_std > 10.0) or np.any(velocity_std > 10.0):
                return {
                    'success': False,
                    'message': 'Model validation failed: unrealistic parameter values detected'
                }

            return {
                'success': True,
                'message': f'Model validation passed: position_std={np.mean(position_std):.3f}, velocity_std={np.mean(velocity_std):.3f}'
            }

    def get_system_parameters(self) -> Dict[str, Any]:
        """Get the current system parameters"""
        return self.system_parameters.copy()

    def set_parameter_bounds(self, param_name: str, min_val: float, max_val: float):
        """Set bounds for a system parameter"""
        # This would be used to constrain parameter estimation
        pass

    def reset_data_buffers(self):
        """Reset all data buffers"""
        with self.data_lock:
            self.joint_data.clear()
            self.imu_data.clear()
            self.command_data.clear()
            self.timestamp_data.clear()

        self.get_logger().info('System identification data buffers reset')


def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()