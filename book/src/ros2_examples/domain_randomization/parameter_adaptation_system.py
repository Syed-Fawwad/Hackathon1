#!/usr/bin/env python3

"""
Parameter Adaptation System for Transfer Optimization
This system adapts parameters to optimize transfer between simulation and reality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Float32MultiArray
from std_srvs.srv import Trigger, SetBool
from rclpy.parameter import Parameter
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
import time
from collections import deque
import threading
import pickle


class ParameterAdaptationSystem(Node):
    """
    Parameter Adaptation System
    Adapts parameters to optimize transfer between simulation and reality.
    """

    def __init__(self):
        super().__init__('parameter_adaptation_system')

        # Initialize parameters
        self.declare_parameter('adaptation_enabled', True)
        self.declare_parameter('adaptation_rate', 0.1)  # Hz (slow for computation)
        self.declare_parameter('learning_rate', 0.01)
        self.declare_parameter('convergence_threshold', 0.001)
        self.declare_parameter('max_iterations', 100)

        self.adaptation_enabled = self.get_parameter('adaptation_enabled').value
        self.adaptation_rate = self.get_parameter('adaptation_rate').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.max_iterations = self.get_parameter('max_iterations').value

        # Store performance data
        self.sim_performance = deque(maxlen=100)
        self.real_performance = deque(maxlen=100)
        self.parameter_history = deque(maxlen=100)

        # Current parameters to adapt
        self.current_parameters = {
            # Physical parameters
            'sim_gravity': -9.81,
            'sim_friction': 0.5,
            'sim_restitution': 0.2,
            'robot_mass': 1.0,
            'robot_com_offset_x': 0.0,
            'robot_com_offset_y': 0.0,
            'robot_com_offset_z': 0.0,

            # Control parameters
            'kp': 1.0,  # Proportional gain
            'ki': 0.1,  # Integral gain
            'kd': 0.05, # Derivative gain

            # Sensor parameters
            'sensor_noise_std': 0.01,
            'sensor_bias': 0.0,
        }

        # Target parameters (initially same as current)
        self.target_parameters = self.current_parameters.copy()

        # Performance metrics
        self.sim_performance_metrics = {}
        self.real_performance_metrics = {}

        # Publishers
        qos_profile = QoSProfile(depth=10)
        self.param_pub = self.create_publisher(
            Float32MultiArray, '/parameter_adaptation/parameters', qos_profile
        )
        self.status_pub = self.create_publisher(
            String, '/parameter_adaptation/status', qos_profile
        )
        self.performance_pub = self.create_publisher(
            Float32MultiArray, '/parameter_adaptation/performance', qos_profile
        )

        # Subscribers
        self.sim_performance_sub = self.create_subscription(
            Float32MultiArray, '/simulation/performance', self.sim_performance_callback, qos_profile
        )
        self.real_performance_sub = self.create_subscription(
            Float32MultiArray, '/reality/performance', self.real_performance_callback, qos_profile
        )

        # Services
        self.start_srv = self.create_service(
            SetBool, '/parameter_adaptation/enable', self.enable_callback
        )
        self.adapt_srv = self.create_service(
            Trigger, '/parameter_adaptation/adapt', self.adapt_callback
        )
        self.get_params_srv = self.create_service(
            Trigger, '/parameter_adaptation/get_params', self.get_params_callback
        )
        self.save_params_srv = self.create_service(
            Trigger, '/parameter_adaptation/save', self.save_params_callback
        )
        self.load_params_srv = self.create_service(
            Trigger, '/parameter_adaptation/load', self.load_params_callback
        )

        # Timer for periodic adaptation
        self.adaptation_timer = self.create_timer(
            1.0 / self.adaptation_rate, self.adapt_parameters
        )

        # Lock for thread safety
        self.data_lock = threading.Lock()

        self.get_logger().info('Parameter Adaptation System initialized')

    def sim_performance_callback(self, msg: Float32MultiArray):
        """Callback for simulation performance data"""
        with self.data_lock:
            self.sim_performance.append(list(msg.data))
            self.sim_performance_metrics = self.calculate_performance_metrics(msg.data)

    def real_performance_callback(self, msg: Float32MultiArray):
        """Callback for real-world performance data"""
        with self.data_lock:
            self.real_performance.append(list(msg.data))
            self.real_performance_metrics = self.calculate_performance_metrics(msg.data)

    def calculate_performance_metrics(self, data: List[float]) -> Dict[str, float]:
        """Calculate performance metrics from raw data"""
        if len(data) == 0:
            return {}

        metrics = {
            'mean': float(np.mean(data)),
            'std': float(np.std(data)),
            'min': float(np.min(data)),
            'max': float(np.max(data)),
            'variance': float(np.var(data))
        }
        return metrics

    def adapt_parameters(self):
        """Adapt parameters based on performance difference"""
        if not self.adaptation_enabled:
            return

        if len(self.sim_performance) < 2 or len(self.real_performance) < 2:
            return

        try:
            # Calculate performance difference
            with self.data_lock:
                sim_metrics = self.sim_performance_metrics
                real_metrics = self.real_performance_metrics

            if not sim_metrics or not real_metrics:
                return

            # Calculate adaptation based on performance difference
            adaptation_result = self.calculate_adaptation(sim_metrics, real_metrics)

            # Update parameters
            self.update_parameters(adaptation_result)

            # Publish updated parameters
            self.publish_parameters()

            # Log status
            status_msg = String()
            status_msg.data = f'Parameters adapted: {len(adaptation_result)} parameters updated'
            self.status_pub.publish(status_msg)

            self.get_logger().info(f'Adapted parameters: {list(adaptation_result.keys())}')

        except Exception as e:
            self.get_logger().error(f'Error in parameter adaptation: {e}')

    def calculate_adaptation(self, sim_metrics: Dict[str, float],
                           real_metrics: Dict[str, float]) -> Dict[str, float]:
        """Calculate parameter adaptation based on performance metrics"""
        adaptation = {}

        # Example adaptation logic - in practice this would be more sophisticated
        for param_name, current_value in self.current_parameters.items():
            # Calculate difference in performance metrics
            sim_val = sim_metrics.get('mean', 0.0)
            real_val = real_metrics.get('mean', 0.0)

            # Calculate error
            error = real_val - sim_val

            # Apply adaptation based on learning rate
            adaptation[param_name] = self.learning_rate * error

        return adaptation

    def update_parameters(self, adaptation: Dict[str, float]):
        """Update parameters based on adaptation values"""
        for param_name, adaptation_value in adaptation.items():
            if param_name in self.current_parameters:
                old_value = self.current_parameters[param_name]
                new_value = old_value + adaptation_value

                # Apply bounds to prevent unrealistic values
                new_value = self.apply_parameter_bounds(param_name, new_value)

                self.current_parameters[param_name] = new_value

        # Store in history
        self.parameter_history.append(self.current_parameters.copy())

    def apply_parameter_bounds(self, param_name: str, value: float) -> float:
        """Apply bounds to parameter values to prevent unrealistic values"""
        bounds = {
            'sim_gravity': (-12.0, -8.0),
            'sim_friction': (0.0, 2.0),
            'sim_restitution': (0.0, 1.0),
            'robot_mass': (0.1, 10.0),
            'robot_com_offset_x': (-0.1, 0.1),
            'robot_com_offset_y': (-0.1, 0.1),
            'robot_com_offset_z': (-0.1, 0.1),
            'kp': (0.0, 10.0),
            'ki': (0.0, 2.0),
            'kd': (0.0, 1.0),
            'sensor_noise_std': (0.0, 0.5),
            'sensor_bias': (-0.1, 0.1),
        }

        if param_name in bounds:
            min_val, max_val = bounds[param_name]
            value = max(min_val, min(max_val, value))

        return value

    def publish_parameters(self):
        """Publish current parameters"""
        param_msg = Float32MultiArray()
        param_values = []

        for key, value in self.current_parameters.items():
            param_values.append(float(value))

        param_msg.data = param_values
        self.param_pub.publish(param_msg)

        # Also publish performance metrics
        perf_msg = Float32MultiArray()
        perf_values = [
            len(self.sim_performance),
            len(self.real_performance),
            self.sim_performance_metrics.get('mean', 0.0) if self.sim_performance_metrics else 0.0,
            self.real_performance_metrics.get('mean', 0.0) if self.real_performance_metrics else 0.0,
        ]
        perf_msg.data = perf_values
        self.performance_pub.publish(perf_msg)

    def enable_callback(self, request, response):
        """Service callback to enable/disable adaptation"""
        self.adaptation_enabled = request.data
        response.success = True
        response.message = f'Parameter adaptation {"enabled" if self.adaptation_enabled else "disabled"}'

        self.get_logger().info(response.message)
        return response

    def adapt_callback(self, request, response):
        """Service callback to manually trigger adaptation"""
        if self.adaptation_enabled:
            self.adapt_parameters()
            response.success = True
            response.message = 'Parameter adaptation triggered'
        else:
            response.success = False
            response.message = 'Parameter adaptation is disabled'

        return response

    def get_params_callback(self, request, response):
        """Service callback to get current parameters"""
        try:
            response.success = True
            response.message = f'Current parameters: {len(self.current_parameters)} total'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error getting parameters: {str(e)}'
            return response

    def save_params_callback(self, request, response):
        """Service callback to save parameters to file"""
        try:
            # Save current parameters and state
            state = {
                'current_parameters': self.current_parameters,
                'parameter_history': list(self.parameter_history),
                'sim_performance': list(self.sim_performance),
                'real_performance': list(self.real_performance),
                'adaptation_enabled': self.adaptation_enabled
            }

            filename = f'/tmp/parameter_adaptation_state_{int(time.time())}.pkl'
            with open(filename, 'wb') as f:
                pickle.dump(state, f)

            response.success = True
            response.message = f'Parameters saved to {filename}'
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error saving parameters: {str(e)}'
            return response

    def load_params_callback(self, request, response):
        """Service callback to load parameters from file"""
        try:
            # In a real implementation, we would specify the filename
            # For now, we'll look for the most recent save file
            import glob
            files = glob.glob('/tmp/parameter_adaptation_state_*.pkl')
            if files:
                latest_file = max(files, key=lambda x: x)
                with open(latest_file, 'rb') as f:
                    state = pickle.load(f)

                self.current_parameters = state.get('current_parameters', self.current_parameters)
                self.parameter_history = deque(state.get('parameter_history', []), maxlen=100)
                self.sim_performance = deque(state.get('sim_performance', []), maxlen=100)
                self.real_performance = deque(state.get('real_performance', []), maxlen=100)
                self.adaptation_enabled = state.get('adaptation_enabled', self.adaptation_enabled)

                response.success = True
                response.message = f'Parameters loaded from {latest_file}'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'No saved parameter files found'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Error loading parameters: {str(e)}'
            return response

    def get_current_parameters(self) -> Dict[str, float]:
        """Get current parameters"""
        return self.current_parameters.copy()

    def set_target_parameters(self, target_params: Dict[str, float]):
        """Set target parameters for adaptation"""
        for key, value in target_params.items():
            if key in self.current_parameters:
                self.target_parameters[key] = value

    def calculate_transfer_score(self) -> float:
        """Calculate transfer score between simulation and reality"""
        with self.data_lock:
            if not self.sim_performance_metrics or not self.real_performance_metrics:
                return 0.0

            # Calculate similarity between simulation and reality
            sim_mean = self.sim_performance_metrics.get('mean', 0.0)
            real_mean = self.real_performance_metrics.get('mean', 0.0)

            # Calculate score based on difference (lower difference = higher score)
            diff = abs(sim_mean - real_mean)
            score = 1.0 / (1.0 + diff)  # Normalize to [0, 1] range

            return score

    def reset_adaptation(self):
        """Reset adaptation process"""
        with self.data_lock:
            self.sim_performance.clear()
            self.real_performance.clear()
            self.parameter_history.clear()
            # Reset to initial parameters
            self.current_parameters = {
                'sim_gravity': -9.81,
                'sim_friction': 0.5,
                'sim_restitution': 0.2,
                'robot_mass': 1.0,
                'robot_com_offset_x': 0.0,
                'robot_com_offset_y': 0.0,
                'robot_com_offset_z': 0.0,
                'kp': 1.0,
                'ki': 0.1,
                'kd': 0.05,
                'sensor_noise_std': 0.01,
                'sensor_bias': 0.0,
            }

        self.get_logger().info('Parameter adaptation reset')


def main(args=None):
    rclpy.init(args=args)
    system = ParameterAdaptationSystem()

    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()