#!/usr/bin/env python3

"""
Calibration Node for Sensor Calibration
This node implements calibration procedures for sensors in the humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header, Float64MultiArray
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu, Temperature
from geometry_msgs.msg import Point, Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time
import threading
import numpy as np
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
import json


@dataclass
class CalibrationData:
    """Data structure for calibration results"""
    sensor_name: str
    calibration_parameters: Dict[str, Any]
    timestamp: float
    accuracy: float  # 0.0 to 1.0
    status: str  # 'success', 'failed', 'in_progress'


class CalibrationNode(Node):
    """
    Calibration Node for sensor calibration
    Manages calibration procedures for various sensors in the robot system.
    """

    def __init__(self):
        super().__init__('calibration_node')

        # QoS profile for calibration messages
        self.calibration_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Calibration status publisher
        self.calibration_status_pub = self.create_publisher(String, '/calibration_status', self.calibration_qos)

        # Calibration data publisher
        self.calibration_data_pub = self.create_publisher(Float64MultiArray, '/calibration_data', self.calibration_qos)

        # Service clients for interacting with sensor drivers
        self.sensor_status_client = self.create_client(String, '/sensor_driver_node/status')
        self.hardware_interface_client = self.create_client(String, '/hardware_interface_node/status')

        # Dictionary to store calibration results
        self.calibration_results: Dict[str, CalibrationData] = {}
        self.calibration_lock = threading.Lock()

        # Timer for calibration monitoring
        self.calibration_monitor_timer = self.create_timer(2.0, self._monitor_calibration_status)

        # Parameter declarations
        self.declare_parameter('auto_calibration_enabled', True)
        self.declare_parameter('calibration_interval', 3600.0)  # 1 hour
        self.declare_parameter('calibration_threshold', 0.95)  # 95% accuracy threshold

        # Start auto calibration if enabled
        if self.get_parameter('auto_calibration_enabled').value:
            self.auto_calibration_timer = self.create_timer(
                self.get_parameter('calibration_interval').value,
                self._auto_calibration_check
            )

        self.get_logger().info('Calibration Node initialized')

    def _monitor_calibration_status(self):
        """Monitor calibration status and publish updates"""
        status_msg = String()
        status_info = {
            'timestamp': time.time(),
            'calibrated_sensors': list(self.calibration_results.keys()),
            'overall_status': 'operational'
        }

        # Check if any sensors need recalibration
        calibration_needed = []
        for sensor_name, calibration_data in self.calibration_results.items():
            if calibration_data.accuracy < self.get_parameter('calibration_threshold').value:
                calibration_needed.append(sensor_name)

        if calibration_needed:
            status_info['overall_status'] = 'recalibration_needed'
            status_info['calibration_needed'] = calibration_needed

        status_msg.data = json.dumps(status_info)
        self.calibration_status_pub.publish(status_msg)

    def _auto_calibration_check(self):
        """Check if auto calibration is needed for any sensors"""
        for sensor_name, calibration_data in self.calibration_results.items():
            if calibration_data.accuracy < self.get_parameter('calibration_threshold').value:
                self.get_logger().info(f'Starting auto calibration for {sensor_name}')
                self.start_sensor_calibration(sensor_name)

    def start_sensor_calibration(self, sensor_name: str) -> bool:
        """
        Start calibration procedure for a specific sensor
        Returns True if calibration started successfully
        """
        with self.calibration_lock:
            self.get_logger().info(f'Starting calibration for sensor: {sensor_name}')

            # Simulate different calibration procedures based on sensor type
            if sensor_name.startswith('camera'):
                result = self._calibrate_camera(sensor_name)
            elif sensor_name.startswith('imu'):
                result = self._calibrate_imu(sensor_name)
            elif sensor_name.startswith('lidar'):
                result = self._calibrate_lidar(sensor_name)
            elif sensor_name.startswith('temperature'):
                result = self._calibrate_temperature(sensor_name)
            else:
                result = self._calibrate_generic(sensor_name)

            if result:
                self.calibration_results[sensor_name] = result
                self._publish_calibration_data(sensor_name, result.calibration_parameters)
                self.get_logger().info(f'Calibration completed for {sensor_name} with accuracy: {result.accuracy}')
                return True
            else:
                self.get_logger().error(f'Calibration failed for {sensor_name}')
                return False

    def _calibrate_camera(self, sensor_name: str) -> Optional[CalibrationData]:
        """Calibrate camera sensor"""
        self.get_logger().info(f'Calibrating camera: {sensor_name}')

        # Simulate camera calibration process
        # This would typically involve showing calibration patterns and analyzing images
        time.sleep(2.0)  # Simulate calibration time

        # Generate calibration parameters
        calibration_params = {
            'camera_matrix': np.random.rand(3, 3).tolist(),
            'distortion_coefficients': np.random.rand(5).tolist(),
            'image_width': 640,
            'image_height': 480,
            'calibration_samples': 20
        }

        # Simulate accuracy based on calibration quality
        accuracy = 0.98 + np.random.rand() * 0.02  # 98-100%

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=calibration_params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def _calibrate_imu(self, sensor_name: str) -> Optional[CalibrationData]:
        """Calibrate IMU sensor"""
        self.get_logger().info(f'Calibrating IMU: {sensor_name}')

        # Simulate IMU calibration process
        # This would typically involve static positioning and motion patterns
        time.sleep(1.5)  # Simulate calibration time

        # Generate calibration parameters
        calibration_params = {
            'bias_accel_x': np.random.normal(0, 0.01),
            'bias_accel_y': np.random.normal(0, 0.01),
            'bias_accel_z': np.random.normal(0, 0.01),
            'bias_gyro_x': np.random.normal(0, 0.001),
            'bias_gyro_y': np.random.normal(0, 0.001),
            'bias_gyro_z': np.random.normal(0, 0.001),
            'calibration_samples': 100
        }

        # Simulate accuracy based on calibration quality
        accuracy = 0.96 + np.random.rand() * 0.04  # 96-100%

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=calibration_params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def _calibrate_lidar(self, sensor_name: str) -> Optional[CalibrationData]:
        """Calibrate LIDAR sensor"""
        self.get_logger().info(f'Calibrating LIDAR: {sensor_name}')

        # Simulate LIDAR calibration process
        # This would typically involve known reference objects and distance measurements
        time.sleep(3.0)  # Simulate calibration time

        # Generate calibration parameters
        calibration_params = {
            'range_offset': np.random.normal(0, 0.001),
            'angle_offset': np.random.normal(0, 0.0001),
            'intensity_calibration': [1.0 + np.random.normal(0, 0.01) for _ in range(360)],
            'calibration_samples': 50
        }

        # Simulate accuracy based on calibration quality
        accuracy = 0.97 + np.random.rand() * 0.03  # 97-100%

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=calibration_params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def _calibrate_temperature(self, sensor_name: str) -> Optional[CalibrationData]:
        """Calibrate temperature sensor"""
        self.get_logger().info(f'Calibrating temperature sensor: {sensor_name}')

        # Simulate temperature sensor calibration process
        # This would typically involve reference temperature sources
        time.sleep(1.0)  # Simulate calibration time

        # Generate calibration parameters
        calibration_params = {
            'offset': np.random.normal(0, 0.1),
            'scale_factor': 1.0 + np.random.normal(0, 0.001),
            'reference_temperature': 25.0,
            'calibration_samples': 10
        }

        # Simulate accuracy based on calibration quality
        accuracy = 0.99 + np.random.rand() * 0.01  # 99-100%

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=calibration_params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def _calibrate_generic(self, sensor_name: str) -> Optional[CalibrationData]:
        """Generic calibration for unknown sensor types"""
        self.get_logger().info(f'Calibrating generic sensor: {sensor_name}')

        time.sleep(1.0)  # Simulate calibration time

        # Generic calibration parameters
        calibration_params = {
            'offset': np.random.normal(0, 0.01),
            'scale': 1.0 + np.random.normal(0, 0.001),
            'reference_value': 0.0,
            'calibration_samples': 5
        }

        # Simulate accuracy
        accuracy = 0.95 + np.random.rand() * 0.05  # 95-100%

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=calibration_params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def _publish_calibration_data(self, sensor_name: str, calibration_params: Dict[str, Any]):
        """Publish calibration data for a sensor"""
        # Convert calibration parameters to Float64MultiArray
        flat_params = self._flatten_calibration_params(calibration_params)
        calibration_msg = Float64MultiArray()
        calibration_msg.data = flat_params

        # Add sensor name as metadata
        calibration_msg.layout.data_offset = 0

        self.calibration_data_pub.publish(calibration_msg)

        # Also publish to sensor-specific topic
        sensor_calibration_pub = self.create_publisher(
            Float64MultiArray,
            f'/{sensor_name}/calibration_data',
            self.calibration_qos
        )
        sensor_calibration_pub.publish(calibration_msg)

    def _flatten_calibration_params(self, params: Dict[str, Any]) -> List[float]:
        """Flatten calibration parameters dictionary to a list of floats"""
        flat_list = []
        for key, value in params.items():
            if isinstance(value, (int, float)):
                flat_list.append(float(value))
            elif isinstance(value, list):
                for item in value:
                    if isinstance(item, (int, float)):
                        flat_list.append(float(item))
                    elif isinstance(item, list):
                        # Handle nested lists (e.g., 2D arrays)
                        for nested_item in item:
                            if isinstance(nested_item, (int, float)):
                                flat_list.append(float(nested_item))
            elif isinstance(value, (np.ndarray, np.matrix)):
                flat_list.extend(value.flatten().astype(float).tolist())
        return flat_list

    def get_calibration_result(self, sensor_name: str) -> Optional[CalibrationData]:
        """Get calibration result for a specific sensor"""
        with self.calibration_lock:
            return self.calibration_results.get(sensor_name, None)

    def get_all_calibration_results(self) -> Dict[str, CalibrationData]:
        """Get all calibration results"""
        with self.calibration_lock:
            return self.calibration_results.copy()

    def is_sensor_calibrated(self, sensor_name: str) -> bool:
        """Check if a sensor is calibrated"""
        with self.calibration_lock:
            if sensor_name in self.calibration_results:
                result = self.calibration_results[sensor_name]
                return result.status == 'success' and result.accuracy >= self.get_parameter('calibration_threshold').value
            return False

    def reset_calibration(self, sensor_name: str) -> bool:
        """Reset calibration for a specific sensor"""
        with self.calibration_lock:
            if sensor_name in self.calibration_results:
                del self.calibration_results[sensor_name]
                self.get_logger().info(f'Reset calibration for {sensor_name}')
                return True
            return False

    def get_calibration_accuracy(self, sensor_name: str) -> float:
        """Get calibration accuracy for a specific sensor"""
        with self.calibration_lock:
            if sensor_name in self.calibration_results:
                return self.calibration_results[sensor_name].accuracy
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()

    # Start calibration for a few sample sensors
    sample_sensors = ['camera_front', 'imu_base', 'lidar_main', 'temperature_chassis']

    def calibrate_sensors():
        time.sleep(2.0)  # Wait for node to fully initialize
        for sensor in sample_sensors:
            calibration_node.start_sensor_calibration(sensor)

    # Start calibration in a separate thread to not block the main loop
    calibration_thread = threading.Thread(target=calibrate_sensors)
    calibration_thread.daemon = True
    calibration_thread.start()

    try:
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()