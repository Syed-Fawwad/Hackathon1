#!/usr/bin/env python3

"""
Sensor Services for Calibration and Configuration
This module implements services for sensor calibration and configuration in the humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header, Float64MultiArray
from sensor_msgs.msg import Image, Imu, LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
import time
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class CalibrationData:
    """Data structure for calibration results"""
    sensor_name: str
    calibration_parameters: Dict[str, Any]
    timestamp: float
    accuracy: float  # 0.0 to 1.0
    status: str  # 'success', 'failed', 'in_progress'


class SensorServicesNode(Node):
    """
    Node that provides sensor services: /calibrate_sensor and /get_calibration
    """

    def __init__(self):
        super().__init__('sensor_services_node')

        # Initialize calibration tracking
        self.calibration_results: Dict[str, CalibrationData] = {}

        # Create services
        self.calibrate_sensor_srv = self.create_service(
            CalibrateSensor,
            '/calibrate_sensor',
            self.calibrate_sensor_callback
        )

        self.get_calibration_srv = self.create_service(
            GetCalibration,
            '/get_calibration',
            self.get_calibration_callback
        )

        # Publishers for calibration updates
        self.calibration_status_pub = self.create_publisher(String, '/calibration_status', 10)

        self.get_logger().info('Sensor Services Node initialized with /calibrate_sensor and /get_calibration services')

    def calibrate_sensor_callback(self, request, response):
        """Callback for calibrate_sensor service"""
        sensor_name = request.sensor_name
        self.get_logger().info(f'Received request to calibrate sensor: {sensor_name}')

        # Validate sensor name
        if not sensor_name or len(sensor_name.strip()) == 0:
            response.success = False
            response.message = 'Invalid sensor name provided'
            self.get_logger().error('Invalid sensor name provided for calibration')
            return response

        # Simulate calibration process
        try:
            # In a real implementation, this would trigger actual calibration procedures
            # For simulation, we'll create a mock calibration result
            calibration_result = self._perform_sensor_calibration(sensor_name)

            if calibration_result:
                # Store the result
                self.calibration_results[sensor_name] = calibration_result

                # Publish status update
                status_msg = String()
                status_msg.data = f"Calibration completed for {sensor_name}: accuracy={calibration_result.accuracy}"
                self.calibration_status_pub.publish(status_msg)

                response.success = True
                response.message = f'Successfully calibrated {sensor_name}'
                response.calibration_accuracy = calibration_result.accuracy
                response.timestamp = str(calibration_result.timestamp)

                self.get_logger().info(f'Successfully calibrated sensor {sensor_name} with accuracy {calibration_result.accuracy}')
            else:
                response.success = False
                response.message = f'Failed to calibrate sensor {sensor_name}'
                response.calibration_accuracy = 0.0
                response.timestamp = str(time.time())

                self.get_logger().error(f'Failed to calibrate sensor {sensor_name}')

        except Exception as e:
            response.success = False
            response.message = f'Exception during calibration: {str(e)}'
            response.calibration_accuracy = 0.0
            response.timestamp = str(time.time())

            self.get_logger().error(f'Exception during calibration of {sensor_name}: {e}')

        return response

    def get_calibration_callback(self, request, response):
        """Callback for get_calibration service"""
        sensor_name = request.sensor_name
        self.get_logger().info(f'Received request for calibration data for: {sensor_name}')

        # Check if calibration exists for this sensor
        if sensor_name in self.calibration_results:
            calibration_data = self.calibration_results[sensor_name]

            response.success = True
            response.calibration_accuracy = calibration_data.accuracy
            response.calibration_parameters = json.dumps(calibration_data.calibration_parameters)
            response.timestamp = str(calibration_data.timestamp)
            response.status = calibration_data.status
            response.message = f'Calibration data retrieved for {sensor_name}'

            self.get_logger().info(f'Returned calibration data for {sensor_name}')
        else:
            response.success = False
            response.calibration_accuracy = 0.0
            response.calibration_parameters = '{}'
            response.timestamp = str(time.time())
            response.status = 'not_calibrated'
            response.message = f'No calibration data available for {sensor_name}'

            self.get_logger().warn(f'No calibration data available for {sensor_name}')

        return response

    def _perform_sensor_calibration(self, sensor_name: str) -> Optional[CalibrationData]:
        """Simulate performing sensor calibration"""
        self.get_logger().info(f'Performing calibration for {sensor_name}')

        # Simulate calibration time
        time.sleep(0.5)  # Simulate processing time

        # Determine sensor type based on name
        if 'camera' in sensor_name.lower():
            accuracy = 0.98 + (time.time() % 0.02)  # 98-100%
            params = {
                'camera_matrix': [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                'distortion_coeffs': [0.1, -0.2, 0, 0, 0.1],
                'image_size': [640, 480]
            }
        elif 'imu' in sensor_name.lower():
            accuracy = 0.97 + (time.time() % 0.03)  # 97-100%
            params = {
                'accel_bias': [0.001, -0.002, 0.003],
                'gyro_bias': [0.0001, -0.0002, 0.0003],
                'mag_bias': [0.01, -0.02, 0.03]
            }
        elif 'lidar' in sensor_name.lower():
            accuracy = 0.96 + (time.time() % 0.04)  # 96-100%
            params = {
                'range_offset': 0.001,
                'angle_offset': 0.0001,
                'intensity_cal': [1.0] * 360
            }
        else:
            # Generic sensor
            accuracy = 0.95 + (time.time() % 0.05)  # 95-100%
            params = {
                'offset': 0.001,
                'scale': 1.001,
                'reference_value': 0.0
            }

        return CalibrationData(
            sensor_name=sensor_name,
            calibration_parameters=params,
            timestamp=time.time(),
            accuracy=min(accuracy, 1.0),
            status='success'
        )

    def get_all_calibrations(self) -> Dict[str, CalibrationData]:
        """Get all stored calibration results"""
        return self.calibration_results.copy()

    def reset_calibration(self, sensor_name: str) -> bool:
        """Reset calibration for a specific sensor"""
        if sensor_name in self.calibration_results:
            del self.calibration_results[sensor_name]
            self.get_logger().info(f'Reset calibration for {sensor_name}')
            return True
        return False


# Define service interfaces since they're not in standard ROS 2 interfaces
# In a real implementation, these would be defined in .srv files and generated
# For this example, we'll define simple service classes

class CalibrateSensor:
    class Request:
        def __init__(self):
            self.sensor_name = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""
            self.calibration_accuracy = 0.0
            self.timestamp = ""


class GetCalibration:
    class Request:
        def __init__(self):
            self.sensor_name = ""

    class Response:
        def __init__(self):
            self.success = False
            self.calibration_accuracy = 0.0
            self.calibration_parameters = ""
            self.timestamp = ""
            self.status = ""
            self.message = ""


def main(args=None):
    rclpy.init(args=args)
    sensor_services_node = SensorServicesNode()

    try:
        rclpy.spin(sensor_services_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_services_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()