#!/usr/bin/env python3

"""
Hardware Interface Node for Sensor Abstraction
This node provides a standardized interface to hardware devices for sensor abstraction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu, JointState, Temperature, BatteryState
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
import threading
import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass


@dataclass
class DeviceConfig:
    """Configuration for a hardware device"""
    device_path: str
    device_type: str
    parameters: Dict[str, Any]
    enabled: bool = True


class HardwareDevice:
    """
    Abstract representation of a hardware device
    """

    def __init__(self, config: DeviceConfig):
        self.config = config
        self.is_connected = False
        self.is_calibrated = False
        self.last_read_time = 0
        self.lock = threading.Lock()

    def connect(self) -> bool:
        """Connect to the hardware device"""
        with self.lock:
            # In a real implementation, this would establish a connection to the actual device
            self.is_connected = True
            return True

    def disconnect(self):
        """Disconnect from the hardware device"""
        with self.lock:
            self.is_connected = False

    def read_data(self) -> Optional[Any]:
        """Read data from the hardware device"""
        if not self.is_connected:
            return None

        with self.lock:
            # Simulate reading data based on device type
            self.last_read_time = time.time()
            return self._simulate_data()

    def write_data(self, data: Any) -> bool:
        """Write data to the hardware device"""
        if not self.is_connected:
            return False

        with self.lock:
            # In a real implementation, this would send data to the device
            return True

    def calibrate(self) -> bool:
        """Calibrate the hardware device"""
        if not self.is_connected:
            return False

        with self.lock:
            # In a real implementation, this would perform calibration procedures
            self.is_calibrated = True
            return True

    def _simulate_data(self) -> Any:
        """Simulate data for testing purposes"""
        if self.config.device_type == 'camera':
            return {
                'width': self.config.parameters.get('width', 640),
                'height': self.config.parameters.get('height', 480),
                'timestamp': self.last_read_time
            }
        elif self.config.device_type == 'lidar':
            return {
                'ranges': [1.0] * 360,  # 360 range measurements
                'timestamp': self.last_read_time
            }
        elif self.config.device_type == 'imu':
            return {
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81],
                'timestamp': self.last_read_time
            }
        else:
            return {'timestamp': self.last_read_time}


class HardwareInterfaceNode(Node):
    """
    Hardware Interface Node for abstraction layer
    Provides standardized access to hardware devices across different platforms.
    """

    def __init__(self):
        super().__init__('hardware_interface_node')

        # QoS profile for internal communication
        self.internal_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Dictionary to store hardware devices
        self.devices: Dict[str, HardwareDevice] = {}
        self.device_configs: Dict[str, DeviceConfig] = {}

        # Initialize device configurations
        self._initialize_device_configs()

        # Connect to all configured devices
        self._connect_all_devices()

        # Publishers for device status
        self.device_status_pub = self.create_publisher(String, '/hardware_interface/device_status', self.internal_qos)

        # Parameter service
        self.get_parameters_client = self.create_client(GetParameters, '/hardware_interface/get_parameters')
        self.set_parameters_client = self.create_client(SetParameters, '/hardware_interface/set_parameters')

        # Timer for periodic device monitoring
        self.monitor_timer = self.create_timer(1.0, self._monitor_devices)

        # Status publisher timer
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info('Hardware Interface Node initialized with abstraction layer')

    def _initialize_device_configs(self):
        """Initialize configurations for all expected hardware devices"""
        # Camera configuration
        camera_config = DeviceConfig(
            device_path='/dev/video0',
            device_type='camera',
            parameters={
                'width': 640,
                'height': 480,
                'format': 'RGB8',
                'framerate': 30
            }
        )
        self.device_configs['camera'] = camera_config

        # LIDAR configuration
        lidar_config = DeviceConfig(
            device_path='/dev/ttyUSB0',
            device_type='lidar',
            parameters={
                'range_min': 0.1,
                'range_max': 30.0,
                'scan_frequency': 10.0
            }
        )
        self.device_configs['lidar'] = lidar_config

        # IMU configuration
        imu_config = DeviceConfig(
            device_path='/dev/i2c-1',
            device_type='imu',
            parameters={
                'rate': 100,
                'calibration_required': True
            }
        )
        self.device_configs['imu'] = imu_config

        # Temperature sensor configuration
        temp_config = DeviceConfig(
            device_path='/sys/bus/iio/devices/iio:device0',
            device_type='temperature',
            parameters={
                'rate': 1.0,
                'unit': 'celsius'
            }
        )
        self.device_configs['temperature'] = temp_config

        self.get_logger().info(f'Initialized {len(self.device_configs)} device configurations')

    def _connect_all_devices(self):
        """Connect to all configured devices"""
        for name, config in self.device_configs.items():
            if config.enabled:
                device = HardwareDevice(config)
                if device.connect():
                    self.devices[name] = device
                    self.get_logger().info(f'Successfully connected to device: {name}')
                else:
                    self.get_logger().error(f'Failed to connect to device: {name}')

    def _monitor_devices(self):
        """Monitor device status periodically"""
        for name, device in self.devices.items():
            if not device.is_connected:
                self.get_logger().warn(f'Device {name} is not connected, attempting to reconnect...')
                device.connect()

    def _publish_status(self):
        """Publish device status information"""
        status_msg = String()
        status_info = {
            'timestamp': time.time(),
            'devices': {}
        }

        for name, device in self.devices.items():
            status_info['devices'][name] = {
                'connected': device.is_connected,
                'calibrated': device.is_calibrated,
                'last_read': device.last_read_time
            }

        status_msg.data = str(status_info)
        self.device_status_pub.publish(status_msg)

    def read_device_data(self, device_name: str) -> Optional[Any]:
        """Read data from a specific device"""
        if device_name in self.devices:
            return self.devices[device_name].read_data()
        else:
            self.get_logger().error(f'Device {device_name} not found')
            return None

    def write_device_data(self, device_name: str, data: Any) -> bool:
        """Write data to a specific device"""
        if device_name in self.devices:
            return self.devices[device_name].write_data(data)
        else:
            self.get_logger().error(f'Device {device_name} not found')
            return False

    def calibrate_device(self, device_name: str) -> bool:
        """Calibrate a specific device"""
        if device_name in self.devices:
            return self.devices[device_name].calibrate()
        else:
            self.get_logger().error(f'Device {device_name} not found')
            return False

    def get_device_status(self, device_name: str) -> Optional[Dict[str, Any]]:
        """Get status of a specific device"""
        if device_name in self.devices:
            device = self.devices[device_name]
            return {
                'connected': device.is_connected,
                'calibrated': device.is_calibrated,
                'last_read_time': device.last_read_time
            }
        else:
            return None

    def list_devices(self) -> List[str]:
        """List all connected devices"""
        return list(self.devices.keys())

    def get_all_device_statuses(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all devices"""
        statuses = {}
        for name, device in self.devices.items():
            statuses[name] = {
                'connected': device.is_connected,
                'calibrated': device.is_calibrated,
                'last_read_time': device.last_read_time,
                'device_type': self.device_configs[name].device_type
            }
        return statuses

    def shutdown_all_devices(self):
        """Properly shutdown all connected devices"""
        for name, device in self.devices.items():
            device.disconnect()
            self.get_logger().info(f'Disconnected device: {name}')


def main(args=None):
    rclpy.init(args=args)
    hardware_interface_node = HardwareInterfaceNode()

    try:
        rclpy.spin(hardware_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        hardware_interface_node.shutdown_all_devices()
        hardware_interface_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()