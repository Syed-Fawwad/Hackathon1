#!/usr/bin/env python3

"""
Sensor Driver Node with Hardware Abstraction
This node implements sensor drivers with proper hardware abstraction for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu, JointState, Temperature, BatteryState, MagneticField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
import time
import threading
import numpy as np
from typing import Dict, Any, Optional


class SensorHardwareInterface:
    """
    Hardware abstraction interface for sensor devices
    """

    def __init__(self, sensor_type: str, device_path: str, config: Dict[str, Any]):
        self.sensor_type = sensor_type
        self.device_path = device_path
        self.config = config
        self.is_initialized = False
        self.is_connected = False

    def initialize(self) -> bool:
        """Initialize the hardware sensor"""
        # In a real implementation, this would open the device and configure it
        self.is_initialized = True
        self.is_connected = True
        return True

    def read_sensor_data(self) -> Optional[Any]:
        """Read raw data from the sensor hardware"""
        if not self.is_connected:
            return None

        # Simulate sensor data based on sensor type
        if self.sensor_type == 'camera':
            return self._simulate_camera_data()
        elif self.sensor_type == 'lidar':
            return self._simulate_lidar_data()
        elif self.sensor_type == 'imu':
            return self._simulate_imu_data()
        elif self.sensor_type == 'temperature':
            return self._simulate_temperature_data()
        else:
            return None

    def shutdown(self):
        """Shutdown the hardware sensor"""
        self.is_connected = False
        self.is_initialized = False

    def _simulate_camera_data(self):
        """Simulate camera data"""
        width = self.config.get('width', 640)
        height = self.config.get('height', 480)
        return {
            'width': width,
            'height': height,
            'data': np.random.randint(0, 255, (height, width, 3), dtype=np.uint8),
            'timestamp': time.time()
        }

    def _simulate_lidar_data(self):
        """Simulate LIDAR data"""
        angle_min = self.config.get('angle_min', -np.pi)
        angle_max = self.config.get('angle_max', np.pi)
        angle_increment = self.config.get('angle_increment', 0.01)

        num_points = int((angle_max - angle_min) / angle_increment)
        ranges = [2.0 + 0.1 * np.sin(i * 0.1) for i in range(num_points)]

        return {
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'ranges': ranges,
            'timestamp': time.time()
        }

    def _simulate_imu_data(self):
        """Simulate IMU data"""
        return {
            'orientation': [0.0, 0.0, 0.0, 1.0],  # x, y, z, w
            'angular_velocity': [0.01, 0.01, 0.01],
            'linear_acceleration': [0.1, 0.1, 9.81],
            'timestamp': time.time()
        }

    def _simulate_temperature_data(self):
        """Simulate temperature data"""
        return {
            'temperature': 25.0 + 0.5 * np.sin(time.time() * 0.1),
            'timestamp': time.time()
        }


class SensorDriverNode(Node):
    """
    Sensor Driver Node with Hardware Abstraction
    """

    def __init__(self):
        super().__init__('sensor_driver_node')

        # QoS profile for sensor data
        self.sensor_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Initialize sensor hardware interfaces
        self.sensors = {}
        self._initialize_sensors()

        # Publishers for different sensor types
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', self.sensor_qos)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', self.sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', self.sensor_qos)
        self.temp_pub = self.create_publisher(Temperature, '/temperature', self.sensor_qos)

        # TF publishers
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 100)

        # Timer for sensor data acquisition
        self.sensor_timer = self.create_timer(0.1, self._sensor_callback)  # 10 Hz

        # Parameter declarations
        self.declare_parameter('sensor_update_rate', 10.0)
        self.declare_parameter('publish_tf', True)

        self.get_logger().info('Sensor Driver Node with hardware abstraction initialized')

    def _initialize_sensors(self):
        """Initialize all configured sensors with hardware abstraction"""
        # Camera sensor
        camera_config = {
            'width': 640,
            'height': 480,
            'format': 'rgb8'
        }
        self.sensors['camera'] = SensorHardwareInterface('camera', '/dev/camera0', camera_config)

        # LIDAR sensor
        lidar_config = {
            'angle_min': -np.pi,
            'angle_max': np.pi,
            'angle_increment': 0.01,
            'range_min': 0.1,
            'range_max': 30.0
        }
        self.sensors['lidar'] = SensorHardwareInterface('lidar', '/dev/lidar0', lidar_config)

        # IMU sensor
        imu_config = {
            'rate': 100
        }
        self.sensors['imu'] = SensorHardwareInterface('imu', '/dev/imu0', imu_config)

        # Temperature sensor
        temp_config = {
            'rate': 1
        }
        self.sensors['temperature'] = SensorHardwareInterface('temperature', '/dev/temp0', temp_config)

        # Initialize all sensors
        for name, sensor in self.sensors.items():
            if sensor.initialize():
                self.get_logger().info(f'Sensor {name} initialized successfully')
            else:
                self.get_logger().error(f'Failed to initialize sensor {name}')

    def _sensor_callback(self):
        """Main sensor callback - reads from all sensors and publishes data"""
        for name, sensor in self.sensors.items():
            try:
                raw_data = sensor.read_sensor_data()
                if raw_data is not None:
                    self._process_and_publish_sensor_data(name, raw_data)
            except Exception as e:
                self.get_logger().error(f'Error reading sensor {name}: {e}')

    def _process_and_publish_sensor_data(self, sensor_type: str, raw_data: Any):
        """Process raw sensor data and publish appropriate ROS message"""
        if sensor_type == 'camera':
            self._publish_camera_data(raw_data)
        elif sensor_type == 'lidar':
            self._publish_lidar_data(raw_data)
        elif sensor_type == 'imu':
            self._publish_imu_data(raw_data)
        elif sensor_type == 'temperature':
            self._publish_temperature_data(raw_data)

    def _publish_camera_data(self, data):
        """Publish camera image data"""
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        img_msg.height = data['height']
        img_msg.width = data['width']
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = data['width'] * 3  # width * bytes per pixel

        # Convert numpy array to bytes
        img_msg.data = data['data'].tobytes()

        self.camera_pub.publish(img_msg)
        self.get_logger().debug(f'Published camera image: {img_msg.width}x{img_msg.height}')

    def _publish_lidar_data(self, data):
        """Publish LIDAR scan data"""
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_frame'
        scan_msg.angle_min = data['angle_min']
        scan_msg.angle_max = data['angle_max']
        scan_msg.angle_increment = data['angle_increment']
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0
        scan_msg.ranges = data['ranges']

        self.lidar_pub.publish(scan_msg)
        self.get_logger().debug(f'Published LIDAR scan with {len(scan_msg.ranges)} points')

    def _publish_imu_data(self, data):
        """Publish IMU data"""
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'

        # Set orientation (x, y, z, w)
        imu_msg.orientation.x = data['orientation'][0]
        imu_msg.orientation.y = data['orientation'][1]
        imu_msg.orientation.z = data['orientation'][2]
        imu_msg.orientation.w = data['orientation'][3]

        # Set angular velocity
        imu_msg.angular_velocity.x = data['angular_velocity'][0]
        imu_msg.angular_velocity.y = data['angular_velocity'][1]
        imu_msg.angular_velocity.z = data['angular_velocity'][2]

        # Set linear acceleration
        imu_msg.linear_acceleration.x = data['linear_acceleration'][0]
        imu_msg.linear_acceleration.y = data['linear_acceleration'][1]
        imu_msg.linear_acceleration.z = data['linear_acceleration'][2]

        self.imu_pub.publish(imu_msg)
        self.get_logger().debug(f'Published IMU data')

    def _publish_temperature_data(self, data):
        """Publish temperature data"""
        temp_msg = Temperature()
        temp_msg.header = Header()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = 'temperature_frame'
        temp_msg.temperature = data['temperature']
        temp_msg.variance = 0.0  # No variance in simulation

        self.temp_pub.publish(temp_msg)
        self.get_logger().debug(f'Published temperature: {temp_msg.temperature}°C')

    def publish_transforms(self):
        """Publish transform data if enabled"""
        if not self.get_parameter('publish_tf').value:
            return

        # Create a simple transform from base_link to sensor frames
        transforms = []

        # Camera transform
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_frame'
        camera_tf.transform.translation.x = 0.1
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.8
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 1.0
        transforms.append(camera_tf)

        # LIDAR transform
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_frame'
        lidar_tf.transform.translation.x = 0.0
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.5
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0
        transforms.append(lidar_tf)

        # Publish transforms
        tf_msg = TFMessage()
        tf_msg.transforms = transforms
        self.tf_pub.publish(tf_msg)

    def shutdown_sensors(self):
        """Properly shutdown all sensors"""
        for name, sensor in self.sensors.items():
            sensor.shutdown()
            self.get_logger().info(f'Sensor {name} shut down')


def main(args=None):
    rclpy.init(args=args)
    sensor_driver_node = SensorDriverNode()

    try:
        rclpy.spin(sensor_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_driver_node.shutdown_sensors()
        sensor_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()