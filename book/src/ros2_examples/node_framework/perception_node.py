#!/usr/bin/env python3

"""
Perception Node for the ROS 2 Communication Framework
This node processes sensor data and creates perception results for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import Point
from node_framework.communication_framework import CommunicationFrameworkNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class PerceptionNode(CommunicationFrameworkNode):
    """
    Perception Node - processes sensor data and creates perception results
    """

    def __init__(self):
        super().__init__('perception_node')

        # QoS profile for sensor data (may need different settings for different sensor types)
        sensor_qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Additional publishers for perception-specific topics
        self.perception_result_pub = self.create_publisher(String, '/perception_results', self.qos_profile)
        self.object_detection_pub = self.create_publisher(String, '/object_detections', self.qos_profile)
        self.pose_estimation_pub = self.create_publisher(String, '/pose_estimates', self.qos_profile)

        # Subscriptions for various sensor data types
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/pointcloud', self.pointcloud_callback, sensor_qos)
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, sensor_qos)

        # Timer for periodic perception processing
        self.perception_timer = self.create_timer(0.1, self.perception_callback)  # 10 Hz

        # Internal state
        self.latest_image = None
        self.latest_pointcloud = None
        self.latest_laser_scan = None
        self.perception_count = 0

        self.get_logger().info('Perception Node initialized with communication interfaces')

    def image_callback(self, msg):
        """Callback for image messages"""
        self.latest_image = msg
        self.get_logger().debug(f'Received image with dimensions: {msg.width}x{msg.height}')

    def pointcloud_callback(self, msg):
        """Callback for point cloud messages"""
        self.latest_pointcloud = msg
        self.get_logger().debug(f'Received point cloud with {msg.height * msg.width} points')

    def laser_scan_callback(self, msg):
        """Callback for laser scan messages"""
        self.latest_laser_scan = msg
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} range measurements')

    def perception_callback(self):
        """Main perception processing callback"""
        self.get_logger().info(f'Processing perception data (cycle {self.perception_count})...')

        # Simulate perception processing using available sensor data
        perception_results = self.process_sensor_data()

        # Publish perception results
        if perception_results:
            result_msg = String()
            result_msg.data = perception_results
            self.perception_result_pub.publish(result_msg)

            # Also publish to general sensor data topic
            self.publish_sensor_data(perception_results)

        self.perception_count += 1

    def process_sensor_data(self):
        """Process available sensor data and return perception results"""
        results = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'processed_sensors': [],
            'objects_detected': 0,
            'environment_analysis': 'normal'
        }

        # Process image data if available
        if self.latest_image is not None:
            results['processed_sensors'].append('camera')
            # Simulate object detection
            results['objects_detected'] += 2  # Simulated detection

        # Process point cloud data if available
        if self.latest_pointcloud is not None:
            results['processed_sensors'].append('lidar')
            # Simulate environment analysis
            results['environment_analysis'] = '3D mapping in progress'

        # Process laser scan data if available
        if self.latest_laser_scan is not None:
            results['processed_sensors'].append('laser')
            # Simulate obstacle detection
            min_range = min(self.latest_laser_scan.ranges) if self.latest_laser_scan.ranges else float('inf')
            if min_range < 1.0:  # Less than 1 meter
                results['environment_analysis'] = 'obstacle_detected'

        return f"Perception Result: {results}"

    def publish_object_detections(self, objects):
        """Publish object detection results"""
        detection_msg = String()
        detection_msg.data = f"Objects detected: {objects}"
        self.object_detection_pub.publish(detection_msg)

    def publish_pose_estimates(self, poses):
        """Publish pose estimation results"""
        pose_msg = String()
        pose_msg.data = f"Pose estimates: {poses}"
        self.pose_estimation_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()