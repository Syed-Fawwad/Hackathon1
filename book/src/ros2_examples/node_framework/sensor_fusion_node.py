#!/usr/bin/env python3

"""
Sensor Fusion Node for the ROS 2 Communication Framework
This node combines data from multiple sensors for coherent understanding of the humanoid robot's environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Point, Pose, Vector3
from nav_msgs.msg import Odometry
from node_framework.communication_framework import CommunicationFrameworkNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
from collections import deque


class SensorFusionNode(CommunicationFrameworkNode):
    """
    Sensor Fusion Node - combines data from multiple sensors for coherent understanding
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # QoS profile for sensor data (may need different settings for different sensor types)
        sensor_qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Additional publishers for sensor fusion-specific topics
        self.fused_data_pub = self.create_publisher(String, '/fused_sensor_data', self.qos_profile)
        self.estimated_state_pub = self.create_publisher(Odometry, '/estimated_robot_state', self.qos_profile)
        self.environment_map_pub = self.create_publisher(String, '/environment_map', self.qos_profile)

        # Subscriptions for various sensor data types
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, sensor_qos)
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos)
        self.perception_result_sub = self.create_subscription(
            String, '/perception_results', self.perception_result_callback, self.qos_profile)

        # Timer for periodic sensor fusion
        self.fusion_timer = self.create_timer(0.05, self.fusion_callback)  # 20 Hz

        # Internal state and data buffers
        self.imu_data = None
        self.joint_data = None
        self.laser_data = None
        self.odom_data = None
        self.perception_data = None

        self.imu_buffer = deque(maxlen=10)  # Keep last 10 IMU readings
        self.odom_buffer = deque(maxlen=10)  # Keep last 10 odometry readings
        self.joint_buffer = deque(maxlen=10)  # Keep last 10 joint state readings

        self.fusion_count = 0
        self.robot_pose_estimate = Pose()
        self.robot_velocity_estimate = Vector3()

        self.get_logger().info('Sensor Fusion Node initialized with communication interfaces')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.imu_data = msg
        self.imu_buffer.append({
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'orientation': msg.orientation,
            'angular_velocity': msg.angular_velocity,
            'linear_acceleration': msg.linear_acceleration
        })
        self.get_logger().debug(f'Received IMU data')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.joint_data = msg
        self.joint_buffer.append({
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'position': msg.position,
            'velocity': msg.velocity,
            'effort': msg.effort
        })
        self.get_logger().debug(f'Received joint state data for {len(msg.name)} joints')

    def laser_scan_callback(self, msg):
        """Callback for laser scan messages"""
        self.laser_data = msg
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} range measurements')

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.odom_data = msg
        self.odom_buffer.append({
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'pose': msg.pose.pose,
            'twist': msg.twist.twist
        })
        self.get_logger().debug(f'Received odometry data')

    def perception_result_callback(self, msg):
        """Callback for perception result messages"""
        self.perception_data = msg.data
        self.get_logger().debug(f'Received perception result for fusion: {msg.data}')

    def fusion_callback(self):
        """Main sensor fusion processing callback"""
        self.get_logger().info(f'Fusing sensor data (cycle {self.fusion_count})...')

        # Perform sensor fusion to create a coherent understanding
        fused_result = self.perform_sensor_fusion()

        if fused_result:
            # Publish fused sensor data
            fused_msg = String()
            fused_msg.data = fused_result
            self.fused_data_pub.publish(fused_msg)

            # Also publish to general sensor data topic
            self.publish_sensor_data(fused_result)

            # Publish estimated robot state
            estimated_state = self.create_estimated_state(fused_result)
            if estimated_state:
                self.estimated_state_pub.publish(estimated_state)

            # Publish environment map if available
            env_map = self.create_environment_map(fused_result)
            if env_map:
                self.environment_map_pub.publish(env_map)

        self.fusion_count += 1

    def perform_sensor_fusion(self):
        """Perform sensor fusion using data from multiple sensors"""
        fusion_result = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'sources_used': [],
            'robot_state': {},
            'environment_state': {},
            'confidence': 0.0
        }

        # Fuse IMU data for orientation and acceleration
        if self.imu_data:
            fusion_result['sources_used'].append('imu')
            fusion_result['robot_state']['orientation'] = {
                'x': self.imu_data.orientation.x,
                'y': self.imu_data.orientation.y,
                'z': self.imu_data.orientation.z,
                'w': self.imu_data.orientation.w
            }
            fusion_result['robot_state']['angular_velocity'] = {
                'x': self.imu_data.angular_velocity.x,
                'y': self.imu_data.angular_velocity.y,
                'z': self.imu_data.angular_velocity.z
            }

        # Fuse joint state data for position and velocity
        if self.joint_data and len(self.joint_data.position) > 0:
            fusion_result['sources_used'].append('joint_state')
            fusion_result['robot_state']['joint_positions'] = list(self.joint_data.position)
            fusion_result['robot_state']['joint_velocities'] = list(self.joint_data.velocity)

        # Fuse odometry data for position and velocity
        if self.odom_data:
            fusion_result['sources_used'].append('odometry')
            fusion_result['robot_state']['position'] = {
                'x': self.odom_data.pose.pose.position.x,
                'y': self.odom_data.pose.pose.position.y,
                'z': self.odom_data.pose.pose.position.z
            }
            fusion_result['robot_state']['linear_velocity'] = {
                'x': self.odom_data.twist.twist.linear.x,
                'y': self.odom_data.twist.twist.linear.y,
                'z': self.odom_data.twist.twist.linear.z
            }

        # Fuse laser scan data for environment mapping
        if self.laser_data:
            fusion_result['sources_used'].append('laser_scan')
            fusion_result['environment_state']['obstacle_distances'] = [
                r for r in self.laser_data.ranges if 0.1 < r < 10.0  # Filter valid range readings
            ]
            fusion_result['environment_state']['obstacle_count'] = len([
                r for r in self.laser_data.ranges if 0.1 < r < 1.0  # Obstacles within 1m
            ])

        # Fuse perception data for object detection
        if self.perception_data:
            fusion_result['sources_used'].append('perception')
            fusion_result['environment_state']['detected_objects'] = self.extract_objects(self.perception_data)

        # Calculate confidence based on number of sources
        fusion_result['confidence'] = min(len(fusion_result['sources_used']) / 5.0, 1.0)

        return f"Fused Sensor Data: {fusion_result}"

    def create_estimated_state(self, fused_result):
        """Create an estimated robot state from fused sensor data"""
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # In a real implementation, this would use proper sensor fusion algorithms (Kalman filters, etc.)
        # For this example, we'll simulate a simple fusion result
        try:
            # Extract position from odometry if available
            if 'position' in fused_result and isinstance(fused_result, dict):
                pos = fused_result.get('robot_state', {}).get('position', {})
                odom_msg.pose.pose.position.x = pos.get('x', 0.0)
                odom_msg.pose.pose.position.y = pos.get('y', 0.0)
                odom_msg.pose.pose.position.z = pos.get('z', 0.0)

            # Extract orientation from IMU if available
            orientation = fused_result.get('robot_state', {}).get('orientation', {})
            odom_msg.pose.pose.orientation.x = orientation.get('x', 0.0)
            odom_msg.pose.pose.orientation.y = orientation.get('y', 0.0)
            odom_msg.pose.pose.orientation.z = orientation.get('z', 0.0)
            odom_msg.pose.pose.orientation.w = orientation.get('w', 1.0)

            # Extract linear velocity
            linear_vel = fused_result.get('robot_state', {}).get('linear_velocity', {})
            odom_msg.twist.twist.linear.x = linear_vel.get('x', 0.0)
            odom_msg.twist.twist.linear.y = linear_vel.get('y', 0.0)
            odom_msg.twist.twist.linear.z = linear_vel.get('z', 0.0)

            return odom_msg
        except:
            # If parsing fails, return a default odometry message
            return odom_msg

    def create_environment_map(self, fused_result):
        """Create an environment map from fused sensor data"""
        map_msg = String()
        try:
            env_state = fused_result.get('environment_state', {})
            obstacle_count = env_state.get('obstacle_count', 0)
            obstacle_distances = env_state.get('obstacle_distances', [])

            map_data = {
                'obstacle_count': obstacle_count,
                'closest_obstacle': min(obstacle_distances) if obstacle_distances else float('inf'),
                'timestamp': fused_result.get('timestamp')
            }

            map_msg.data = f"Environment Map: {map_data}"
            return map_msg
        except:
            map_msg.data = f"Environment Map: {{'obstacle_count': 0, 'timestamp': {fused_result.get('timestamp')}}}"
            return map_msg

    def extract_objects(self, perception_data):
        """Extract object information from perception data"""
        # In a real implementation, this would parse actual object detection results
        # For this example, we'll simulate extracting object information
        if 'object' in str(perception_data).lower():
            return ['object1', 'object2']
        return []

    def publish_fused_data(self, data):
        """Publish fused sensor data"""
        fused_msg = String()
        fused_msg.data = str(data)
        self.fused_data_pub.publish(fused_msg)

    def publish_environment_map(self, env_map):
        """Publish environment map"""
        map_msg = String()
        map_msg.data = str(env_map)
        self.environment_map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()