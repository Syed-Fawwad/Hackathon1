#!/usr/bin/env python3

"""
Visualization Node for Data Processing
This node processes data for visualization in Unity and other visualization tools.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState, Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray, Twist
from std_msgs.msg import String, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.parameter import Parameter
import numpy as np
import math
from std_msgs.msg import Header


class VisualizationNode(Node):
    """
    Visualization Node
    Processes data for visualization in Unity and other visualization tools.
    """

    def __init__(self):
        super().__init__('visualization_node')

        # Initialize publishers for visualization
        qos_profile = QoSProfile(depth=10)

        # Visualization markers publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', qos_profile)

        # Processed data publishers
        self.processed_joint_states_pub = self.create_publisher(
            JointState, '/processed_joint_states', qos_profile
        )
        self.robot_path_pub = self.create_publisher(
            PoseArray, '/robot_path', qos_profile
        )
        self.robot_trajectory_pub = self.create_publisher(
            Marker, '/robot_trajectory', qos_profile
        )

        # Subscribers for raw data
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, qos_profile
        )
        self.robot_pose_sub = self.create_subscription(
            Pose, '/robot_pose', self.robot_pose_callback, qos_profile
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile
        )
        self.user_commands_sub = self.create_subscription(
            String, '/user_commands', self.user_commands_callback, qos_profile
        )

        # Store robot path for visualization
        self.robot_path = []
        self.max_path_length = 100  # Maximum points to store in path

        # Timer for publishing visualization data
        self.vis_timer = self.create_timer(0.05, self.publish_visualization_data)  # 20 Hz

        self.get_logger().info('Visualization Node initialized')

    def joint_states_callback(self, msg):
        """Callback for joint states - process and republish if needed"""
        try:
            # Process joint states for visualization
            processed_msg = JointState()
            processed_msg.header = msg.header
            processed_msg.name = msg.name
            processed_msg.position = msg.position
            processed_msg.velocity = msg.velocity
            processed_msg.effort = msg.effort

            # Add any processing needed for visualization
            # For now, just republish with a slight modification for demonstration
            processed_msg.position = [pos * 1.0 for pos in msg.position]  # Identity transform

            self.processed_joint_states_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing joint states: {e}')

    def robot_pose_callback(self, msg):
        """Callback for robot pose - add to path for trajectory visualization"""
        try:
            # Add current pose to robot path
            self.robot_path.append(msg)
            if len(self.robot_path) > self.max_path_length:
                self.robot_path.pop(0)  # Remove oldest point

        except Exception as e:
            self.get_logger().error(f'Error processing robot pose: {e}')

    def cmd_vel_callback(self, msg):
        """Callback for velocity commands - can be used for visualization"""
        try:
            # Process velocity commands for visualization purposes
            self.get_logger().debug(f'Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                                  f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')
        except Exception as e:
            self.get_logger().error(f'Error processing cmd_vel: {e}')

    def user_commands_callback(self, msg):
        """Callback for user commands - can be visualized"""
        try:
            # Process user commands for visualization
            self.get_logger().info(f'Received user command for visualization: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing user commands: {e}')

    def publish_visualization_data(self):
        """Publish visualization data"""
        try:
            # Publish robot path as a marker
            if len(self.robot_path) > 1:
                self.publish_robot_path()

            # Publish other visualization markers
            self.publish_robot_model_markers()

        except Exception as e:
            self.get_logger().error(f'Error publishing visualization data: {e}')

    def publish_robot_path(self):
        """Publish robot path as a line marker"""
        try:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robot_path'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set the scale of the marker
            marker.scale.x = 0.02  # Line width

            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            # Set the points
            for pose in self.robot_path:
                marker.points.append(pose.position)

            self.robot_trajectory_pub.publish(marker)

        except Exception as e:
            self.get_logger().error(f'Error publishing robot path: {e}')

    def publish_robot_model_markers(self):
        """Publish markers for robot model visualization"""
        try:
            marker_array = MarkerArray()

            # Example: Publish a marker for the robot base
            base_marker = Marker()
            base_marker.header.frame_id = 'base_link'
            base_marker.header.stamp = self.get_clock().now().to_msg()
            base_marker.ns = 'robot_model'
            base_marker.id = 1
            base_marker.type = Marker.CYLINDER
            base_marker.action = Marker.ADD

            # Set the scale (0.3m diameter, 0.1m height)
            base_marker.scale.x = 0.3
            base_marker.scale.y = 0.3
            base_marker.scale.z = 0.1

            # Set the color (gray)
            base_marker.color.r = 0.5
            base_marker.color.g = 0.5
            base_marker.color.b = 0.5
            base_marker.color.a = 0.8

            # Set the position (at origin of base_link)
            base_marker.pose.position.x = 0.0
            base_marker.pose.position.y = 0.0
            base_marker.pose.position.z = 0.05  # Half height offset

            # Set orientation (no rotation)
            base_marker.pose.orientation.w = 1.0
            base_marker.pose.orientation.x = 0.0
            base_marker.pose.orientation.y = 0.0
            base_marker.pose.orientation.z = 0.0

            marker_array.markers.append(base_marker)

            # Publish the marker array
            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f'Error publishing robot model markers: {e}')

    def process_joint_angles_for_visualization(self, joint_positions):
        """Process joint angles for better visualization"""
        try:
            # Apply any filtering or processing to joint angles
            processed_positions = []
            for pos in joint_positions:
                # Example: Apply a simple smoothing or constraint
                processed_pos = max(min(pos, math.pi), -math.pi)  # Constrain to [-π, π]
                processed_positions.append(processed_pos)

            return processed_positions
        except Exception as e:
            self.get_logger().error(f'Error processing joint angles: {e}')
            return joint_positions

    def create_robot_mesh_marker(self, mesh_resource, position, orientation, scale=(1.0, 1.0, 1.0)):
        """Create a marker for a robot mesh"""
        try:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robot_mesh'
            marker.id = len(self.marker_pub)  # This won't work, but for illustration
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD

            marker.mesh_resource = mesh_resource
            marker.mesh_use_embedded_materials = True

            marker.pose.position = position
            marker.pose.orientation = orientation

            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]

            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 0.8

            return marker
        except Exception as e:
            self.get_logger().error(f'Error creating robot mesh marker: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    viz_node = VisualizationNode()

    try:
        rclpy.spin(viz_node)
    except KeyboardInterrupt:
        pass
    finally:
        viz_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()