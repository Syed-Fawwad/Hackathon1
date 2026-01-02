#!/usr/bin/env python3

"""
Joint State Publisher for Joint State Management
This node manages and publishes joint states for the robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
import math
from std_msgs.msg import Header
import time


class JointStatePublisher(Node):
    """
    Joint State Publisher Node
    Manages and publishes joint states for the robot based on joint parameters and control inputs.
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Initialize joint state publisher
        qos_profile = QoSProfile(depth=10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Declare parameters for joint positions
        self.declare_parameter('publish_frequency', 50)  # Hz
        self.declare_parameter('use_default_joints', True)

        # Default joint names for our humanoid robot
        default_joints = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint',
            'right_hip_joint', 'right_knee_joint'
        ]

        for joint_name in default_joints:
            self.declare_parameter(f'{joint_name}_position', 0.0)
            self.declare_parameter(f'{joint_name}_velocity', 0.0)
            self.declare_parameter(f'{joint_name}_effort', 0.0)

        # Timer for publishing joint states
        publish_freq = self.get_parameter('publish_frequency').value
        self.timer = self.create_timer(1.0 / publish_freq, self.publish_joint_states)

        # Initialize joint state message
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = default_joints
        self.joint_state_msg.position = [0.0] * len(default_joints)
        self.joint_state_msg.velocity = [0.0] * len(default_joints)
        self.joint_state_msg.effort = [0.0] * len(default_joints)

        self.get_logger().info(f'Joint State Publisher initialized with {len(default_joints)} joints')

    def publish_joint_states(self):
        """Publish the current joint states"""
        try:
            # Update header
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_msg.header.frame_id = 'base_link'

            # Update joint positions from parameters
            for i, joint_name in enumerate(self.joint_state_msg.name):
                try:
                    pos_param = self.get_parameter(f'{joint_name}_position').value
                    vel_param = self.get_parameter(f'{joint_name}_velocity').value
                    eff_param = self.get_parameter(f'{joint_name}_effort').value

                    self.joint_state_msg.position[i] = pos_param
                    self.joint_state_msg.velocity[i] = vel_param
                    self.joint_state_msg.effort[i] = eff_param
                except:
                    # If parameter doesn't exist, keep the current value
                    continue

            # Publish the joint state message
            self.joint_state_pub.publish(self.joint_state_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {e}')

    def update_joint_positions(self, joint_positions):
        """
        Update joint positions programmatically
        joint_positions: dict with joint names as keys and positions as values
        """
        for joint_name, position in joint_positions.items():
            if joint_name in self.joint_state_msg.name:
                index = self.joint_state_msg.name.index(joint_name)
                self.joint_state_msg.position[index] = position

    def get_joint_positions(self):
        """Get current joint positions"""
        positions = {}
        for i, joint_name in enumerate(self.joint_state_msg.name):
            positions[joint_name] = self.joint_state_msg.position[i]
        return positions


def main(args=None):
    rclpy.init(args=args)
    publisher = JointStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()