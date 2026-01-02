#!/usr/bin/env python3

"""
Control Node for the ROS 2 Communication Framework
This node executes low-level control commands to actuate the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from node_framework.communication_framework import CommunicationFrameworkNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math


class ControlNode(CommunicationFrameworkNode):
    """
    Control Node - executes low-level control commands to actuate the robot
    """

    def __init__(self):
        super().__init__('control_node')

        # Additional publishers for control-specific topics
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', self.qos_profile)
        self.motor_command_pub = self.create_publisher(Twist, '/motor_commands', self.qos_profile)
        self.control_status_pub = self.create_publisher(String, '/control_status', self.qos_profile)

        # Subscriptions for control commands and state feedback
        self.velocity_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_cmd_callback, self.qos_profile)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, self.qos_profile)
        self.plan_result_sub = self.create_subscription(
            String, '/plan_results', self.plan_result_callback, self.qos_profile)

        # Timer for periodic control updates
        self.control_timer = self.create_timer(0.033, self.control_callback)  # ~30 Hz

        # Internal state
        self.desired_velocity = Twist()
        self.current_joint_states = JointState()
        self.control_count = 0
        self.control_enabled = True

        # Joint position tracking
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}

        self.get_logger().info('Control Node initialized with communication interfaces')

    def velocity_cmd_callback(self, msg):
        """Callback for velocity command messages"""
        self.desired_velocity = msg
        self.get_logger().info(f'Received velocity command: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                              f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.current_joint_states = msg
        # Update internal joint state tracking
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]

        self.get_logger().debug(f'Updated joint states for {len(msg.name)} joints')

    def plan_result_callback(self, msg):
        """Callback for plan result messages"""
        self.get_logger().debug(f'Received plan result for execution: {msg.data}')

    def control_callback(self):
        """Main control processing callback - runs at high frequency"""
        if not self.control_enabled:
            return

        self.get_logger().info(f'Executing control commands (cycle {self.control_count})...')

        # Execute control commands based on desired velocity
        if self.desired_velocity.linear.x != 0 or self.desired_velocity.angular.z != 0:
            self.execute_velocity_control()
        else:
            # If no velocity command, maintain current position
            self.maintain_position()

        # Update joint states and publish
        self.update_joint_states()
        self.publish_control_status()

        self.control_count += 1

    def execute_velocity_control(self):
        """Execute velocity-based control commands"""
        # In a real implementation, this would interface with the robot's motor controllers
        # For simulation, we'll just publish the desired commands
        self.motor_command_pub.publish(self.desired_velocity)

        # Update joint positions based on velocity commands (simplified simulation)
        self.simulate_joint_motion()

        self.get_logger().debug('Executed velocity control')

    def maintain_position(self):
        """Maintain current position when no velocity command is active"""
        # In a real implementation, this would hold current position
        # For simulation, just maintain current state
        hold_cmd = Twist()
        self.motor_command_pub.publish(hold_cmd)

        self.get_logger().debug('Maintaining current position')

    def simulate_joint_motion(self):
        """Simulate joint motion based on velocity commands"""
        # This is a simplified simulation of how velocity commands might affect joint positions
        # In a real robot, this would be handled by low-level controllers

        # Example: update some joint positions based on velocity commands
        for joint_name in ['left_wheel_joint', 'right_wheel_joint', 'head_pan_joint', 'head_tilt_joint']:
            if joint_name not in self.joint_positions:
                self.joint_positions[joint_name] = 0.0

        # Update wheel joints based on linear and angular velocity
        linear_vel = self.desired_velocity.linear.x
        angular_vel = self.desired_velocity.angular.z

        # Simple differential drive model
        left_wheel_vel = linear_vel - angular_vel * 0.5  # 0.5 is wheelbase/2
        right_wheel_vel = linear_vel + angular_vel * 0.5

        # Update joint positions (integrate velocity)
        dt = 0.033  # Control loop time step (~30Hz)
        self.joint_positions['left_wheel_joint'] += left_wheel_vel * dt
        self.joint_positions['right_wheel_joint'] += right_wheel_vel * dt

        # Update head joints based on angular velocity
        if angular_vel != 0:
            self.joint_positions['head_pan_joint'] += angular_vel * dt * 0.1  # Scale factor

    def update_joint_states(self):
        """Update and publish joint states"""
        # Create joint state message
        joint_state_msg = JointState()
        joint_state_msg.header = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())
        joint_state_msg.velocity = [self.joint_velocities.get(name, 0.0) for name in joint_state_msg.name]
        joint_state_msg.effort = [self.joint_efforts.get(name, 0.0) for name in joint_state_msg.name]

        # Publish updated joint states
        self.publish_joint_states(
            joint_state_msg.name,
            joint_state_msg.position,
            joint_state_msg.velocity,
            joint_state_msg.effort
        )

    def publish_control_status(self):
        """Publish control status"""
        status_msg = String()
        status_msg.data = f"Control active, cycle {self.control_count}, joints: {len(self.joint_positions)}"
        self.control_status_pub.publish(status_msg)

    def publish_joint_commands(self, joint_names, positions):
        """Publish joint position commands"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = positions
        self.joint_command_pub.publish(cmd_msg)

    def enable_control(self):
        """Enable the control system"""
        self.control_enabled = True
        self.get_logger().info('Control system enabled')

    def disable_control(self):
        """Disable the control system"""
        self.control_enabled = False
        self.get_logger().info('Control system disabled')

    def emergency_stop(self):
        """Emergency stop - immediately halt all motion"""
        self.desired_velocity = Twist()  # Zero all velocities
        self.maintain_position()
        self.get_logger().warn('Emergency stop executed')


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()

    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()