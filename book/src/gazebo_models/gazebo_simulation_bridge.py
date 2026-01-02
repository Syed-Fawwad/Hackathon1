#!/usr/bin/env python3

"""
Gazebo Simulation Bridge for Simulation Topics
This node handles Gazebo simulation topics including model states, link states, and joint commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from gazebo_msgs.msg import ModelStates, LinkStates
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class GazeboSimulationBridge(Node):
    """
    Gazebo Simulation Bridge Node
    Handles simulation topics: /gazebo/model_states, /gazebo/link_states, /joint_commands
    """

    def __init__(self):
        super().__init__('gazebo_simulation_bridge')

        # Initialize publishers for simulation topics
        qos_profile = QoSProfile(depth=10)

        # Publishers
        self.model_states_pub = self.create_publisher(ModelStates, '/gazebo/model_states', qos_profile)
        self.link_states_pub = self.create_publisher(LinkStates, '/gazebo/link_states', qos_profile)
        self.joint_commands_pub = self.create_publisher(JointState, '/joint_commands', qos_profile)

        # Subscriber for joint commands
        self.joint_commands_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_commands_callback,
            qos_profile
        )

        # Timer for publishing simulation states
        self.timer = self.create_timer(0.1, self.publish_simulation_states)  # 10 Hz

        # Initialize model states
        self.model_states = ModelStates()
        self.link_states = LinkStates()
        self.joint_commands = JointState()

        # Initialize with some default models
        self.model_states.name = ['humanoid_robot', 'ground_plane']
        self.model_states.pose = [self.create_default_pose()] * 2
        self.model_states.twist = [self.create_default_twist()] * 2

        # Initialize with some default links
        self.link_states.name = ['humanoid_robot::base_link', 'ground_plane::link']
        self.link_states.pose = [self.create_default_pose()] * 2
        self.link_states.twist = [self.create_default_twist()] * 2

        self.get_logger().info('Gazebo Simulation Bridge initialized')

    def create_default_pose(self):
        """Create a default pose"""
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        return pose

    def create_default_twist(self):
        """Create a default twist"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        return twist

    def publish_simulation_states(self):
        """Publish simulation states"""
        try:
            # Update header timestamps
            current_time = self.get_clock().now().to_msg()

            # Publish model states
            model_states_msg = ModelStates()
            model_states_msg.header.stamp = current_time
            model_states_msg.header.frame_id = 'world'
            model_states_msg.name = self.model_states.name.copy()
            model_states_msg.pose = self.model_states.pose.copy()
            model_states_msg.twist = self.model_states.twist.copy()
            self.model_states_pub.publish(model_states_msg)

            # Publish link states
            link_states_msg = LinkStates()
            link_states_msg.header.stamp = current_time
            link_states_msg.name = self.link_states.name.copy()
            link_states_msg.pose = self.link_states.pose.copy()
            link_states_msg.twist = self.link_states.twist.copy()
            self.link_states_pub.publish(link_states_msg)

            # Publish joint commands (echo for now)
            joint_cmd_msg = JointState()
            joint_cmd_msg.header.stamp = current_time
            joint_cmd_msg.header.frame_id = 'base_link'
            joint_cmd_msg.name = self.joint_commands.name.copy()
            joint_cmd_msg.position = self.joint_commands.position.copy()
            joint_cmd_msg.velocity = self.joint_commands.velocity.copy()
            joint_cmd_msg.effort = self.joint_commands.effort.copy()
            self.joint_commands_pub.publish(joint_cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing simulation states: {e}')

    def joint_commands_callback(self, msg):
        """Callback for joint commands"""
        try:
            self.joint_commands = msg
            self.get_logger().info(f'Received joint commands for {len(msg.name)} joints')
        except Exception as e:
            self.get_logger().error(f'Error processing joint commands: {e}')

    def update_model_state(self, model_name, pose, twist):
        """Update a specific model's state"""
        if model_name in self.model_states.name:
            index = self.model_states.name.index(model_name)
            self.model_states.pose[index] = pose
            self.model_states.twist[index] = twist
        else:
            # Add new model
            self.model_states.name.append(model_name)
            self.model_states.pose.append(pose)
            self.model_states.twist.append(twist)

    def update_link_state(self, link_name, pose, twist):
        """Update a specific link's state"""
        if link_name in self.link_states.name:
            index = self.link_states.name.index(link_name)
            self.link_states.pose[index] = pose
            self.link_states.twist[index] = twist
        else:
            # Add new link
            self.link_states.name.append(link_name)
            self.link_states.pose.append(pose)
            self.link_states.twist.append(twist)


def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboSimulationBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()