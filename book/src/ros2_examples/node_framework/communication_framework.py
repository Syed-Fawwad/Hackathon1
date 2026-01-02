#!/usr/bin/env python3

"""
ROS 2 Node Communication Framework
This provides a foundational communication framework for the humanoid robot,
enabling reliable inter-process communication between perception, planning, and control systems.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class CommunicationFrameworkNode(Node):
    """
    Base class for all communication framework nodes that handles common ROS 2 patterns
    """

    def __init__(self, node_name, qos_profile=None):
        super().__init__(node_name)

        # Set default QoS profile if not provided
        if qos_profile is None:
            self.qos_profile = QoSProfile(
                depth=10,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        else:
            self.qos_profile = qos_profile

        # Common topics for robot communication
        self.sensor_data_pub = self.create_publisher(String, '/sensor_data', self.qos_profile)
        self.robot_state_pub = self.create_publisher(String, '/robot_state', self.qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', self.qos_profile)

        # Subscriptions
        self.sensor_data_sub = self.create_subscription(
            String, '/sensor_data', self.sensor_data_callback, self.qos_profile)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.robot_state_callback, self.qos_profile)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, self.qos_profile)

        self.get_logger().info(f'Communication Framework Node {node_name} initialized')

    def sensor_data_callback(self, msg):
        """Callback for sensor data messages"""
        self.get_logger().info(f'Sensor data received: {msg.data}')

    def robot_state_callback(self, msg):
        """Callback for robot state messages"""
        self.get_logger().info(f'Robot state received: {msg.data}')

    def cmd_vel_callback(self, msg):
        """Callback for velocity command messages"""
        self.get_logger().info(f'Velocity command received: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                              f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

    def publish_sensor_data(self, data):
        """Publish sensor data to the /sensor_data topic"""
        msg = String()
        msg.data = str(data)
        self.sensor_data_pub.publish(msg)

    def publish_robot_state(self, state):
        """Publish robot state to the /robot_state topic"""
        msg = String()
        msg.data = str(state)
        self.robot_state_pub.publish(msg)

    def publish_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                                angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish velocity command to the /cmd_vel topic"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_joint_states(self, joint_names, positions, velocities=None, efforts=None):
        """Publish joint states to the /joint_states topic"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions
        if velocities is not None:
            msg.velocity = velocities
        if efforts is not None:
            msg.effort = efforts
        self.joint_states_pub.publish(msg)


class PerceptionNode(CommunicationFrameworkNode):
    """
    Perception Node - processes sensor data and creates perception results
    """

    def __init__(self):
        super().__init__('perception_node')

        # Additional publishers/subscribers for perception-specific topics
        self.perception_result_pub = self.create_publisher(String, '/perception_results', self.qos_profile)

        # Timer for periodic perception processing
        self.perception_timer = self.create_timer(0.1, self.perception_callback)  # 10 Hz

        self.get_logger().info('Perception Node initialized')

    def perception_callback(self):
        """Main perception processing callback"""
        # Simulate perception processing
        self.get_logger().info('Processing perception data...')

        # Publish perception results
        result_msg = String()
        result_msg.data = f'Perception result at {self.get_clock().now().seconds_nanoseconds()}'
        self.perception_result_pub.publish(result_msg)


class PlanningNode(CommunicationFrameworkNode):
    """
    Planning Node - plans actions and paths based on current state and goals
    """

    def __init__(self):
        super().__init__('planning_node')

        # Additional publishers/subscribers for planning-specific topics
        self.plan_result_pub = self.create_publisher(String, '/plan_results', self.qos_profile)
        self.goal_sub = self.create_subscription(String, '/goal', self.goal_callback, self.qos_profile)

        # Timer for periodic planning
        self.planning_timer = self.create_timer(0.5, self.planning_callback)  # 2 Hz

        self.get_logger().info('Planning Node initialized')

    def goal_callback(self, msg):
        """Callback for goal messages"""
        self.get_logger().info(f'Received goal: {msg.data}')

    def planning_callback(self):
        """Main planning processing callback"""
        # Simulate planning processing
        self.get_logger().info('Planning actions...')

        # Publish plan results
        result_msg = String()
        result_msg.data = f'Plan result at {self.get_clock().now().seconds_nanoseconds()}'
        self.plan_result_pub.publish(result_msg)


class ControlNode(CommunicationFrameworkNode):
    """
    Control Node - executes low-level control commands to actuate the robot
    """

    def __init__(self):
        super().__init__('control_node')

        # Additional publishers/subscribers for control-specific topics
        self.control_command_pub = self.create_publisher(String, '/control_commands', self.qos_profile)

        # Timer for periodic control updates
        self.control_timer = self.create_timer(0.033, self.control_callback)  # ~30 Hz

        self.get_logger().info('Control Node initialized')

    def control_callback(self):
        """Main control processing callback"""
        # Simulate control processing
        self.get_logger().info('Executing control commands...')


class SensorFusionNode(CommunicationFrameworkNode):
    """
    Sensor Fusion Node - combines data from multiple sensors for coherent understanding
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Additional publishers/subscribers for sensor fusion-specific topics
        self.fused_data_pub = self.create_publisher(String, '/fused_sensor_data', self.qos_profile)

        # Timer for periodic sensor fusion
        self.fusion_timer = self.create_timer(0.05, self.fusion_callback)  # 20 Hz

        self.get_logger().info('Sensor Fusion Node initialized')

    def fusion_callback(self):
        """Main sensor fusion processing callback"""
        # Simulate sensor fusion processing
        self.get_logger().info('Fusing sensor data...')


def main(args=None):
    rclpy.init(args=args)

    # Create all framework nodes
    perception_node = PerceptionNode()
    planning_node = PlanningNode()
    control_node = ControlNode()
    sensor_fusion_node = SensorFusionNode()

    # Create an executor and add all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(perception_node)
    executor.add_node(planning_node)
    executor.add_node(control_node)
    executor.add_node(sensor_fusion_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        perception_node.destroy_node()
        planning_node.destroy_node()
        control_node.destroy_node()
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()