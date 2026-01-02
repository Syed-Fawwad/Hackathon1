#!/usr/bin/env python3

"""
Planning Node for the ROS 2 Communication Framework
This node plans actions and paths based on current state and goals for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Path
from node_framework.communication_framework import CommunicationFrameworkNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class PlanningNode(CommunicationFrameworkNode):
    """
    Planning Node - plans actions and paths based on current state and goals
    """

    def __init__(self):
        super().__init__('planning_node')

        # Additional publishers for planning-specific topics
        self.plan_result_pub = self.create_publisher(String, '/plan_results', self.qos_profile)
        self.path_pub = self.create_publisher(Path, '/planned_path', self.qos_profile)
        self.velocity_cmd_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)

        # Subscriptions for goals and current state
        self.goal_sub = self.create_subscription(String, '/goal', self.goal_callback, self.qos_profile)
        self.current_pose_sub = self.create_subscription(
            String, '/robot_state', self.current_pose_callback, self.qos_profile)
        self.perception_result_sub = self.create_subscription(
            String, '/perception_results', self.perception_result_callback, self.qos_profile)

        # Timer for periodic planning
        self.planning_timer = self.create_timer(0.5, self.planning_callback)  # 2 Hz

        # Internal state
        self.current_goal = None
        self.current_pose = None
        self.perception_data = None
        self.planning_count = 0
        self.active_plan = None

        self.get_logger().info('Planning Node initialized with communication interfaces')

    def goal_callback(self, msg):
        """Callback for goal messages"""
        self.current_goal = msg.data
        self.get_logger().info(f'Received goal: {msg.data}')
        # Trigger immediate replanning when a new goal is received
        self.replan()

    def current_pose_callback(self, msg):
        """Callback for current pose/state messages"""
        self.current_pose = msg.data
        self.get_logger().debug(f'Current pose updated: {msg.data}')

    def perception_result_callback(self, msg):
        """Callback for perception results"""
        self.perception_data = msg.data
        self.get_logger().debug(f'Perception data updated: {msg.data}')

    def planning_callback(self):
        """Main planning processing callback - runs periodically"""
        self.get_logger().info(f'Planning actions (cycle {self.planning_count})...')

        # Generate or update plan based on current state and goal
        if self.current_goal:
            plan_result = self.generate_plan()

            if plan_result:
                # Publish plan result
                result_msg = String()
                result_msg.data = plan_result
                self.plan_result_pub.publish(result_msg)

                # Publish path if available
                path = self.create_path_from_plan(plan_result)
                if path:
                    self.path_pub.publish(path)

                # Execute plan by publishing velocity commands
                self.execute_plan(plan_result)

        self.planning_count += 1

    def replan(self):
        """Trigger immediate replanning when a new goal is received"""
        self.get_logger().info('Replanning triggered due to new goal')
        self.planning_callback()

    def generate_plan(self):
        """Generate a plan based on current state, goal, and perception data"""
        if not self.current_goal:
            return None

        plan = {
            'goal': self.current_goal,
            'current_pose': self.current_pose,
            'perception_data': self.perception_data,
            'plan_id': f'plan_{self.planning_count}',
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'actions': []
        }

        # Simulate planning logic
        if self.current_pose and 'obstacle' not in str(self.perception_data or ''):
            # If no obstacles detected, plan a direct path
            plan['actions'].append('move_forward')
            plan['actions'].append('turn_toward_goal')
            plan['actions'].append('approach_goal')
        else:
            # If obstacles detected, plan obstacle avoidance
            plan['actions'].append('stop_for_obstacle')
            plan['actions'].append('plan_around_obstacle')
            plan['actions'].append('resume_path')

        return f"Plan: {plan}"

    def create_path_from_plan(self, plan_result):
        """Create a Path message from the plan result"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Create some sample poses for the path
        # In a real implementation, this would come from the actual path planning algorithm
        for i in range(5):
            pose = Pose()
            pose.position.x = i * 0.5  # Move 0.5m at a time
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # No rotation
            path_msg.poses.append(pose)

        return path_msg

    def execute_plan(self, plan_result):
        """Execute the plan by publishing appropriate commands"""
        # For now, just publish a simple velocity command
        # In a real implementation, this would interpret the plan and generate appropriate commands
        if 'approach_goal' in plan_result:
            self.publish_velocity_command(linear_x=0.2)  # Move forward slowly
        elif 'stop_for_obstacle' in plan_result:
            self.publish_velocity_command(linear_x=0.0)  # Stop
        elif 'turn_toward_goal' in plan_result:
            self.publish_velocity_command(angular_z=0.3)  # Turn toward goal

    def publish_path(self, path):
        """Publish the planned path"""
        self.path_pub.publish(path)

    def publish_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                                angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Override parent method to also publish to local velocity_cmd_pub"""
        # Call parent method to publish to the framework's cmd_vel
        super().publish_velocity_command(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

        # Also publish to local publisher
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.linear.y = linear_y
        cmd_msg.linear.z = linear_z
        cmd_msg.angular.x = angular_x
        cmd_msg.angular.y = angular_y
        cmd_msg.angular.z = angular_z
        self.velocity_cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    planning_node = PlanningNode()

    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()