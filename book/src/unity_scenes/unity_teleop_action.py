#!/usr/bin/env python3

"""
Unity Teleoperation Action Server
This node implements the /unity_teleop_action for controlling Unity-based teleoperation.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import time
import threading
from rclpy.parameter import Parameter


class UnityTeleopAction(Node):
    """
    Unity Teleoperation Action Server
    Implements action: /unity_teleop_action
    """

    def __init__(self):
        super().__init__('unity_teleop_action')

        # Initialize action server
        self._action_server = ActionServer(
            self,
            # We'll use a placeholder since we need to generate the action type
            # In a real implementation, we'd use the generated action type
            None,  # This would be the generated action type
            'unity_teleop_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for sending commands to Unity
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.unity_command_pub = self.create_publisher(String, '/unity_commands', qos_profile)

        # Store active goals
        self.active_goals = {}

        self.get_logger().info('Unity Teleop Action Server initialized')

    def goal_callback(self, goal_request):
        """Handle goal request"""
        self.get_logger().info(f'Received teleop goal: {goal_request}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel request"""
        self.get_logger().info('Received cancel request for teleop action')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the teleoperation action"""
        try:
            # This would be implemented with a proper action definition
            # For now, we'll simulate the action execution
            self.get_logger().info('Executing unity_teleop_action')

            # Get goal parameters (in a real implementation, these would come from the goal)
            command = getattr(goal_handle.request, 'command', 'move')
            linear_velocity = getattr(goal_handle.request, 'linear_velocity', 0.5)
            angular_velocity = getattr(goal_handle.request, 'angular_velocity', 0.5)
            duration_ms = getattr(goal_handle.request, 'duration_ms', 1000)

            # Send command to Unity
            unity_cmd = String()
            unity_cmd.data = f'teleop:{command}:{linear_velocity}:{angular_velocity}:{duration_ms}'
            self.unity_command_pub.publish(unity_cmd)

            # Send velocity command
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)

            # Simulate action execution with feedback
            start_time = time.time()
            duration = duration_ms / 1000.0  # Convert to seconds

            while time.time() - start_time < duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    # Stop the robot
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
                    self.get_logger().info('Teleop action canceled')
                    return  # Return without setting result

                # Publish feedback
                feedback = None  # This would be a proper feedback message
                # In a real implementation: goal_handle.publish_feedback(feedback)

                time.sleep(0.1)  # 10 Hz feedback

            # Complete the action
            goal_handle.succeed()
            result = None  # This would be a proper result message
            # In a real implementation: result.success = True

            self.get_logger().info('Teleop action completed successfully')

            # Stop the robot after action completion
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            # Return result (in a real implementation)
            # return result

        except Exception as e:
            self.get_logger().error(f'Error executing teleop action: {e}')
            goal_handle.abort()
            result = None  # This would be a proper result message
            # In a real implementation: result.success = False
            # In a real implementation: result.message = str(e)
            # return result

    def send_direct_teleop_command(self, linear_vel, angular_vel, duration=1.0):
        """Send a direct teleop command without using the action interface"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        # Publish for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    action_server = UnityTeleopAction()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()