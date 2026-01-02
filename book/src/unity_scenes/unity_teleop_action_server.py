#!/usr/bin/env python3

"""
Unity Teleoperation Action Server
This node implements the /unity_teleop_action for controlling Unity-based teleoperation.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from threading import Lock


class UnityTeleopActionServer(Node):
    """
    Unity Teleoperation Action Server
    Implements action: /unity_teleop_action
    NOTE: This implementation assumes the action has been generated from action/UnityTeleop.action
    In a real ROS 2 package, you would import the generated action type like:
    from unity_scenes_interfaces.action import UnityTeleop
    """

    def __init__(self):
        super().__init__('unity_teleop_action_server')

        # Callback group for reentrant callbacks
        callback_group = ReentrantCallbackGroup()

        # Initialize action server
        # NOTE: In a real implementation, you would import the action type:
        # from unity_scenes_interfaces.action import UnityTeleop
        # self._action_server = ActionServer(
        #     self,
        #     UnityTeleop,
        #     'unity_teleop_action',
        #     self.execute_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback,
        #     callback_group=callback_group
        # )

        # For this example, we'll simulate the action behavior with regular topics
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.unity_command_pub = self.create_publisher(String, '/unity_commands', qos_profile)

        # For demonstration purposes, we'll use a timer to simulate action execution
        self.action_timer = self.create_timer(0.1, self.action_timer_callback)
        self.active_goals = {}
        self.goals_lock = Lock()

        self.get_logger().info('Unity Teleop Action Server initialized (simulated)')

    def goal_callback(self, goal_request):
        """Handle goal request"""
        self.get_logger().info(f'Received teleop goal request')
        # In a real implementation, you would validate the goal here
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel request"""
        self.get_logger().info('Received cancel request for teleop action')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the teleoperation action"""
        # This is the actual action execution method
        # In a real implementation, this would contain the logic to execute the action
        self.get_logger().info('Starting teleop action execution')

        # Get the goal parameters
        goal = goal_handle.request
        command = goal.command
        linear_velocity = goal.linear_velocity
        angular_velocity = goal.angular_velocity
        duration_ms = goal.duration_ms

        self.get_logger().info(f'Executing teleop: cmd={command}, lin={linear_velocity}, ang={angular_velocity}, dur={duration_ms}ms')

        # Send command to Unity
        unity_cmd = String()
        unity_cmd.data = f'teleop:{command}:{linear_velocity}:{angular_velocity}:{duration_ms}'
        self.unity_command_pub.publish(unity_cmd)

        # Send velocity command to robot
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

        # Execute the action with feedback
        start_time = time.time()
        duration = duration_ms / 1000.0  # Convert to seconds

        while rclpy.ok():
            # Check if the action should be canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                # Stop the robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)

                result = None  # Would be UnityTeleop.Result()
                # result.success = False
                # result.message = 'Action canceled'
                self.get_logger().info('Teleop action canceled')
                return result  # Would return result

            # Calculate progress
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0) if duration > 0 else 1.0

            # Publish feedback
            feedback = None  # Would be UnityTeleop.Feedback()
            # feedback.status = f'Executing {command}'
            # feedback.progress = progress
            # goal_handle.publish_feedback(feedback)

            # Check if action is complete
            if elapsed >= duration:
                break

            # Sleep briefly to allow other callbacks to run
            time.sleep(0.05)

        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        # Complete the action
        goal_handle.succeed()
        result = None  # Would be UnityTeleop.Result()
        # result.success = True
        # result.message = f'Teleop action {command} completed successfully'

        self.get_logger().info('Teleop action completed successfully')
        # return result  # Would return result

    def action_timer_callback(self):
        """Timer callback to simulate action progress"""
        # This is just for demonstration since we can't fully implement
        # the action without the generated action types
        pass


def main(args=None):
    rclpy.init(args=args)
    action_server = UnityTeleopActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()