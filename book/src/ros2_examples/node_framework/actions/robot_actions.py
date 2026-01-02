#!/usr/bin/env python3

"""
Robot Actions for the ROS 2 Communication Framework
This module implements actions for move_to_pose and grasp_object operations.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from node_framework.communication_framework import CommunicationFrameworkNode
import time
import math


class RobotActionsNode(CommunicationFrameworkNode):
    """
    Node that provides robot actions: /move_to_pose and /grasp_object
    """

    def __init__(self):
        super().__init__('robot_actions_node')

        # Use a reentrant callback group to handle multiple actions concurrently
        callback_group = ReentrantCallbackGroup()

        # Create action servers
        self.move_to_pose_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            execute_callback=self.move_to_pose_callback,
            callback_group=callback_group,
            goal_callback=self.move_to_pose_goal_callback,
            cancel_callback=self.move_to_pose_cancel_callback
        )

        self.grasp_object_server = ActionServer(
            self,
            GraspObject,
            'grasp_object',
            execute_callback=self.grasp_object_callback,
            callback_group=callback_group,
            goal_callback=self.grasp_object_goal_callback,
            cancel_callback=self.grasp_object_cancel_callback
        )

        self.get_logger().info('Robot Actions Node initialized with /move_to_pose and /grasp_object actions')

    def destroy_node(self):
        """Clean up action servers when destroying the node"""
        self.move_to_pose_server.destroy()
        self.grasp_object_server.destroy()
        super().destroy_node()

    # Move to Pose Action Methods
    def move_to_pose_goal_callback(self, goal_request):
        """Accept or reject a goal to move to a pose."""
        self.get_logger().info('Received move_to_pose goal request')

        # Check if the requested pose is valid
        if self.is_valid_pose(goal_request.target_pose):
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn('Invalid pose requested')
            return GoalResponse.REJECT

    def move_to_pose_cancel_callback(self, goal_handle):
        """Accept or reject a cancel request for move_to_pose."""
        self.get_logger().info('Received request to cancel move_to_pose')
        return CancelResponse.ACCEPT

    def move_to_pose_callback(self, goal_handle):
        """Execute the move_to_pose action."""
        self.get_logger().info('Executing move_to_pose action...')

        # Get the target pose from the goal
        target_pose = goal_handle.request.target_pose
        self.get_logger().info(f'Moving to pose: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}')

        # Create feedback and result messages
        feedback_msg = MoveToPose.Feedback()
        result_msg = MoveToPose.Result()

        # Simulate the movement with feedback
        for i in range(0, 101, 10):  # Progress from 0 to 100%
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = 'Goal was canceled'
                self.get_logger().info('Move to pose was canceled')
                return result_msg

            # Simulate movement progress
            feedback_msg.current_pose = target_pose  # Simplified - in reality, this would be actual current pose
            feedback_msg.progress = float(i)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Move to pose progress: {i}%')

            # Sleep to simulate movement time
            time.sleep(0.2)

        # Check if the goal was canceled before completing
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result_msg.success = False
            result_msg.message = 'Goal was canceled'
            self.get_logger().info('Move to pose was canceled')
            return result_msg

        # Complete the goal successfully
        goal_handle.succeed()
        result_msg.success = True
        result_msg.message = 'Successfully moved to pose'
        self.get_logger().info('Move to pose completed successfully')
        return result_msg

    def is_valid_pose(self, pose):
        """Check if the given pose is valid."""
        # Check if position values are reasonable (not infinite or NaN)
        if (math.isfinite(pose.position.x) and
            math.isfinite(pose.position.y) and
            math.isfinite(pose.position.z)):
            return True
        return False

    # Grasp Object Action Methods
    def grasp_object_goal_callback(self, goal_request):
        """Accept or reject a goal to grasp an object."""
        self.get_logger().info('Received grasp_object goal request')

        # Check if the requested object is valid
        if goal_request.object_id and len(goal_request.object_id.strip()) > 0:
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn('Invalid object ID requested for grasping')
            return GoalResponse.REJECT

    def grasp_object_cancel_callback(self, goal_request):
        """Accept or reject a cancel request for grasp_object."""
        self.get_logger().info('Received request to cancel grasp_object')
        return CancelResponse.ACCEPT

    def grasp_object_callback(self, goal_handle):
        """Execute the grasp_object action."""
        self.get_logger().info('Executing grasp_object action...')

        # Get the object ID from the goal
        object_id = goal_handle.request.object_id
        self.get_logger().info(f'Attempting to grasp object: {object_id}')

        # Create feedback and result messages
        feedback_msg = GraspObject.Feedback()
        result_msg = GraspObject.Result()

        # Simulate the grasping process with feedback
        feedback_msg.status = 'Approaching object'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Approaching object...')
        time.sleep(1.0)

        # Check if the goal has been canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result_msg.success = False
            result_msg.message = 'Goal was canceled during approach'
            self.get_logger().info('Grasp object was canceled during approach')
            return result_msg

        feedback_msg.status = 'Aligning with object'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Aligning with object...')
        time.sleep(1.0)

        # Check if the goal has been canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result_msg.success = False
            result_msg.message = 'Goal was canceled during alignment'
            self.get_logger().info('Grasp object was canceled during alignment')
            return result_msg

        feedback_msg.status = 'Grasping object'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Grasping object...')
        time.sleep(1.0)

        # Check if the goal has been canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result_msg.success = False
            result_msg.message = 'Goal was canceled during grasp'
            self.get_logger().info('Grasp object was canceled during grasp')
            return result_msg

        # Check if the goal was canceled before completing
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result_msg.success = False
            result_msg.message = 'Goal was canceled'
            self.get_logger().info('Grasp object was canceled')
            return result_msg

        # Complete the goal successfully
        goal_handle.succeed()
        result_msg.success = True
        result_msg.message = f'Successfully grasped object {object_id}'
        self.get_logger().info(f'Grasp object completed successfully for {object_id}')
        return result_msg


# Define action interfaces since they're not in standard ROS 2 interfaces
# In a real implementation, these would be defined in .action files and generated
# For this example, we'll define simple action classes

class MoveToPose:
    class Goal:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.target_pose = Pose()

    class Feedback:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.current_pose = Pose()
            self.progress = 0.0

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""

    class SendGoalRequest:
        pass

    class SendGoalResponse:
        def __init__(self):
            self.accepted = False
            self.stamp = None


class GraspObject:
    class Goal:
        def __init__(self):
            self.object_id = ""

    class Feedback:
        def __init__(self):
            self.status = ""

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""

    class SendGoalRequest:
        pass

    class SendGoalResponse:
        def __init__(self):
            self.accepted = False
            self.stamp = None


def main(args=None):
    rclpy.init(args=args)
    robot_actions_node = RobotActionsNode()

    try:
        rclpy.spin(robot_actions_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_actions_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()