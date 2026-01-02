#!/usr/bin/env python3

"""
Action Server Node for High-Level Behaviors
This node implements action servers for high-level robot behaviors in the humanoid system.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from typing import Dict, Any, Optional, Callable, List
import uuid
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from action_server_framework import BehaviorManager, BaseActionServer


class HighLevelBehaviorActionServer(Node):
    """
    Action Server Node for High-Level Behaviors
    Manages complex robot behaviors through action-based interfaces.
    """

    def __init__(self):
        super().__init__('action_server_node')

        # QoS profile for action-related messages
        self.action_qos = QoSProfile(depth=10)

        # Initialize behavior manager
        self.behavior_manager = BehaviorManager(self)

        # Use a reentrant callback group to handle multiple actions concurrently
        callback_group = ReentrantCallbackGroup()

        # Create action servers for high-level behaviors
        self.navigate_to_pose_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.navigate_to_pose_callback,
            callback_group=callback_group,
            goal_callback=self.navigate_to_pose_goal_callback,
            cancel_callback=self.navigate_to_pose_cancel_callback
        )

        self.pick_and_place_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            execute_callback=self.pick_and_place_callback,
            callback_group=callback_group,
            goal_callback=self.pick_and_place_goal_callback,
            cancel_callback=self.pick_and_place_cancel_callback
        )

        self.speech_recognition_server = ActionServer(
            self,
            SpeechRecognition,
            'speech_recognition',
            execute_callback=self.speech_recognition_callback,
            callback_group=callback_group,
            goal_callback=self.speech_recognition_goal_callback,
            cancel_callback=self.speech_recognition_cancel_callback
        )

        # Publishers for action status and feedback
        self.action_status_pub = self.create_publisher(String, '/action_status', self.action_qos)
        self.action_feedback_pub = self.create_publisher(String, '/action_feedback', self.action_qos)

        # Register behaviors with the behavior manager
        self._register_behaviors()

        # Track active goals
        self.active_goals = {}
        self.goal_lock = threading.Lock()

        self.get_logger().info('Action Server Node for high-level behaviors initialized')

    def destroy_node(self):
        """Clean up action servers when destroying the node"""
        self.navigate_to_pose_server.destroy()
        self.pick_and_place_server.destroy()
        self.speech_recognition_server.destroy()
        super().destroy_node()

    def _register_behaviors(self):
        """Register high-level behaviors with the behavior manager"""
        # Navigation behaviors
        self.behavior_manager.register_behavior('navigate_to_pose', self._navigate_behavior)
        self.behavior_manager.register_behavior('localize_robot', self._localize_behavior)

        # Manipulation behaviors
        self.behavior_manager.register_behavior('pick_object', self._pick_behavior)
        self.behavior_manager.register_behavior('place_object', self._place_behavior)
        self.behavior_manager.register_behavior('grasp_object', self._grasp_behavior)

        # Communication behaviors
        self.behavior_manager.register_behavior('listen_speech', self._listen_behavior)
        self.behavior_manager.register_behavior('recognize_intent', self._recognize_intent_behavior)

        self.get_logger().info('Registered high-level behaviors with behavior manager')

    # Navigate to Pose Action Methods
    def navigate_to_pose_goal_callback(self, goal_request):
        """Accept or reject navigate to pose goal requests."""
        self.get_logger().info('Received navigate_to_pose goal request')
        return GoalResponse.ACCEPT

    def navigate_to_pose_cancel_callback(self, goal_handle):
        """Accept or reject navigate to pose cancel requests."""
        self.get_logger().info('Received request to cancel navigate_to_pose')
        return CancelResponse.ACCEPT

    def navigate_to_pose_callback(self, goal_handle):
        """Execute the navigate to pose action."""
        self.get_logger().info('Executing navigate_to_pose action...')

        # Get the target pose from the goal
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to pose: x={target_pose.position.x}, y={target_pose.position.y}')

        # Create feedback and result messages
        feedback_msg = NavigateToPose.Feedback()
        result_msg = NavigateToPose.Result()

        try:
            # Execute the navigation behavior
            behavior_result = self.behavior_manager.execute_behavior(
                'navigate_to_pose',
                goal_handle,
                target_pose=target_pose
            )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = behavior_result.get('success', False)
            result_msg.message = behavior_result.get('message', 'Navigation completed')
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f'Navigation failed: {str(e)}'
            return result_msg

    # Pick and Place Action Methods
    def pick_and_place_goal_callback(self, goal_request):
        """Accept or reject pick and place goal requests."""
        self.get_logger().info('Received pick_and_place goal request')
        return GoalResponse.ACCEPT

    def pick_and_place_cancel_callback(self, goal_handle):
        """Accept or reject pick and place cancel requests."""
        self.get_logger().info('Received request to cancel pick_and_place')
        return CancelResponse.ACCEPT

    def pick_and_place_callback(self, goal_handle):
        """Execute the pick and place action."""
        self.get_logger().info('Executing pick_and_place action...')

        # Get the pick and place poses from the goal
        pick_pose = goal_handle.request.pick_pose
        place_pose = goal_handle.request.place_pose
        self.get_logger().info(f'Picking from: x={pick_pose.position.x}, y={pick_pose.position.y}, placing at: x={place_pose.position.x}, y={place_pose.position.y}')

        # Create feedback and result messages
        feedback_msg = PickAndPlace.Feedback()
        result_msg = PickAndPlace.Result()

        try:
            # Execute the pick and place behavior
            behavior_result = self.behavior_manager.execute_behavior(
                'pick_object',
                goal_handle,
                target_pose=pick_pose
            )

            if behavior_result.get('success', False):
                behavior_result = self.behavior_manager.execute_behavior(
                    'place_object',
                    goal_handle,
                    target_pose=place_pose
                )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = behavior_result.get('success', False)
            result_msg.message = behavior_result.get('message', 'Pick and place completed')
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f'Pick and place failed: {str(e)}'
            return result_msg

    # Speech Recognition Action Methods
    def speech_recognition_goal_callback(self, goal_request):
        """Accept or reject speech recognition goal requests."""
        self.get_logger().info('Received speech_recognition goal request')
        return GoalResponse.ACCEPT

    def speech_recognition_cancel_callback(self, goal_handle):
        """Accept or reject speech recognition cancel requests."""
        self.get_logger().info('Received request to cancel speech_recognition')
        return CancelResponse.ACCEPT

    def speech_recognition_callback(self, goal_handle):
        """Execute the speech recognition action."""
        self.get_logger().info('Executing speech_recognition action...')

        # Get the duration from the goal
        duration = goal_handle.request.duration
        self.get_logger().info(f'Recognizing speech for {duration} seconds')

        # Create feedback and result messages
        feedback_msg = SpeechRecognition.Feedback()
        result_msg = SpeechRecognition.Result()

        try:
            # Execute the speech recognition behavior
            behavior_result = self.behavior_manager.execute_behavior(
                'listen_speech',
                goal_handle,
                duration=duration
            )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = behavior_result.get('success', False)
            result_msg.recognized_text = behavior_result.get('recognized_text', '')
            result_msg.confidence = behavior_result.get('confidence', 0.0)
            result_msg.message = behavior_result.get('message', 'Speech recognition completed')
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.recognized_text = ''
            result_msg.confidence = 0.0
            result_msg.message = f'Speech recognition failed: {str(e)}'
            return result_msg

    # Behavior Implementations
    def _navigate_behavior(self, goal_handle, **kwargs):
        """Navigation behavior implementation"""
        target_pose = kwargs.get('target_pose')

        # Simulate navigation with feedback
        for progress in range(0, 101, 10):  # 0 to 100% in 10% steps
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Navigation was canceled'}

            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose = target_pose  # In real implementation, this would be actual current pose
            feedback_msg.distance_remaining = 10.0 - (progress / 10.0)  # Simulated distance
            feedback_msg.progress = float(progress)

            # Publish action feedback
            feedback_str = String()
            feedback_str.data = f"Navigation progress: {progress}%"
            self.action_feedback_pub.publish(feedback_str)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Navigation progress: {progress}%')

            time.sleep(0.5)  # Simulate navigation time

        return {'success': True, 'message': 'Navigation completed', 'recognized_text': '', 'confidence': 0.0}

    def _pick_behavior(self, goal_handle, **kwargs):
        """Pick behavior implementation"""
        target_pose = kwargs.get('target_pose')

        # Simulate pick operation with feedback
        steps = [
            ('Approaching object', 20.0),
            ('Positioning gripper', 40.0),
            ('Closing gripper', 60.0),
            ('Lifting object', 80.0),
            ('Retracting', 100.0)
        ]

        for step_name, progress in steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Pick operation was canceled'}

            feedback_msg = PickAndPlace.Feedback()
            feedback_msg.current_step = step_name
            feedback_msg.progress = progress

            # Publish action feedback
            feedback_str = String()
            feedback_str.data = f"Pick operation: {step_name}"
            self.action_feedback_pub.publish(feedback_str)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Pick operation: {step_name} ({progress}%)')

            time.sleep(0.8)  # Simulate action time

        return {'success': True, 'message': 'Pick completed'}

    def _place_behavior(self, goal_handle, **kwargs):
        """Place behavior implementation"""
        target_pose = kwargs.get('target_pose')

        # Simulate place operation with feedback
        steps = [
            ('Moving to place location', 25.0),
            ('Positioning for placement', 50.0),
            ('Opening gripper', 75.0),
            ('Retracting gripper', 100.0)
        ]

        for step_name, progress in steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Place operation was canceled'}

            feedback_msg = PickAndPlace.Feedback()
            feedback_msg.current_step = step_name
            feedback_msg.progress = progress

            # Publish action feedback
            feedback_str = String()
            feedback_str.data = f"Place operation: {step_name}"
            self.action_feedback_pub.publish(feedback_str)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Place operation: {step_name} ({progress}%)')

            time.sleep(0.8)  # Simulate action time

        return {'success': True, 'message': 'Place completed'}

    def _listen_behavior(self, goal_handle, **kwargs):
        """Speech listening behavior implementation"""
        duration = kwargs.get('duration', 5.0)

        # Simulate speech recognition with feedback
        for i in range(int(duration)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {
                    'success': False,
                    'recognized_text': '',
                    'confidence': 0.0,
                    'message': 'Speech recognition was canceled'
                }

            feedback_msg = SpeechRecognition.Feedback()
            feedback_msg.listening_progress = (i + 1) / duration * 100.0
            feedback_msg.interim_text = f'Listening... {i+1}/{int(duration)}'

            # Publish action feedback
            feedback_str = String()
            feedback_str.data = f"Listening... {i+1}/{int(duration)}"
            self.action_feedback_pub.publish(feedback_str)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Speech recognition: Listening... {i+1}/{int(duration)}')

            time.sleep(1.0)

        # Simulate recognition result
        return {
            'success': True,
            'recognized_text': 'Hello, how can I help you?',
            'confidence': 0.95,
            'message': 'Speech recognition completed'
        }

    def _localize_behavior(self, goal_handle, **kwargs):
        """Robot localization behavior implementation"""
        # Simulate localization with feedback
        for progress in range(0, 101, 25):  # 0, 25, 50, 75, 100%
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Localization was canceled'}

            time.sleep(0.3)  # Simulate processing time

        return {'success': True, 'message': 'Localization completed'}

    def _grasp_behavior(self, goal_handle, **kwargs):
        """Object grasping behavior implementation"""
        # Simulate grasping with feedback
        for progress in range(0, 101, 20):  # 0, 20, 40, 60, 80, 100%
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Grasping was canceled'}

            time.sleep(0.2)  # Simulate processing time

        return {'success': True, 'message': 'Grasping completed'}

    def _recognize_intent_behavior(self, goal_handle, **kwargs):
        """Intent recognition behavior implementation"""
        # Simulate intent recognition with feedback
        for progress in range(0, 101, 33):  # 0, 33, 66, 99%
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Intent recognition was canceled'}

            time.sleep(0.3)  # Simulate processing time

        return {'success': True, 'message': 'Intent recognition completed'}


def main(args=None):
    rclpy.init(args=args)
    action_server_node = HighLevelBehaviorActionServer()

    try:
        rclpy.spin(action_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_server_node.destroy_node()
        rclpy.shutdown()


# Define action interfaces since they're not in standard ROS 2 interfaces
# In a real implementation, these would be defined in .action files and generated
# For this example, we'll define simple action classes

class NavigateToPose:
    class Goal:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.pose = Pose()

    class Feedback:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.current_pose = Pose()
            self.distance_remaining = 0.0
            self.progress = 0.0

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""


class PickAndPlace:
    class Goal:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.pick_pose = Pose()
            self.place_pose = Pose()

    class Feedback:
        def __init__(self):
            self.current_step = ""
            self.progress = 0.0

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""


class SpeechRecognition:
    class Goal:
        def __init__(self):
            self.duration = 5.0  # seconds

    class Feedback:
        def __init__(self):
            self.listening_progress = 0.0
            self.interim_text = ""

    class Result:
        def __init__(self):
            self.success = False
            self.recognized_text = ""
            self.confidence = 0.0
            self.message = ""


if __name__ == '__main__':
    main()