#!/usr/bin/env python3

"""
Action Server Framework for the ROS 2 Communication Framework
This module provides a standardized framework for action servers with behavior management.
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
from abc import ABC, abstractmethod
import uuid


class ActionInterface(ABC):
    """
    Abstract base class for action interfaces
    """

    @abstractmethod
    async def execute_goal(self, goal_handle, feedback_callback=None):
        """Execute the goal and return the result"""
        pass

    @abstractmethod
    def goal_callback(self, goal_request):
        """Handle incoming goal requests"""
        pass

    @abstractmethod
    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests"""
        pass


class BaseActionServer(Node):
    """
    Base class for all action servers that provides common functionality
    """

    def __init__(self, action_name: str, action_type, execute_callback: Callable, **kwargs):
        super().__init__(f'{action_name.replace("/", "_")}_action_server')

        # Use a reentrant callback group to handle multiple actions concurrently
        callback_group = ReentrantCallbackGroup()

        # Create action server
        self.action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=execute_callback,
            callback_group=callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.action_name = action_name
        self.action_type = action_type
        self.active_goals = {}
        self.goal_lock = threading.Lock()

        # Declare parameters
        self.declare_parameter('max_concurrent_goals', 1)
        self.declare_parameter('enable_feedback', True)

        self.max_concurrent_goals = self.get_parameter('max_concurrent_goals').value
        self.enable_feedback = self.get_parameter('enable_feedback').value

        self.get_logger().info(f'Base action server for {action_name} initialized')

    def destroy_node(self):
        """Clean up action server when destroying the node"""
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Default goal callback - accept if under limit"""
        with self.goal_lock:
            active_count = len(self.active_goals)
            if active_count < self.max_concurrent_goals:
                self.get_logger().info(f'Accepting new goal for {self.action_name}, active: {active_count}')
                return GoalResponse.ACCEPT
            else:
                self.get_logger().warn(f'Rejecting goal for {self.action_name}, limit reached: {active_count}')
                return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Default cancel callback - accept all cancellation requests"""
        self.get_logger().info(f'Received cancel request for {self.action_name}')
        return CancelResponse.ACCEPT

    def add_active_goal(self, goal_id: str, goal_handle):
        """Add a goal to the active goals list"""
        with self.goal_lock:
            self.active_goals[goal_id] = goal_handle

    def remove_active_goal(self, goal_id: str):
        """Remove a goal from the active goals list"""
        with self.goal_lock:
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]

    def get_active_goal_count(self) -> int:
        """Get the number of active goals"""
        with self.goal_lock:
            return len(self.active_goals)


class BehaviorManager:
    """
    Manager for high-level behaviors and action coordination
    """

    def __init__(self, node: Node):
        self.node = node
        self.behaviors: Dict[str, Callable] = {}
        self.active_behaviors: Dict[str, str] = {}  # behavior_name -> goal_id
        self.behavior_lock = threading.Lock()

    def register_behavior(self, name: str, behavior_func: Callable):
        """Register a high-level behavior"""
        self.behaviors[name] = behavior_func
        self.node.get_logger().info(f'Registered behavior: {name}')

    def execute_behavior(self, name: str, goal_handle, **kwargs):
        """Execute a registered behavior"""
        if name in self.behaviors:
            with self.behavior_lock:
                goal_id = str(uuid.uuid4())
                self.active_behaviors[name] = goal_id

            try:
                result = self.behaviors[name](goal_handle, **kwargs)
                with self.behavior_lock:
                    if name in self.active_behaviors:
                        del self.active_behaviors[name]
                return result
            except Exception as e:
                self.node.get_logger().error(f'Error executing behavior {name}: {e}')
                with self.behavior_lock:
                    if name in self.active_behaviors:
                        del self.active_behaviors[name]
                raise
        else:
            raise ValueError(f'Behavior {name} not registered')

    def get_active_behaviors(self) -> List[str]:
        """Get list of currently active behaviors"""
        with self.behavior_lock:
            return list(self.active_behaviors.keys())

    def cancel_behavior(self, name: str) -> bool:
        """Cancel a running behavior"""
        with self.behavior_lock:
            if name in self.active_behaviors:
                goal_id = self.active_behaviors[name]
                # In a real implementation, this would cancel the specific goal
                del self.active_behaviors[name]
                return True
        return False


class NavigateToPoseAction(BaseActionServer):
    """
    Navigate to pose action server
    """

    def __init__(self):
        super().__init__('navigate_to_pose', NavigateToPose, self.execute_callback)

        # Initialize behavior manager
        self.behavior_manager = BehaviorManager(self)

        # Register navigation behavior
        self.behavior_manager.register_behavior('navigate', self._navigate_behavior)

        self.get_logger().info('Navigate to pose action server initialized')

    def execute_callback(self, goal_handle):
        """Execute the navigate to pose action"""
        self.get_logger().info('Executing navigate to pose action...')

        # Get the target pose from the goal
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to pose: x={target_pose.position.x}, y={target_pose.position.y}')

        # Create feedback and result messages
        feedback_msg = NavigateToPose.Feedback()
        result_msg = NavigateToPose.Result()

        try:
            # Execute the navigation behavior
            result = self.behavior_manager.execute_behavior(
                'navigate',
                goal_handle,
                target_pose=target_pose
            )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = True
            result_msg.message = 'Successfully navigated to pose'
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f'Navigation failed: {str(e)}'
            return result_msg

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

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Navigation progress: {progress}%')

            time.sleep(0.5)  # Simulate navigation time

        return {'success': True, 'message': 'Navigation completed'}


class PickAndPlaceAction(BaseActionServer):
    """
    Pick and place action server
    """

    def __init__(self):
        super().__init__('pick_and_place', PickAndPlace, self.execute_callback)

        # Initialize behavior manager
        self.behavior_manager = BehaviorManager(self)

        # Register pick and place behavior
        self.behavior_manager.register_behavior('pick_and_place', self._pick_and_place_behavior)

        self.get_logger().info('Pick and place action server initialized')

    def execute_callback(self, goal_handle):
        """Execute the pick and place action"""
        self.get_logger().info('Executing pick and place action...')

        # Get the pick and place poses from the goal
        pick_pose = goal_handle.request.pick_pose
        place_pose = goal_handle.request.place_pose
        self.get_logger().info(f'Picking from: x={pick_pose.position.x}, y={pick_pose.position.y}, placing at: x={place_pose.position.x}, y={place_pose.position.y}')

        # Create feedback and result messages
        feedback_msg = PickAndPlace.Feedback()
        result_msg = PickAndPlace.Result()

        try:
            # Execute the pick and place behavior
            result = self.behavior_manager.execute_behavior(
                'pick_and_place',
                goal_handle,
                pick_pose=pick_pose,
                place_pose=place_pose
            )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = True
            result_msg.message = 'Successfully picked and placed object'
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f'Pick and place failed: {str(e)}'
            return result_msg

    def _pick_and_place_behavior(self, goal_handle, **kwargs):
        """Pick and place behavior implementation"""
        pick_pose = kwargs.get('pick_pose')
        place_pose = kwargs.get('place_pose')

        # Simulate pick and place with feedback
        steps = [
            ('Approaching pick location', 0.2),
            ('Grasping object', 0.4),
            ('Lifting object', 0.6),
            ('Moving to place location', 0.8),
            ('Placing object', 1.0)
        ]

        for i, (step_name, progress) in enumerate(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'message': 'Pick and place was canceled'}

            feedback_msg = PickAndPlace.Feedback()
            feedback_msg.current_step = step_name
            feedback_msg.progress = progress * 100.0

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Pick and place: {step_name} ({progress*100.0}%)')

            time.sleep(1.0)  # Simulate action time

        return {'success': True, 'message': 'Pick and place completed'}


class SpeechRecognitionAction(BaseActionServer):
    """
    Speech recognition action server
    """

    def __init__(self):
        super().__init__('speech_recognition', SpeechRecognition, self.execute_callback)

        # Initialize behavior manager
        self.behavior_manager = BehaviorManager(self)

        # Register speech recognition behavior
        self.behavior_manager.register_behavior('recognize_speech', self._recognize_speech_behavior)

        self.get_logger().info('Speech recognition action server initialized')

    def execute_callback(self, goal_handle):
        """Execute the speech recognition action"""
        self.get_logger().info('Executing speech recognition action...')

        # Get the recognition parameters from the goal
        duration = goal_handle.request.duration
        self.get_logger().info(f'Recognizing speech for {duration} seconds')

        # Create feedback and result messages
        feedback_msg = SpeechRecognition.Feedback()
        result_msg = SpeechRecognition.Result()

        try:
            # Execute the speech recognition behavior
            result = self.behavior_manager.execute_behavior(
                'recognize_speech',
                goal_handle,
                duration=duration
            )

            # Complete the goal
            goal_handle.succeed()
            result_msg.success = True
            result_msg.recognized_text = result.get('recognized_text', '')
            result_msg.confidence = result.get('confidence', 0.0)
            result_msg.message = 'Speech recognition completed'
            return result_msg
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.recognized_text = ''
            result_msg.confidence = 0.0
            result_msg.message = f'Speech recognition failed: {str(e)}'
            return result_msg

    def _recognize_speech_behavior(self, goal_handle, **kwargs):
        """Speech recognition behavior implementation"""
        duration = kwargs.get('duration', 5.0)

        # Simulate speech recognition with feedback
        for i in range(int(duration)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return {'success': False, 'recognized_text': '', 'confidence': 0.0, 'message': 'Recognition was canceled'}

            feedback_msg = SpeechRecognition.Feedback()
            feedback_msg.listening_progress = (i + 1) / duration * 100.0
            feedback_msg.interim_text = f'Listening... {i+1}/{int(duration)}'

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


def main(args=None):
    rclpy.init(args=args)

    # Create action servers
    navigate_server = NavigateToPoseAction()
    pick_place_server = PickAndPlaceAction()
    speech_server = SpeechRecognitionAction()

    # Create an executor and add all nodes
    executor = MultiThreadedExecutor()
    executor.add_node(navigate_server)
    executor.add_node(pick_place_server)
    executor.add_node(speech_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigate_server.destroy_node()
        pick_place_server.destroy_node()
        speech_server.destroy_node()
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