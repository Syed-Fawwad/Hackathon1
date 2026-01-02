#!/usr/bin/env python3

"""
Service Server Node for Request/Response Patterns
This node implements services for request/response communication patterns in the humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass


class ServiceServerNode(Node):
    """
    Service Server Node for request/response patterns
    Provides various services for the robot system with proper request/response handling.
    """

    def __init__(self):
        super().__init__('service_server_node')

        # Initialize callback group for handling multiple services concurrently
        callback_group = ReentrantCallbackGroup()

        # Initialize robot state
        self.robot_state = {
            'mode': 'idle',
            'status': 'operational',
            'battery_level': 85.0,
            'active_nodes': ['perception', 'planning', 'control'],
            'last_update': time.time()
        }

        # Initialize service clients for internal communication
        self.service_clients = {}

        # Create services
        self.get_robot_status_srv = self.create_service(
            GetRobotStatus,
            '/get_robot_status',
            self.get_robot_status_callback,
            callback_group=callback_group
        )

        self.set_robot_mode_srv = self.create_service(
            SetRobotMode,
            '/set_robot_mode',
            self.set_robot_mode_callback,
            callback_group=callback_group
        )

        self.get_robot_pose_srv = self.create_service(
            GetRobotPose,
            '/get_robot_pose',
            self.get_robot_pose_callback,
            callback_group=callback_group
        )

        self.set_robot_pose_srv = self.create_service(
            SetRobotPose,
            '/set_robot_pose',
            self.set_robot_pose_callback,
            callback_group=callback_group
        )

        self.get_joint_states_srv = self.create_service(
            GetJointStates,
            '/get_joint_states',
            self.get_joint_states_callback,
            callback_group=callback_group
        )

        self.set_joint_positions_srv = self.create_service(
            SetJointPositions,
            '/set_joint_positions',
            self.set_joint_positions_callback,
            callback_group=callback_group
        )

        self.cancel_goal_srv = self.create_service(
            CancelGoal,
            '/cancel_goal',
            self.cancel_goal_callback,
            callback_group=callback_group
        )

        self.get_result_srv = self.create_service(
            GetResult,
            '/get_result',
            self.get_result_callback,
            callback_group=callback_group
        )

        # Publishers for status updates
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.service_status_pub = self.create_publisher(String, '/service_status', 10)

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # Parameter declarations
        self.declare_parameter('enable_service_logging', True)
        self.declare_parameter('service_timeout', 5.0)

        self.enable_logging = self.get_parameter('enable_service_logging').value
        self.service_timeout = self.get_parameter('service_timeout').value

        self.get_logger().info('Service Server Node initialized with request/response pattern services')

    def _publish_status(self):
        """Publish robot status periodically"""
        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': time.time(),
            'robot_state': self.robot_state
        })
        self.status_pub.publish(status_msg)

    def get_robot_status_callback(self, request, response):
        """Callback for get_robot_status service"""
        if self.enable_logging:
            self.get_logger().info('Received request for robot status')

        # Update timestamp
        self.robot_state['last_update'] = time.time()

        # Populate response
        response.operational = self.robot_state['status'] == 'operational'
        response.mode = self.robot_state['mode']
        response.battery_level = self.robot_state['battery_level']
        response.error_code = 0  # 0 indicates no error
        response.timestamp = str(self.robot_state['last_update'])
        response.message = f"Robot status: {self.robot_state['status']}, mode: {self.robot_state['mode']}"

        if self.enable_logging:
            self.get_logger().info(f'Returning robot status: mode={response.mode}, battery={response.battery_level}%')

        # Publish service status
        service_status = String()
        service_status.data = f"get_robot_status: returned status for mode {response.mode}"
        self.service_status_pub.publish(service_status)

        return response

    def set_robot_mode_callback(self, request, response):
        """Callback for set_robot_mode service"""
        requested_mode = request.mode
        if self.enable_logging:
            self.get_logger().info(f'Received request to set robot mode to: {requested_mode}')

        # Validate and set the mode
        valid_modes = ['idle', 'autonomous', 'manual', 'emergency', 'calibration', 'maintenance', 'navigation', 'manipulation']

        if requested_mode in valid_modes:
            old_mode = self.robot_state['mode']
            self.robot_state['mode'] = requested_mode

            response.success = True
            response.message = f'Mode changed from {old_mode} to {requested_mode}'
            response.previous_mode = old_mode
            response.new_mode = requested_mode

            if self.enable_logging:
                self.get_logger().info(f'Mode successfully changed from {old_mode} to {requested_mode}')
        else:
            response.success = False
            response.message = f'Invalid mode: {requested_mode}. Valid modes: {valid_modes}'
            response.previous_mode = self.robot_state['mode']
            response.new_mode = requested_mode

            if self.enable_logging:
                self.get_logger().warn(f'Invalid mode requested: {requested_mode}')

        # Publish service status
        service_status = String()
        service_status.data = f"set_robot_mode: {'success' if response.success else 'failed'} - {requested_mode}"
        self.service_status_pub.publish(service_status)

        return response

    def get_robot_pose_callback(self, request, response):
        """Callback for get_robot_pose service"""
        if self.enable_logging:
            self.get_logger().info('Received request for robot pose')

        # Simulate getting current robot pose
        # In a real implementation, this would interface with localization system
        response.success = True
        response.pose.position.x = 1.0 + (time.time() % 10) * 0.1  # Simulated position
        response.pose.position.y = 2.0 + (time.time() % 10) * 0.1
        response.pose.position.z = 0.0
        response.pose.orientation.x = 0.0
        response.pose.orientation.y = 0.0
        response.pose.orientation.z = 0.0
        response.pose.orientation.w = 1.0  # No rotation
        response.frame_id = 'map'
        response.timestamp = str(time.time())
        response.message = 'Current robot pose retrieved'

        if self.enable_logging:
            self.get_logger().info(f'Returning robot pose: x={response.pose.position.x}, y={response.pose.position.y}')

        # Publish service status
        service_status = String()
        service_status.data = f"get_robot_pose: returned pose at ({response.pose.position.x}, {response.pose.position.y})"
        self.service_status_pub.publish(service_status)

        return response

    def set_robot_pose_callback(self, request, response):
        """Callback for set_robot_pose service"""
        target_pose = request.pose
        frame_id = request.frame_id
        if self.enable_logging:
            self.get_logger().info(f'Received request to set robot pose: x={target_pose.position.x}, y={target_pose.position.y}, frame={frame_id}')

        # In a real implementation, this would send the robot to the target pose
        # For simulation, we'll just acknowledge the request
        response.success = True
        response.message = f'Set robot pose to x={target_pose.position.x}, y={target_pose.position.y} in frame {frame_id}'
        response.executed_pose = target_pose
        response.timestamp = str(time.time())

        if self.enable_logging:
            self.get_logger().info(f'Set robot pose executed: {response.message}')

        # Publish service status
        service_status = String()
        service_status.data = f"set_robot_pose: success - moved to ({target_pose.position.x}, {target_pose.position.y})"
        self.service_status_pub.publish(service_status)

        return response

    def get_joint_states_callback(self, request, response):
        """Callback for get_joint_states service"""
        if self.enable_logging:
            self.get_logger().info('Received request for joint states')

        # Simulate getting current joint states
        # In a real implementation, this would interface with joint controllers
        joint_names = ['left_wheel_joint', 'right_wheel_joint', 'head_pan_joint', 'head_tilt_joint',
                      'left_arm_shoulder', 'left_arm_elbow', 'right_arm_shoulder', 'right_arm_elbow']
        positions = [0.1, 0.1, 0.0, 0.0, 0.5, 0.3, 0.5, 0.3]  # Simulated positions
        velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Simulated velocities
        efforts = [0.5, 0.5, 0.2, 0.2, 1.0, 0.8, 1.0, 0.8]    # Simulated efforts

        response.success = True
        response.joint_state.name = joint_names
        response.joint_state.position = positions
        response.joint_state.velocity = velocities
        response.joint_state.effort = efforts
        response.joint_state.header.stamp = self.get_clock().now().to_msg()
        response.joint_state.header.frame_id = 'base_link'
        response.message = f'Retrieved states for {len(joint_names)} joints'
        response.timestamp = str(time.time())

        if self.enable_logging:
            self.get_logger().info(f'Returning joint states for {len(joint_names)} joints')

        # Publish service status
        service_status = String()
        service_status.data = f"get_joint_states: returned states for {len(joint_names)} joints"
        self.service_status_pub.publish(service_status)

        return response

    def set_joint_positions_callback(self, request, response):
        """Callback for set_joint_positions service"""
        target_positions = request.target_positions
        joint_names = request.joint_names
        if self.enable_logging:
            self.get_logger().info(f'Received request to set joint positions for {len(joint_names)} joints')

        # In a real implementation, this would command the joint controllers
        # For simulation, we'll just acknowledge the request
        response.success = True
        response.message = f'Set positions for {len(joint_names)} joints'
        response.achieved_positions = target_positions  # In simulation, we assume perfect execution
        response.executed_joint_names = joint_names
        response.timestamp = str(time.time())

        if self.enable_logging:
            self.get_logger().info(f'Set joint positions executed for {len(joint_names)} joints')

        # Publish service status
        service_status = String()
        service_status.data = f"set_joint_positions: success - set {len(joint_names)} joints"
        self.service_status_pub.publish(service_status)

        return response

    def cancel_goal_callback(self, request, response):
        """Callback for cancel_goal service"""
        goal_id = request.goal_id
        if self.enable_logging:
            self.get_logger().info(f'Received request to cancel goal: {goal_id}')

        # In a real implementation, this would cancel the specific goal
        # For simulation, we'll just acknowledge the request
        response.success = True
        response.message = f'Goal {goal_id} cancellation requested'
        response.goal_id = goal_id
        response.timestamp = str(time.time())

        if self.enable_logging:
            self.get_logger().info(f'Goal cancellation requested for: {goal_id}')

        # Publish service status
        service_status = String()
        service_status.data = f"cancel_goal: requested cancellation for {goal_id}"
        self.service_status_pub.publish(service_status)

        return response

    def get_result_callback(self, request, response):
        """Callback for get_result service"""
        request_id = request.request_id
        result_type = request.result_type
        if self.enable_logging:
            self.get_logger().info(f'Received request for result: {request_id}, type: {result_type}')

        # In a real implementation, this would retrieve the result of a previous request
        # For simulation, we'll return a mock result
        response.success = True
        response.result_data = f'Mock result for {result_type} request {request_id}'
        response.timestamp = str(time.time())
        response.message = f'Retrieved result for {result_type} request {request_id}'

        if self.enable_logging:
            self.get_logger().info(f'Returned mock result for request {request_id}')

        # Publish service status
        service_status = String()
        service_status.data = f"get_result: returned result for {request_id}"
        self.service_status_pub.publish(service_status)

        return response


# Define service interfaces since they're not in standard ROS 2 interfaces
# In a real implementation, these would be defined in .srv files and generated
# For this example, we'll define simple service classes

class GetRobotStatus:
    class Request:
        def __init__(self):
            pass

    class Response:
        def __init__(self):
            self.operational = False
            self.mode = ""
            self.battery_level = 0.0
            self.error_code = 0
            self.timestamp = ""
            self.message = ""


class SetRobotMode:
    class Request:
        def __init__(self):
            self.mode = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""
            self.previous_mode = ""
            self.new_mode = ""


class GetRobotPose:
    class Request:
        def __init__(self):
            self.frame_id = "map"

    class Response:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.success = False
            self.pose = Pose()
            self.frame_id = ""
            self.timestamp = ""
            self.message = ""


class SetRobotPose:
    class Request:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.pose = Pose()
            self.frame_id = "map"

    class Response:
        def __init__(self):
            from geometry_msgs.msg import Pose
            self.success = False
            self.message = ""
            self.executed_pose = Pose()
            self.timestamp = ""


class GetJointStates:
    class Request:
        def __init__(self):
            self.joint_names = []

    class Response:
        def __init__(self):
            from sensor_msgs.msg import JointState
            self.success = False
            self.joint_state = JointState()
            self.message = ""
            self.timestamp = ""


class SetJointPositions:
    class Request:
        def __init__(self):
            self.joint_names = []
            self.target_positions = []

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""
            self.achieved_positions = []
            self.executed_joint_names = []
            self.timestamp = ""


class CancelGoal:
    class Request:
        def __init__(self):
            self.goal_id = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""
            self.goal_id = ""
            self.timestamp = ""


class GetResult:
    class Request:
        def __init__(self):
            self.request_id = ""
            self.result_type = ""

    class Response:
        def __init__(self):
            self.success = False
            self.result_data = ""
            self.timestamp = ""
            self.message = ""


def main(args=None):
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()