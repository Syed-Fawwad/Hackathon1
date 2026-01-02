#!/usr/bin/env python3

"""
Unity Bridge Node for ROS-Unity Communication
This node facilitates communication between ROS 2 and Unity via TCP/UDP protocols.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, Pose
from rclpy.parameter import Parameter
import socket
import threading
import json
import time
from std_msgs.msg import Header


class UnityBridgeNode(Node):
    """
    Unity Bridge Node
    Facilitates communication between ROS 2 and Unity via TCP/UDP protocols.
    """

    def __init__(self):
        super().__init__('unity_bridge_node')

        # Initialize TCP server for Unity communication
        self.tcp_host = 'localhost'
        self.tcp_port = 9090
        self.tcp_server = None
        self.unity_connection = None
        self.unity_connected = False

        # Initialize publishers and subscribers
        qos_profile = QoSProfile(depth=10)

        # ROS 2 publishers
        self.unity_robot_state_pub = self.create_publisher(
            Float32MultiArray, '/unity_robot_state', qos_profile
        )
        self.unity_camera_feed_pub = self.create_publisher(
            Image, '/unity_camera_feed', qos_profile
        )

        # ROS 2 subscribers
        self.user_commands_sub = self.create_subscription(
            String, '/user_commands', self.user_commands_callback, qos_profile
        )
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, qos_profile
        )
        self.robot_state_sub = self.create_subscription(
            Pose, '/robot_pose', self.robot_pose_callback, qos_profile
        )

        # Unity topics publishers (to send to Unity)
        self.unity_joint_states_pub = self.create_publisher(
            JointState, '/unity_joint_states', qos_profile
        )

        # Start TCP server thread
        self.tcp_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.tcp_thread.start()

        # Timer for publishing Unity robot state
        self.timer = self.create_timer(0.1, self.publish_unity_robot_state)  # 10 Hz

        self.get_logger().info(f'Unity Bridge Node initialized on {self.tcp_host}:{self.tcp_port}')

    def start_tcp_server(self):
        """Start TCP server to communicate with Unity"""
        try:
            self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server.bind((self.tcp_host, self.tcp_port))
            self.tcp_server.listen(1)
            self.get_logger().info(f'Unity Bridge TCP server listening on {self.tcp_host}:{self.tcp_port}')

            while rclpy.ok():
                try:
                    self.unity_connection, addr = self.tcp_server.accept()
                    self.get_logger().info(f'Unity client connected from {addr}')
                    self.unity_connected = True

                    # Handle Unity client communication
                    while self.unity_connected:
                        data = self.unity_connection.recv(4096)
                        if not data:
                            break

                        try:
                            # Parse JSON message from Unity
                            message = json.loads(data.decode('utf-8'))
                            self.handle_unity_message(message)
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f'Error decoding JSON from Unity: {e}')
                        except Exception as e:
                            self.get_logger().error(f'Error processing Unity message: {e}')

                except ConnectionResetError:
                    self.get_logger().info('Unity client disconnected')
                    self.unity_connected = False
                    if self.unity_connection:
                        self.unity_connection.close()

        except Exception as e:
            self.get_logger().error(f'Error starting TCP server: {e}')

    def handle_unity_message(self, message):
        """Handle incoming messages from Unity"""
        msg_type = message.get('type', '')

        if msg_type == 'robot_state':
            # Forward Unity robot state to ROS 2
            self.forward_unity_robot_state(message)
        elif msg_type == 'sensor_data':
            # Forward Unity sensor data to ROS 2
            self.forward_unity_sensor_data(message)
        elif msg_type == 'user_input':
            # Forward Unity user input to ROS 2
            self.forward_unity_user_input(message)
        else:
            self.get_logger().warn(f'Unknown message type from Unity: {msg_type}')

    def forward_unity_robot_state(self, message):
        """Forward robot state from Unity to ROS 2"""
        try:
            # Create and publish Float32MultiArray message
            robot_state_msg = Float32MultiArray()
            robot_state_msg.data = message.get('data', [])
            self.unity_robot_state_pub.publish(robot_state_msg)
        except Exception as e:
            self.get_logger().error(f'Error forwarding Unity robot state: {e}')

    def forward_unity_sensor_data(self, message):
        """Forward sensor data from Unity to ROS 2"""
        try:
            # For now, just log the sensor data
            self.get_logger().info(f'Unity sensor data: {message.get("data", {})}')
        except Exception as e:
            self.get_logger().error(f'Error forwarding Unity sensor data: {e}')

    def forward_unity_user_input(self, message):
        """Forward user input from Unity to ROS 2"""
        try:
            # Publish user command as String message
            command_msg = String()
            command_msg.data = message.get('command', '')

            # Use a different topic for Unity user commands to distinguish from other sources
            unity_user_commands_pub = self.create_publisher(
                String, '/unity_user_commands', QoSProfile(depth=10)
            )
            unity_user_commands_pub.publish(command_msg)
        except Exception as e:
            self.get_logger().error(f'Error forwarding Unity user input: {e}')

    def send_to_unity(self, message):
        """Send message to Unity client"""
        if self.unity_connected and self.unity_connection:
            try:
                json_message = json.dumps(message)
                self.unity_connection.send(json_message.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Error sending to Unity: {e}')
                self.unity_connected = False

    def user_commands_callback(self, msg):
        """Callback for user commands from ROS 2"""
        try:
            # Forward user command to Unity
            unity_message = {
                'type': 'user_command',
                'command': msg.data
            }
            self.send_to_unity(unity_message)
        except Exception as e:
            self.get_logger().error(f'Error in user_commands_callback: {e}')

    def joint_states_callback(self, msg):
        """Callback for joint states from ROS 2"""
        try:
            # Forward joint states to Unity
            joint_state_data = {
                'type': 'joint_states',
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'efforts': list(msg.effort)
            }
            self.send_to_unity(joint_state_data)
        except Exception as e:
            self.get_logger().error(f'Error in joint_states_callback: {e}')

    def robot_pose_callback(self, msg):
        """Callback for robot pose from ROS 2"""
        try:
            # Forward robot pose to Unity
            pose_data = {
                'type': 'robot_pose',
                'position': {
                    'x': msg.position.x,
                    'y': msg.position.y,
                    'z': msg.position.z
                },
                'orientation': {
                    'w': msg.orientation.w,
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z
                }
            }
            self.send_to_unity(pose_data)
        except Exception as e:
            self.get_logger().error(f'Error in robot_pose_callback: {e}')

    def publish_unity_robot_state(self):
        """Publish Unity robot state periodically"""
        try:
            if self.unity_connected:
                # Create a dummy robot state message for Unity
                robot_state_msg = Float32MultiArray()
                robot_state_msg.data = [0.0, 0.0, 0.0]  # x, y, z position
                self.unity_robot_state_pub.publish(robot_state_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing Unity robot state: {e}')

    def destroy_node(self):
        """Clean up resources"""
        self.unity_connected = False
        if self.tcp_server:
            self.tcp_server.close()
        if self.unity_connection:
            self.unity_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge = UnityBridgeNode()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()