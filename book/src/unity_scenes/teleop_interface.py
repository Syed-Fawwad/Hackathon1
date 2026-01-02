#!/usr/bin/env python3

"""
Teleoperation Interface for Human-Robot Interaction
This node provides a teleoperation interface for controlling the robot through various input methods.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.parameter import Parameter
import sys
import select
import termios
import tty
import threading
from std_msgs.msg import Header


class TeleopInterface(Node):
    """
    Teleoperation Interface Node
    Provides teleoperation capabilities for human-robot interaction.
    Supports keyboard, joystick, and GUI-based control.
    """

    def __init__(self):
        super().__init__('teleop_interface')

        # Initialize publishers
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.user_commands_pub = self.create_publisher(String, '/user_commands', qos_profile)
        self.teleop_status_pub = self.create_publisher(Bool, '/teleop_active', qos_profile)

        # Initialize subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joystick_callback, qos_profile
        )
        self.teleop_command_sub = self.create_subscription(
            String, '/teleop_command', self.teleop_command_callback, qos_profile
        )

        # Initialize action server for teleop
        self._action_server = ActionServer(
            self,
            # Using a placeholder action type since we need to define it
            # In a real implementation, we'd define a custom action
            None,  # This would be a custom action type
            'teleop_action',
            self.execute_teleop_callback,
            goal_callback=self.goal_teleop_callback,
            cancel_callback=self.cancel_teleop_callback
        )

        # Teleoperation parameters
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.teleop_active = False
        self.joy_enabled = True
        self.keyboard_enabled = True

        # Declare parameters
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('enable_joy', True)
        self.declare_parameter('enable_keyboard', True)

        # Update parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.joy_enabled = self.get_parameter('enable_joy').value
        self.keyboard_enabled = self.get_parameter('enable_keyboard').value

        # Timer for publishing teleop status
        self.status_timer = self.create_timer(1.0, self.publish_teleop_status)

        # Keyboard input handling
        self.old_settings = None
        self.key_thread = None
        self.running = True

        if self.keyboard_enabled:
            self.setup_keyboard_input()

        self.get_logger().info('Teleoperation Interface initialized')

    def setup_keyboard_input(self):
        """Setup keyboard input for teleoperation"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            self.key_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
            self.key_thread.start()
        except Exception as e:
            self.get_logger().warn(f'Could not setup keyboard input: {e}')
            self.keyboard_enabled = False

    def keyboard_input_loop(self):
        """Main loop for keyboard input"""
        try:
            while self.running and self.keyboard_enabled:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.process_keyboard_input(key)
        except Exception as e:
            self.get_logger().error(f'Error in keyboard input loop: {e}')

    def process_keyboard_input(self, key):
        """Process keyboard input for robot control"""
        if not self.teleop_active:
            return

        twist_msg = Twist()

        if key == 'w':  # Forward
            twist_msg.linear.x = self.linear_scale
        elif key == 's':  # Backward
            twist_msg.linear.x = -self.linear_scale
        elif key == 'a':  # Left
            twist_msg.angular.z = self.angular_scale
        elif key == 'd':  # Right
            twist_msg.angular.z = -self.angular_scale
        elif key == 'q':  # Rotate left
            twist_msg.angular.z = self.angular_scale
        elif key == 'e':  # Rotate right
            twist_msg.angular.z = -self.angular_scale
        elif key == ' ':  # Stop
            # Zero velocities
            pass
        elif key == '\x03':  # Ctrl+C to exit
            self.running = False
            return
        else:
            # Unknown key, don't send command
            return

        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f'Sent velocity command: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

    def joystick_callback(self, msg):
        """Callback for joystick input"""
        if not self.joy_enabled or not self.teleop_active:
            return

        # Assuming standard joystick layout:
        # Axes: [left_right, up_down, twist, etc.]
        # Buttons: [0, 1, 2, 3, etc.]

        twist_msg = Twist()

        # Map joystick axes to robot velocities
        # Axis 0: typically left/right movement (angular)
        # Axis 1: typically forward/backward movement (linear)
        if len(msg.axes) >= 2:
            twist_msg.linear.x = msg.axes[1] * self.linear_scale
            twist_msg.angular.z = msg.axes[0] * self.angular_scale

        # Additional controls could be added here
        # For example, using triggers for different speed modes
        if len(msg.axes) >= 6:
            # Left and right triggers (axes 2 and 5 are common for triggers)
            left_trigger = msg.axes[2]  # Range [-1, 1] where -1 is pressed
            right_trigger = msg.axes[5]  # Range [-1, 1] where -1 is pressed
            # Use triggers for speed adjustment
            speed_factor = 1.0 + (right_trigger - left_trigger) / 2.0
            twist_msg.linear.x *= speed_factor
            twist_msg.angular.z *= speed_factor

        # Process buttons for special commands
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:  # Button 0 pressed
            # Emergency stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(twist_msg)

    def teleop_command_callback(self, msg):
        """Callback for teleoperation commands from other nodes/UI"""
        try:
            command = msg.data.lower().strip()

            if command == 'activate':
                self.teleop_active = True
                self.get_logger().info('Teleoperation activated')
            elif command == 'deactivate':
                self.teleop_active = False
                # Send stop command
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                self.get_logger().info('Teleoperation deactivated')
            elif command.startswith('move:'):
                # Parse move command: "move:x,y,theta"
                parts = command[5:].split(',')
                if len(parts) == 3:
                    try:
                        x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
                        twist_msg = Twist()
                        twist_msg.linear.x = x
                        twist_msg.linear.y = y
                        twist_msg.angular.z = theta
                        self.cmd_vel_pub.publish(twist_msg)
                    except ValueError:
                        self.get_logger().warn(f'Invalid move command format: {command}')
            elif command.startswith('speed:'):
                # Parse speed command: "speed:linear_scale,angular_scale"
                parts = command[6:].split(',')
                if len(parts) == 2:
                    try:
                        linear_scale, angular_scale = float(parts[0]), float(parts[1])
                        self.linear_scale = linear_scale
                        self.angular_scale = angular_scale
                        self.get_logger().info(f'Speed updated: linear={linear_scale}, angular={angular_scale}')
                    except ValueError:
                        self.get_logger().warn(f'Invalid speed command format: {command}')
            else:
                self.get_logger().warn(f'Unknown teleop command: {command}')

        except Exception as e:
            self.get_logger().error(f'Error processing teleop command: {e}')

    def publish_teleop_status(self):
        """Publish teleoperation status"""
        status_msg = Bool()
        status_msg.data = self.teleop_active
        self.teleop_status_pub.publish(status_msg)

    def goal_teleop_callback(self, goal_request):
        """Handle teleop action goal request"""
        # Accept all goals for now
        return GoalResponse.ACCEPT

    def cancel_teleop_callback(self, goal_handle):
        """Handle teleop action cancel request"""
        # Accept all cancel requests for now
        return CancelResponse.ACCEPT

    def execute_teleop_callback(self, goal_handle):
        """Execute teleop action"""
        # This would be implemented with a proper action definition
        # For now, just return success
        goal_handle.succeed()
        result = None  # This would be a proper result message
        return result

    def send_user_command(self, command):
        """Send a user command to the system"""
        cmd_msg = String()
        cmd_msg.data = command
        self.user_commands_pub.publish(cmd_msg)

    def activate_teleop(self):
        """Activate teleoperation mode"""
        self.teleop_active = True
        self.publish_teleop_status()
        self.get_logger().info('Teleoperation mode activated')

    def deactivate_teleop(self):
        """Deactivate teleoperation mode"""
        self.teleop_active = False
        # Send stop command
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.publish_teleop_status()
        self.get_logger().info('Teleoperation mode deactivated')

    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_interface = TeleopInterface()

    try:
        rclpy.spin(teleop_interface)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()