# Humanoid Coordinator Node
# This node coordinates all modules for the capstone autonomous humanoid system

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rclpy


class HumanoidCoordinator(Node):
    def __init__(self):
        super().__init__('humanoid_coordinator')

        # Create subscribers for all system inputs
        self.voice_sub = self.create_subscription(
            String,
            '/user_command',
            self.voice_command_callback,
            QoSProfile(depth=10)
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10)
        )

        # Create publishers for system outputs
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10)
        )

        self.status_pub = self.create_publisher(
            String,
            '/humanoid_state',
            QoSProfile(depth=10)
        )

        self.get_logger().info('Humanoid Coordinator node initialized - Capstone System')

    def voice_command_callback(self, msg):
        self.get_logger().info(f'Received voice command: {msg.data}')
        # Process the voice command through the VLA pipeline

    def image_callback(self, msg):
        self.get_logger().info(f'Received image for context: {msg.width}x{msg.height}')
        # Process the image for visual context


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()