# Isaac Sim Bridge Node
# This node provides integration between Isaac Sim and ROS 2

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
import rclpy


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        # Create publishers for Isaac Sim topics
        self.rgb_publisher = self.create_publisher(
            Image,
            '/isaac_rgb',
            QoSProfile(depth=10)
        )

        self.depth_publisher = self.create_publisher(
            Image,
            '/isaac_depth',
            QoSProfile(depth=10)
        )

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10)
        )

        self.get_logger().info('Isaac Sim Bridge node initialized')

    def cmd_vel_callback(self, msg):
        # Handle velocity commands from ROS to Isaac Sim
        self.get_logger().info(f'Received velocity command: {msg.linear.x}, {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()