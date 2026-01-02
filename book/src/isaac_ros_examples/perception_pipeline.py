# Isaac ROS Perception Pipeline
# This node implements perception processing using Isaac ROS packages

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
import rclpy


class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Create subscribers for Isaac camera data
        self.rgb_sub = self.create_subscription(
            Image,
            '/isaac_rgb',
            self.rgb_callback,
            QoSProfile(depth=10)
        )

        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/perception_detections',
            QoSProfile(depth=10)
        )

        self.get_logger().info('Perception Pipeline node initialized')

    def rgb_callback(self, msg):
        # Process the RGB image and perform object detection
        # This is a simplified version - in reality, this would use Isaac ROS perception packages
        self.get_logger().info(f'Processing RGB image: {msg.width}x{msg.height}')

        # In a real implementation, this would run Isaac ROS perception algorithms
        # For demo purposes, we'll just log the image info


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()