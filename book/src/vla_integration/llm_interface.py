# LLM Interface Node
# This node provides integration with language models for understanding and reasoning

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import rclpy


class LLMInterface(Node):
    def __init__(self):
        super().__init__('llm_interface')

        # Create subscriber for parsed intents
        self.intent_sub = self.create_subscription(
            String,
            '/parsed_intent',
            self.intent_callback,
            QoSProfile(depth=10)
        )

        # Create subscriber for visual context
        self.context_sub = self.create_subscription(
            String,
            '/visual_context',
            self.context_callback,
            QoSProfile(depth=10)
        )

        # Create publisher for LLM responses
        self.response_pub = self.create_publisher(
            String,
            '/llm_response',
            QoSProfile(depth=10)
        )

        self.get_logger().info('LLM Interface node initialized')

    def intent_callback(self, msg):
        # Process intent from vision-language fusion
        self.get_logger().info(f'Processing intent: {msg.data}')

    def context_callback(self, msg):
        # Process visual context
        self.get_logger().info(f'Processing visual context: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()