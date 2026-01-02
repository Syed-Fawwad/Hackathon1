#!/usr/bin/env python3

"""
Robot State Publisher for TF Broadcasting
This node publishes the robot's state to the TF tree based on joint states and URDF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
import math
from std_msgs.msg import Header
import xml.dom.minidom
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time as TimeMsg


class RobotStatePublisher(Node):
    """
    Robot State Publisher Node
    Publishes the robot's state to the TF tree based on joint states and URDF model.
    """

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize joint state subscriber
        qos_profile = QoSProfile(depth=10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            qos_profile
        )

        # Load robot description (URDF)
        self.declare_parameter('robot_description', '')
        self.robot_description = self.get_parameter('robot_description').value

        # Parse URDF to get joint information
        self.joint_info = {}
        self.fixed_joints = []
        self.parent_links = {}

        if self.robot_description:
            self.parse_urdf(self.robot_description)
        else:
            self.get_logger().warn('No robot_description parameter found')

        # Timer for publishing static transforms
        self.static_tf_timer = self.create_timer(1.0, self.publish_static_transforms)

        self.get_logger().info('Robot State Publisher initialized')

    def parse_urdf(self, urdf_string):
        """Parse URDF to extract joint and link information"""
        try:
            dom = xml.dom.minidom.parseString(urdf_string)
            robot = dom.documentElement

            # Get all joints
            for joint in robot.getElementsByTagName('joint'):
                joint_name = joint.getAttribute('name')
                joint_type = joint.getAttribute('type')

                # Get parent and child links
                parent_elements = joint.getElementsByTagName('parent')
                child_elements = joint.getElementsByTagName('child')

                if parent_elements and child_elements:
                    parent_link = parent_elements[0].getAttribute('link')
                    child_link = child_elements[0].getAttribute('link')

                    # Get origin (transform from parent to joint)
                    origin_elements = joint.getElementsByTagName('origin')
                    if origin_elements:
                        origin = origin_elements[0]
                        xyz = origin.getAttribute('xyz').split()
                        rpy = origin.getAttribute('rpy').split()

                        transform = {
                            'xyz': [float(x) for x in xyz],
                            'rpy': [float(r) for r in rpy]
                        }
                    else:
                        transform = {
                            'xyz': [0.0, 0.0, 0.0],
                            'rpy': [0.0, 0.0, 0.0]
                        }

                    if joint_type == 'fixed':
                        self.fixed_joints.append({
                            'name': joint_name,
                            'parent': parent_link,
                            'child': child_link,
                            'transform': transform
                        })
                    else:
                        self.joint_info[joint_name] = {
                            'parent': parent_link,
                            'child': child_link,
                            'type': joint_type,
                            'transform': transform
                        }

                    self.parent_links[child_link] = parent_link

            # Get all links to find the base link (root)
            all_child_links = set()
            for joint in self.fixed_joints + list(self.joint_info.values()):
                all_child_links.add(joint['child'])

            # Find links that are not children of any joint (root/base links)
            for link in robot.getElementsByTagName('link'):
                link_name = link.getAttribute('name')
                if link_name not in all_child_links:
                    self.base_link = link_name
                    break
            else:
                self.base_link = 'base_link'  # Default if not found

        except Exception as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        try:
            transforms = []

            # Process each joint state
            for i, joint_name in enumerate(msg.name):
                if joint_name in self.joint_info:
                    joint_info = self.joint_info[joint_name]

                    # Create transform from parent to child
                    transform = TransformStamped()
                    transform.header.stamp = msg.header.stamp
                    transform.header.frame_id = joint_info['parent']
                    transform.child_frame_id = joint_info['child']

                    # Set translation from joint origin
                    origin = joint_info['transform']
                    transform.transform.translation.x = origin['xyz'][0]
                    transform.transform.translation.y = origin['xyz'][1]
                    transform.transform.translation.z = origin['xyz'][2]

                    # Calculate rotation based on joint type and position
                    if joint_info['type'] in ['revolute', 'continuous']:
                        # For revolute joints, use the joint position
                        angle = msg.position[i] if i < len(msg.position) else 0.0

                        # For simplicity, assume rotation around Z axis
                        # In a real implementation, you'd use the joint axis
                        cy = math.cos(angle * 0.5)
                        sy = math.sin(angle * 0.5)
                        cp = math.cos(0.0 * 0.5)
                        sp = math.sin(0.0 * 0.5)
                        cr = math.cos(0.0 * 0.5)
                        sr = math.sin(0.0 * 0.5)

                        transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
                        transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
                        transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
                        transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
                    else:
                        # For other joint types, use the fixed orientation
                        roll, pitch, yaw = origin['rpy']

                        cy = math.cos(yaw * 0.5)
                        sy = math.sin(yaw * 0.5)
                        cp = math.cos(pitch * 0.5)
                        sp = math.sin(pitch * 0.5)
                        cr = math.cos(roll * 0.5)
                        sr = math.sin(roll * 0.5)

                        transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
                        transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
                        transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
                        transform.transform.rotation.z = cr * cp * sy - sr * sp * cy

                    transforms.append(transform)

            # Publish all transforms
            if transforms:
                self.tf_broadcaster.sendTransform(transforms)

        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')

    def publish_static_transforms(self):
        """Publish static transforms for fixed joints"""
        try:
            current_time = self.get_clock().now().to_msg()
            transforms = []

            # Publish fixed joint transforms
            for fixed_joint in self.fixed_joints:
                transform = TransformStamped()
                transform.header.stamp = current_time
                transform.header.frame_id = fixed_joint['parent']
                transform.child_frame_id = fixed_joint['child']

                # Set translation
                origin = fixed_joint['transform']
                transform.transform.translation.x = origin['xyz'][0]
                transform.transform.translation.y = origin['xyz'][1]
                transform.transform.translation.z = origin['xyz'][2]

                # Set rotation (from RPY to quaternion)
                roll, pitch, yaw = origin['rpy']

                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)

                transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
                transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
                transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
                transform.transform.rotation.z = cr * cp * sy - sr * sp * cy

                transforms.append(transform)

            if transforms:
                self.tf_broadcaster.sendTransform(transforms)

        except Exception as e:
            self.get_logger().error(f'Error publishing static transforms: {e}')


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()