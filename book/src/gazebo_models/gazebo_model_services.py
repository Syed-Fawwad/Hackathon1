#!/usr/bin/env python3

"""
Gazebo Model Services for State Management
This node implements services for setting and getting model states in Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header


class GazeboModelServices(Node):
    """
    Gazebo Model Services Node
    Implements services: /gazebo/set_model_state, /gazebo/get_model_state
    """

    def __init__(self):
        super().__init__('gazebo_model_services')

        # Initialize service servers
        self.set_model_state_srv = self.create_service(
            SetModelState,
            '/gazebo/set_model_state',
            self.set_model_state_callback
        )

        self.get_model_state_srv = self.create_service(
            GetModelState,
            '/gazebo/get_model_state',
            self.get_model_state_callback
        )

        # Initialize model states storage
        self.model_states = {}

        self.get_logger().info('Gazebo Model Services initialized')

    def set_model_state_callback(self, request, response):
        """Callback for set model state service"""
        try:
            model_state = request.model_state
            model_name = model_state.model_name

            # Store the model state
            self.model_states[model_name] = {
                'pose': model_state.pose,
                'twist': model_state.twist,
                'reference_frame': model_state.reference_frame
            }

            # Log the update
            self.get_logger().info(
                f'Set model state for {model_name}: '
                f'position=({model_state.pose.position.x}, '
                f'{model_state.pose.position.y}, '
                f'{model_state.pose.position.z})'
            )

            # Set response
            response.success = True
            response.status_message = f'Successfully set state for model {model_name}'

        except Exception as e:
            self.get_logger().error(f'Error in set_model_state: {e}')
            response.success = False
            response.status_message = f'Error setting model state: {str(e)}'

        return response

    def get_model_state_callback(self, request, response):
        """Callback for get model state service"""
        try:
            model_name = request.model_name
            relative_entity_name = request.relative_entity_name

            if model_name in self.model_states:
                # Return stored state
                stored_state = self.model_states[model_name]
                response.pose = stored_state['pose']
                response.twist = stored_state['twist']
                response.success = True
                response.status_message = f'Successfully retrieved state for model {model_name}'
            else:
                # Return default state if model not found
                default_pose = Pose()
                default_pose.position.x = 0.0
                default_pose.position.y = 0.0
                default_pose.position.z = 0.0
                default_pose.orientation.w = 1.0
                default_pose.orientation.x = 0.0
                default_pose.orientation.y = 0.0
                default_pose.orientation.z = 0.0

                default_twist = Twist()
                default_twist.linear.x = 0.0
                default_twist.linear.y = 0.0
                default_twist.linear.z = 0.0
                default_twist.angular.x = 0.0
                default_twist.angular.y = 0.0
                default_twist.angular.z = 0.0

                response.pose = default_pose
                response.twist = default_twist
                response.success = False
                response.status_message = f'Model {model_name} not found in stored states'

        except Exception as e:
            self.get_logger().error(f'Error in get_model_state: {e}')
            response.success = False
            response.status_message = f'Error getting model state: {str(e)}'

        return response

    def update_model_state(self, model_name, pose, twist, reference_frame=''):
        """Programmatically update a model's state"""
        self.model_states[model_name] = {
            'pose': pose,
            'twist': twist,
            'reference_frame': reference_frame
        }

    def get_stored_models(self):
        """Get list of stored model names"""
        return list(self.model_states.keys())

    def remove_model_state(self, model_name):
        """Remove a model's state from storage"""
        if model_name in self.model_states:
            del self.model_states[model_name]
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    services = GazeboModelServices()

    try:
        rclpy.spin(services)
    except KeyboardInterrupt:
        pass
    finally:
        services.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()