#!/usr/bin/env python3

"""
Gazebo ROS Spawner for Robot Spawning
This node provides a standardized interface for spawning robots in Gazebo simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
import os
import time
from typing import Dict, Any, Optional


class GazeboRosSpawner(Node):
    """
    Gazebo ROS Spawner Node
    Provides standardized interface for spawning and managing robots in Gazebo simulation.
    """

    def __init__(self):
        super().__init__('gazebo_ros_spawner')

        # Initialize spawner
        self.spawned_entities = {}

        # Create client for spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Wait for services to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')

        # Create service for spawning robots
        self.spawn_robot_srv = self.create_service(
            SpawnRobot,
            '/gazebo_ros_spawner/spawn_robot',
            self.spawn_robot_callback
        )

        # Create service for deleting robots
        self.delete_robot_srv = self.create_service(
            DeleteRobot,
            '/gazebo_ros_spawner/delete_robot',
            self.delete_robot_callback
        )

        # Publishers for status updates
        self.spawn_status_pub = self.create_publisher(String, '/gazebo_ros_spawner/spawn_status', 10)

        # Timer for monitoring spawned entities
        self.monitor_timer = self.create_timer(2.0, self._monitor_entities)

        # Parameter declarations
        self.declare_parameter('default_spawn_x', 0.0)
        self.declare_parameter('default_spawn_y', 0.0)
        self.declare_parameter('default_spawn_z', 0.5)
        self.declare_parameter('default_spawn_roll', 0.0)
        self.declare_parameter('default_spawn_pitch', 0.0)
        self.declare_parameter('default_spawn_yaw', 0.0)

        self.get_logger().info('Gazebo ROS Spawner initialized')

    def spawn_robot_callback(self, request, response):
        """Callback for spawn robot service"""
        robot_name = request.name
        robot_type = request.type
        x = request.x if request.x is not None else self.get_parameter('default_spawn_x').value
        y = request.y if request.y is not None else self.get_parameter('default_spawn_y').value
        z = request.z if request.z is not None else self.get_parameter('default_spawn_z').value
        roll = request.roll if request.roll is not None else self.get_parameter('default_spawn_roll').value
        pitch = request.pitch if request.pitch is not None else self.get_parameter('default_spawn_pitch').value
        yaw = request.yaw if request.yaw is not None else self.get_parameter('default_spawn_yaw').value

        self.get_logger().info(f'Received request to spawn robot: {robot_name} ({robot_type}) at ({x}, {y}, {z})')

        try:
            # Load robot model from file or predefined models
            robot_model = self._load_robot_model(robot_type)
            if robot_model is None:
                response.success = False
                response.message = f'Failed to load model for robot type: {robot_type}'
                return response

            # Create pose
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            # Convert roll, pitch, yaw to quaternion
            # Simple conversion for now - in real implementation, use tf2 for proper conversion
            from math import sin, cos
            cy = cos(yaw * 0.5)
            sy = sin(yaw * 0.5)
            cp = cos(pitch * 0.5)
            sp = sin(pitch * 0.5)
            cr = cos(roll * 0.5)
            sr = sin(roll * 0.5)

            pose.orientation.w = cr * cp * cy + sr * sp * sy
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy

            # Create spawn request
            spawn_request = SpawnEntity.Request()
            spawn_request.name = robot_name
            spawn_request.xml = robot_model
            spawn_request.initial_pose = pose

            # Send spawn request
            future = self.spawn_client.call_async(spawn_request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result is not None and result.success:
                self.spawned_entities[robot_name] = {
                    'type': robot_type,
                    'pose': pose,
                    'timestamp': time.time()
                }

                response.success = True
                response.message = f'Successfully spawned robot {robot_name}'

                # Publish status update
                status_msg = String()
                status_msg.data = f"Spawned robot {robot_name} of type {robot_type}"
                self.spawn_status_pub.publish(status_msg)

                self.get_logger().info(f'Successfully spawned robot {robot_name}')
            else:
                response.success = False
                response.message = f'Failed to spawn robot {robot_name}: {result.status_message if result else "Unknown error"}'
                self.get_logger().error(f'Failed to spawn robot {robot_name}: {result.status_message if result else "Unknown error"}')

        except Exception as e:
            response.success = False
            response.message = f'Exception during spawning: {str(e)}'
            self.get_logger().error(f'Exception during spawning of {robot_name}: {e}')

        return response

    def delete_robot_callback(self, request, response):
        """Callback for delete robot service"""
        robot_name = request.name

        self.get_logger().info(f'Received request to delete robot: {robot_name}')

        try:
            # Check if entity exists in our tracking
            if robot_name not in self.spawned_entities:
                response.success = False
                response.message = f'Robot {robot_name} not found in spawned entities'
                self.get_logger().warn(f'Robot {robot_name} not found for deletion')
                return response

            # Create delete request
            delete_request = DeleteEntity.Request()
            delete_request.name = robot_name

            # Send delete request
            future = self.delete_client.call_async(delete_request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result is not None and result.success:
                # Remove from tracking
                del self.spawned_entities[robot_name]

                response.success = True
                response.message = f'Successfully deleted robot {robot_name}'

                # Publish status update
                status_msg = String()
                status_msg.data = f"Deleted robot {robot_name}"
                self.spawn_status_pub.publish(status_msg)

                self.get_logger().info(f'Successfully deleted robot {robot_name}')
            else:
                response.success = False
                response.message = f'Failed to delete robot {robot_name}: {result.status_message if result else "Unknown error"}'
                self.get_logger().error(f'Failed to delete robot {robot_name}: {result.status_message if result else "Unknown error"}')

        except Exception as e:
            response.success = False
            response.message = f'Exception during deletion: {str(e)}'
            self.get_logger().error(f'Exception during deletion of {robot_name}: {e}')

        return response

    def _load_robot_model(self, robot_type: str) -> Optional[str]:
        """Load robot model based on type"""
        # In a real implementation, this would load from files or a model database
        # For now, we'll support a few basic types
        if robot_type == 'humanoid':
            # Try to load from the URDF file
            urdf_path = os.path.join(os.path.dirname(__file__), 'models', 'humanoid_robot', 'humanoid_robot.urdf')
            if os.path.exists(urdf_path):
                try:
                    with open(urdf_path, 'r') as f:
                        return f.read()
                except Exception as e:
                    self.get_logger().error(f'Error reading URDF file: {e}')
                    return None
            else:
                self.get_logger().error(f'URDF file not found: {urdf_path}')
                return None
        elif robot_type == 'turtlebot3':
            # Return a basic turtlebot3 model as an example
            return '''<?xml version="1.0" ?>
<robot name="turtlebot3">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>'''
        else:
            self.get_logger().warn(f'Unknown robot type: {robot_type}')
            return None

    def _monitor_entities(self):
        """Monitor spawned entities"""
        for name, info in self.spawned_entities.items():
            self.get_logger().debug(f'Monitoring entity {name}: {info["type"]} at {info["timestamp"]}')

    def get_spawned_entities(self) -> Dict[str, Any]:
        """Get all spawned entities"""
        return self.spawned_entities.copy()

    def entity_exists(self, name: str) -> bool:
        """Check if an entity exists"""
        return name in self.spawned_entities


# Define service interfaces since they're not in standard ROS 2 interfaces
# In a real implementation, these would be defined in .srv files and generated
# For this example, we'll define simple service classes

class SpawnRobot:
    class Request:
        def __init__(self):
            self.name = ""
            self.type = ""
            self.x = None
            self.y = None
            self.z = None
            self.roll = None
            self.pitch = None
            self.yaw = None

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""


class DeleteRobot:
    class Request:
        def __init__(self):
            self.name = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""


def main(args=None):
    rclpy.init(args=args)
    spawner = GazeboRosSpawner()

    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()