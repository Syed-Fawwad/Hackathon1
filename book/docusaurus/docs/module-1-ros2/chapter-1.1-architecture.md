# Chapter 1.1: ROS 2 Architecture and Node Communication

## Overview

This chapter establishes the foundational communication framework for the humanoid robot, enabling reliable inter-process communication between perception, planning, and control systems using ROS 2's distributed architecture.

## Learning Objectives

By the end of this chapter, you will:
- Understand the ROS 2 communication architecture
- Create basic ROS 2 nodes for robot communication
- Implement topic-based communication patterns
- Set up services for request/response interactions
- Configure action servers for goal-based communication

## Prerequisites

- Basic understanding of ROS 2 concepts
- ROS 2 Humble Hawksbill installed
- Python or C++ development environment

## Core Concepts

### Nodes
ROS 2 nodes are the fundamental execution units that perform computation. Each node in the robotic nervous system has a specific role:

- **perception_node**: Processes sensor data and creates perception results
- **planning_node**: Plans actions and paths based on current state and goals
- **control_node**: Executes low-level control commands to actuate the robot
- **sensor_fusion_node**: Combines data from multiple sensors for coherent understanding

### Topics
Topics enable asynchronous, one-way communication between nodes. The core topics for our system include:

- `/sensor_data`: Sensor readings from all robot sensors
- `/robot_state`: Current robot state including position and orientation
- `/cmd_vel`: Velocity commands for robot base control
- `/joint_states`: Current joint positions and velocities

### Services
Services provide synchronous, request/response communication for operations that require confirmation or results:

- `/get_robot_status`: Retrieve current robot operational status
- `/set_robot_mode`: Change robot operational mode

### Actions
Actions support goal-based communication with feedback for long-running operations:

- `/move_to_pose`: Navigate robot to specified pose
- `/grasp_object`: Execute grasping action on specified object

## Implementation

The ROS 2 communication framework has been implemented with the following components:

### Communication Framework
The `CommunicationFrameworkNode` serves as the base class for all communication nodes, providing standardized interfaces for the core topics:
- `/sensor_data`: For sensor readings and processed information
- `/robot_state`: For current robot state including position and orientation
- `/cmd_vel`: For velocity commands to control robot base movement
- `/joint_states`: For current joint positions and velocities

### Node Implementations

#### Perception Node
Located in `book/src/ros2_examples/node_framework/perception_node.py`, this node:
- Processes sensor data from various sources (cameras, LIDAR, etc.)
- Creates perception results from raw sensor inputs
- Implements interfaces for camera images, point clouds, and laser scans
- Publishes processed information to `/perception_results`

#### Planning Node
Located in `book/src/ros2_examples/node_framework/planning_node.py`, this node:
- Plans actions and paths based on current state and goals
- Subscribes to goal commands and perception results
- Generates plans and publishes them to `/plan_results`
- Publishes planned paths to `/planned_path`

#### Control Node
Located in `book/src/ros2_examples/node_framework/control_node.py`, this node:
- Executes low-level control commands to actuate the robot
- Processes velocity commands from the planning system
- Manages joint position control and motor commands
- Publishes control status information

#### Sensor Fusion Node
Located in `book/src/ros2_examples/node_framework/sensor_fusion_node.py`, this node:
- Combines data from multiple sensors for coherent understanding
- Integrates IMU, joint states, odometry, and perception data
- Creates fused sensor information for improved accuracy
- Publishes fused data to `/fused_sensor_data`

### Service Implementations
Two critical services have been implemented in `book/src/ros2_examples/node_framework/services/robot_services.py`:

- **`/get_robot_status`**: Provides current operational status, mode, battery level, and error codes
- **`/set_robot_mode`**: Allows changing robot operational modes (idle, autonomous, manual, emergency, etc.)

### Action Implementations
Two primary actions have been implemented in `book/src/ros2_examples/node_framework/actions/robot_actions.py`:

- **`/move_to_pose`**: Navigates the robot to a specified pose with progress feedback
- **`/grasp_object`**: Executes grasping action on a specified object with status updates

### Configuration and Launch
The framework includes:
- Topic configuration in `topics_config.yaml`
- Parameter management system for runtime configuration
- Lifecycle management for proper node initialization and shutdown
- Launch files for system initialization in the `launch` directory

This communication framework forms the "nervous system" of the humanoid robot, enabling reliable inter-process communication between all subsystems.