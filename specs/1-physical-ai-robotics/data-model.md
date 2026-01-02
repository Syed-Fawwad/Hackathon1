# Data Model: Physical AI & Humanoid Robotics

**Created**: 2026-01-01
**Feature**: Physical AI & Humanoid Robotics
**Plan**: [plan.md](plan.md)

## Overview

This document defines the key data structures and message formats used in the Physical AI & Humanoid Robotics system. It captures the entities, their relationships, and validation rules derived from the functional requirements.

## Core System Entities

### 1. RobotState
**Description**: Represents the current state of the robot including position, orientation, joint states, and sensor readings.

**Fields**:
- `timestamp`: Time of state measurement (std_msgs/Header)
- `position`: 3D position in world coordinates (geometry_msgs/Point)
- `orientation`: 3D orientation (geometry_msgs/Quaternion)
- `joint_states`: Array of joint positions, velocities, efforts (sensor_msgs/JointState)
- `sensor_data`: Collection of current sensor readings (sensor_msgs/MultiDOFJointState)
- `battery_level`: Current battery percentage (float)
- `operational_mode`: Current operational mode (string: "idle", "navigation", "manipulation", "emergency")

**Relationships**:
- One-to-Many with SensorReading (contains multiple sensor readings)
- One-to-One with RobotHealth (current health status)

**Validation Rules**:
- Position coordinates must be within valid range
- Orientation must be normalized quaternion
- Joint positions within physical limits
- Battery level between 0-100%

### 2. SensorReading
**Description**: Represents a single sensor reading from various robot sensors.

**Fields**:
- `sensor_type`: Type of sensor (string: "camera", "lidar", "imu", "joint_encoder", etc.)
- `sensor_id`: Unique identifier for the sensor (string)
- `timestamp`: Time of reading (builtin_interfaces/Time)
- `data`: Sensor-specific data (varies by sensor type)
- `frame_id`: Coordinate frame of the sensor (string)
- `quality_score`: Quality metric of the reading (float 0.0-1.0)

**Relationships**:
- Many-to-One with RobotState (contained within robot state)
- One-to-Many with PerceptionResult (processed by perception algorithms)

**Validation Rules**:
- Timestamp must be recent (within 1 second)
- Quality score must be between 0.0 and 1.0
- Data format must match sensor type

### 3. PerceptionResult
**Description**: Output from perception algorithms including object detection, localization, and scene understanding.

**Fields**:
- `timestamp`: Time of perception result (builtin_interfaces/Time)
- `detected_objects`: Array of detected objects (object_recognition_msgs/RecognizedObjectArray)
- `pose_estimation`: Robot's estimated pose (geometry_msgs/PoseWithCovarianceStamped)
- `occupancy_grid`: Map of environment (nav_msgs/OccupancyGrid)
- `confidence_score`: Overall confidence in perception (float 0.0-1.0)
- `processing_time`: Time taken for perception (float in seconds)

**Relationships**:
- Many-to-One with SensorReading (processed from sensor data)
- One-to-Many with NavigationGoal (used for path planning)

**Validation Rules**:
- Confidence score must be between 0.0 and 1.0
- Processing time must be positive
- Detected objects must have valid bounding boxes

### 4. NavigationGoal
**Description**: Represents a navigation goal with path planning and execution information.

**Fields**:
- `goal_id`: Unique identifier for the goal (string)
- `target_pose`: Target position and orientation (geometry_msgs/PoseStamped)
- `path`: Planned path as waypoints (nav_msgs/Path)
- `planning_time`: Time taken to plan the path (float in seconds)
- `execution_status`: Current execution status (string: "pending", "executing", "completed", "failed")
- `estimated_time`: Estimated time to reach goal (float in seconds)
- `obstacle_avoidance`: Whether obstacle avoidance is active (bool)

**Relationships**:
- Many-to-One with PerceptionResult (based on environment understanding)
- One-to-Many with RobotCommand (results in movement commands)

**Validation Rules**:
- Target pose must be within navigable area
- Path must be valid with connected waypoints
- Planning time must be positive

### 5. RobotCommand
**Description**: Commands sent to the robot's control system for execution.

**Fields**:
- `command_type`: Type of command (string: "move_base", "joint_trajectory", "gripper_control", "stop")
- `target_values`: Target positions/speeds for actuators (varies by command type)
- `execution_time`: Time by which command should be completed (builtin_interfaces/Time)
- `priority`: Priority level (int 1-10, 10 being highest)
- `timeout`: Timeout for command execution (float in seconds)
- `feedback_topic`: Topic for command execution feedback (string)

**Relationships**:
- Many-to-One with NavigationGoal (derived from navigation goal)
- One-to-Many with RobotState (affects robot state)

**Validation Rules**:
- Target values must be within actuator limits
- Priority must be between 1-10
- Timeout must be positive

### 6. VLACommand
**Description**: Voice-Language-Action command that translates natural language to robot actions.

**Fields**:
- `voice_input`: Original voice input (string)
- `transcribed_text`: Transcribed text from speech (string)
- `parsed_intent`: Parsed intent from NLP (string)
- `visual_context`: Relevant visual context (sensor_msgs/Image)
- `generated_action`: Generated robot action (RobotCommand)
- `confidence`: Confidence in interpretation (float 0.0-1.0)
- `execution_status`: Status of action execution (string)

**Relationships**:
- One-to-One with RobotCommand (generates robot command)
- One-to-Many with PerceptionResult (uses visual context)

**Validation Rules**:
- Confidence must be between 0.0 and 1.0
- Generated action must be valid RobotCommand
- Voice input must not be empty

## System State Entities

### 7. RobotHealth
**Description**: Health and diagnostic information for the robot system.

**Fields**:
- `system_status`: Overall system status (string: "healthy", "warning", "error", "critical")
- `component_health`: Health status of individual components (dictionary)
- `cpu_usage`: CPU usage percentage (float)
- `memory_usage`: Memory usage percentage (float)
- `gpu_usage`: GPU usage percentage (float)
- `temperature`: Critical component temperatures (dictionary)
- `last_error`: Last error message (string)
- `uptime`: System uptime in seconds (float)

**Relationships**:
- One-to-One with RobotState (current health of robot state)

**Validation Rules**:
- CPU, memory, GPU usage between 0-100%
- Temperatures within safe operating range
- System status must be one of defined values

### 8. CapstoneTask
**Description**: High-level tasks for the capstone autonomous humanoid system.

**Fields**:
- `task_id`: Unique identifier for the task (string)
- `task_description`: Natural language description of task (string)
- `task_type`: Type of task (string: "navigation", "manipulation", "perception", "communication")
- `priority`: Task priority (int 1-10)
- `required_capabilities`: Capabilities needed for task (array of strings)
- `estimated_duration`: Estimated time to complete (float in seconds)
- `current_status`: Current status of task (string: "pending", "planning", "executing", "completed", "failed")
- `subtasks`: Array of subtasks (array of CapstoneTask)

**Relationships**:
- One-to-Many with RobotCommand (results in robot commands)
- One-to-Many with VLACommand (can be triggered by voice command)

**Validation Rules**:
- Priority must be between 1-10
- Task type must be one of defined values
- Subtasks must not create circular dependencies

## Message Flow Validation

### ROS 2 Interface Requirements
Based on the functional requirements from the spec, the following ROS 2 interfaces are defined:

**Topics**:
- `/sensor_data` - SensorReading messages
- `/robot_state` - RobotState messages
- `/cmd_vel` - Velocity commands
- `/joint_states` - Joint state feedback
- `/user_command` - VLACommand inputs
- `/parsed_intent` - Parsed intent outputs
- `/visual_context` - Visual context images
- `/humanoid_state` - Capstone system state

**Services**:
- `/get_robot_status` - RobotHealth queries
- `/set_robot_mode` - RobotState mode changes
- `/process_command` - VLACommand processing
- `/get_context` - Visual context queries
- `/start_capstone` - Capstone task initiation
- `/stop_capstone` - Capstone task termination

**Actions**:
- `/navigate_to_pose` - Navigation goals
- `/grasp_object` - Manipulation tasks
- `/execute_behavior` - Complex behaviors
- `/localize_robot` - Localization requests
- `/execute_vla_plan` - VLA command execution

## Validation Rules Summary

All entities must satisfy these cross-entity validation rules:
- Timestamps must be monotonic and recent
- All required fields must be populated
- Relationships must reference valid entities
- Values must be within specified ranges
- Message formats must comply with ROS 2 standards
- Safety limits must be respected for all commands