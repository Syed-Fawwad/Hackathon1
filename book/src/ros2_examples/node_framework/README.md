# ROS 2 Node Communication Framework

This directory contains the foundational communication framework for the humanoid robot, enabling reliable inter-process communication between perception, planning, and control systems using ROS 2's distributed architecture.

## Core Components

### CommunicationFrameworkNode
Base class that provides common ROS 2 communication patterns and handles the primary robot topics:
- `/sensor_data`: Sensor readings from all robot sensors
- `/robot_state`: Current robot state including position and orientation
- `/cmd_vel`: Velocity commands for robot base control
- `/joint_states`: Current joint positions and velocities

### Specialized Nodes
- `PerceptionNode`: Processes sensor data and creates perception results
- `PlanningNode`: Plans actions and paths based on current state and goals
- `ControlNode`: Executes low-level control commands to actuate the robot
- `SensorFusionNode`: Combines data from multiple sensors for coherent understanding

## Communication Patterns

The framework implements all three primary ROS 2 communication patterns:
- **Topics**: Asynchronous, one-way communication for continuous data streams
- **Services**: Synchronous request/response for operations requiring confirmation
- **Actions**: Goal-based communication with feedback for long-running operations

## Usage

```bash
# Run the complete communication framework
ros2 run node_framework communication_framework
```

This framework serves as the "nervous system" of the humanoid robot, providing the essential communication infrastructure for all subsequent modules.