# Chapter 1.3: ROS 2 Action and Service Architecture

## Overview

This chapter covers the action and service architecture for the humanoid robot, providing robust request/response and goal-based communication patterns. The implementation enables complex robot behaviors through well-defined service interfaces and action-based workflows.

## Learning Objectives

By the end of this chapter, you will:
- Understand service-based request/response communication patterns
- Implement action-based goal-oriented behaviors
- Design state machine control for complex behaviors
- Configure action feedback and status reporting
- Create sophisticated behavior management systems

## Core Components

### Service Server Architecture

The service server architecture provides synchronous request/response communication for operations requiring confirmation or results:

- **ServiceServerNode**: Centralized service management for robot operations
- **Request/Response Patterns**: Standardized interfaces for robot commands
- **State Management**: Track robot state through service interactions
- **Error Handling**: Proper error reporting and recovery mechanisms

### Action Server Architecture

The action server architecture provides goal-based communication with feedback for long-running operations:

- **ActionServerNode**: High-level behavior management through actions
- **Goal Management**: Handle complex, multi-step robot behaviors
- **Feedback Systems**: Real-time progress reporting during execution
- **Cancelation Support**: Ability to interrupt long-running actions

### Behavior Management System

The behavior management system coordinates complex robot behaviors using state machine control:

- **BehaviorManager**: Centralized management of robot behaviors
- **StateMachine**: State-based control for behavior execution
- **Priority Scheduling**: Execute behaviors based on priority levels
- **Status Tracking**: Monitor behavior execution in real-time

## Implementation

### Service Server Implementation

The service server framework implements various request/response patterns:

#### Robot Status Services
Located in `book/src/ros2_examples/action_servers/service_server_node.py`, status services provide:
- **`/get_robot_status`**: Retrieve current operational status, mode, and battery level
- **`/set_robot_mode`**: Change robot operational mode with validation

#### Robot Control Services
Control services enable precise robot control:
- **`/get_robot_pose`**: Retrieve current robot pose in the specified frame
- **`/set_robot_pose`**: Set robot pose with frame specification
- **`/get_joint_states`**: Get current joint positions, velocities, and efforts
- **`/set_joint_positions`**: Command joint positions with validation

#### Action Management Services
Action management services coordinate goal-based operations:
- **`/cancel_goal`**: Cancel a specific running goal by ID
- **`/get_result`**: Retrieve results of completed requests

### Action Server Implementation

The action server framework implements goal-oriented behaviors:

#### Navigation Actions
Navigation actions enable robot movement:
- **`/navigate_to_pose`**: Navigate robot to specified pose with progress feedback
- **`/localize_robot`**: Perform robot localization with feedback

#### Manipulation Actions
Manipulation actions control robot manipulation capabilities:
- **`/pick_and_place`**: Execute pick and place operation with detailed feedback
- **`/grasp_object`**: Execute grasping action with progress tracking

#### Communication Actions
Communication actions handle human-robot interaction:
- **`/speech_recognition`**: Perform speech recognition with listening progress
- **`/speech_synthesis`**: Generate speech output with completion tracking

### Behavior Management Implementation

The behavior management system coordinates complex behaviors:

#### State Machine Control
Located in `book/src/ros2_examples/action_servers/behavior_manager.py`, the state machine provides:
- **State Transitions**: Manage behavior state changes (idle, running, paused, completed, failed)
- **Callback Management**: Execute callbacks on state transitions
- **Priority Scheduling**: Execute behaviors based on priority levels
- **Status Monitoring**: Track behavior execution in real-time

#### Behavior Registration
The system supports:
- **Dynamic Registration**: Register new behaviors at runtime
- **Parameter Management**: Pass parameters to behaviors
- **Timeout Handling**: Automatic behavior timeout and cleanup
- **Result Processing**: Capture and process behavior results

### Action Topics Configuration

The system uses standardized topics for action communication:

- **`/action_feedback`**: Real-time feedback messages from action execution
- **`/action_status`**: Status updates for action execution
- **`/behavior_status`**: Status updates for behavior execution
- **`/state_machine_status`**: Status updates for state machine states

### Configuration and Management

The system includes comprehensive configuration management in `action_topics_config.yaml`:
- QoS profiles for different action data types
- Default parameters for action execution
- Maximum active behavior limits
- Logging and monitoring settings

This action and service architecture provides the sophisticated communication framework needed for complex robot behaviors, enabling reliable execution of long-running operations with proper feedback and control mechanisms.