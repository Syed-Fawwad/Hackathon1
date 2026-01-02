# Chapter 1.2: ROS 2 Sensor Integration and Hardware Abstraction

## Overview

This chapter covers the sensor integration framework for the humanoid robot, providing hardware abstraction and standardized interfaces for various sensor types. The implementation enables reliable sensor data acquisition and processing across different hardware platforms.

## Learning Objectives

By the end of this chapter, you will:
- Understand hardware abstraction for sensor integration
- Implement sensor drivers with standardized interfaces
- Configure sensor calibration procedures
- Set up sensor data topics and transforms
- Use sensor services and actions for calibration

## Core Components

### Sensor Driver Framework

The sensor driver framework provides a standardized approach to interfacing with various hardware sensors:

- **SensorInterface**: Abstract base class defining common sensor operations
- **HardwareAbstractionLayer**: Unified interface to different sensor hardware
- **BaseSensorDriver**: Base class with common functionality for all sensors
- **SensorDriverManager**: Orchestrates multiple sensor drivers

### Hardware Interface Node

The hardware interface node provides a standardized abstraction layer for hardware devices:

- **HardwareDevice**: Abstract representation of hardware devices
- **DeviceConfig**: Configuration management for hardware devices
- **Connection Management**: Handles device connection and disconnection
- **Status Monitoring**: Tracks device health and calibration status

### Calibration System

The calibration system ensures accurate sensor data through systematic calibration procedures:

- **CalibrationNode**: Manages calibration processes for various sensors
- **CalibrationData**: Data structure for storing calibration results
- **Accuracy Tracking**: Monitors calibration quality over time
- **Auto-calibration**: Automatic recalibration when accuracy degrades

## Implementation

### Sensor Driver Implementation

The sensor driver framework implements standardized interfaces for common sensor types:

#### Camera Sensors
Located in `book/src/ros2_examples/sensor_drivers/sensor_driver_node.py`, camera drivers provide:
- Raw image data acquisition via `/camera/image_raw`
- Configurable resolution and frame rate parameters
- Hardware abstraction for different camera models

#### LIDAR Sensors
LIDAR drivers implement:
- Scan data acquisition via `/scan`
- Range and intensity processing
- Hardware abstraction for different LIDAR models

#### IMU Sensors
IMU drivers provide:
- Inertial measurement data via `/imu/data`
- Orientation, angular velocity, and linear acceleration
- Calibration for bias and scale factors

### Sensor Topics Configuration

The system uses standardized topics for sensor data:

- **`/imu/data`**: IMU sensor readings with orientation, angular velocity, and linear acceleration
- **`/camera/image_raw`**: Raw camera image data from visual sensors
- **`/tf`**: Dynamic transforms between coordinate frames
- **`/tf_static`**: Static transforms between coordinate frames

### Sensor Services

Two critical services enable sensor management:

- **`/calibrate_sensor`**: Performs calibration for a specific sensor
- **`/get_calibration`**: Retrieves calibration data for a specific sensor

### Sensor Action

The robot calibration action provides comprehensive calibration:

- **`/calibrate_robot`**: Performs systematic calibration of multiple sensors with progress feedback

### Configuration and Management

The system includes comprehensive configuration management in `sensor_topics_config.yaml`:
- QoS profiles for different sensor data types
- Coordinate frame definitions
- Hardware device configurations
- Calibration parameters and thresholds

This sensor integration framework provides the essential hardware abstraction layer needed for reliable sensor operation in the humanoid robot system, enabling consistent behavior across different hardware platforms.