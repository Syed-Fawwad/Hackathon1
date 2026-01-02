# ROS 2 Sensor Driver Framework

This directory contains the sensor driver framework that provides hardware abstraction for various sensors used in the humanoid robot system.

## Architecture

The framework follows a modular design with the following components:

- **SensorInterface**: Abstract base class defining the common interface for all sensors
- **HardwareAbstractionLayer**: Provides unified access to different hardware implementations
- **BaseSensorDriver**: Base class for all sensor drivers with common functionality
- **SensorDriverManager**: Manages multiple sensor drivers and coordinates their operation

## Supported Sensors

The framework currently includes drivers for:

- **Camera**: For visual data acquisition
- **LIDAR**: For distance measurement and environment mapping
- **IMU**: For orientation and acceleration sensing

## Features

- Hardware abstraction for easy sensor replacement
- Standardized parameter configuration
- QoS configuration for reliable data transmission
- Error handling and graceful degradation
- Runtime enabling/disabling of sensors

## Usage

```bash
# Run the sensor driver framework
ros2 run sensor_drivers sensor_driver_framework
```

This framework provides the essential hardware abstraction layer needed for reliable sensor integration in the humanoid robot system.