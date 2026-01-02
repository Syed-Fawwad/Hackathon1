# Gazebo Simulation Environment

This package contains the Gazebo simulation environment for the humanoid robot project. It includes world files, robot models, and configuration for physics simulation.

## Structure

- `worlds/` - Gazebo world files defining simulation environments
- `models/` - 3D models for robots and objects
- `launch/` - Launch files for starting simulation
- `config/` - Configuration files for simulation parameters

## World Files

- `simple_room.world` - Basic indoor environment with walls and furniture for robot testing

## Running the Simulation

To launch the simulation environment:

```bash
# Launch Gazebo with the simple room world
ros2 launch gazebo_ros gazebo.launch.py world:=simple_room.world
```

This simulation environment provides the foundation for testing robot behaviors in a physics-accurate virtual environment before deployment to real hardware.