# Quickstart Guide: Physical AI & Humanoid Robotics

**Created**: 2026-01-01
**Feature**: Physical AI & Humanoid Robotics
**Plan**: [plan.md](plan.md)

## Overview

This quickstart guide provides the essential steps to set up the development environment for the Physical AI & Humanoid Robotics book. It covers the installation of required tools, basic configuration, and running your first example.

## Prerequisites

### Hardware Requirements
- **Development Workstation**:
  - 64-bit processor (Intel/AMD)
  - 16GB+ RAM (32GB recommended)
  - NVIDIA RTX GPU (4080/4090 recommended) with CUDA support
  - 500GB+ SSD storage
- **Edge Platform** (optional for deployment):
  - NVIDIA Jetson Orin NX Developer Kit
  - Compatible robot platform with ROS 2 support

### Software Requirements
- Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- Git version control system
- Docker and Docker Compose
- NVIDIA GPU drivers (if using GPU acceleration)

## Installation Steps

### 1. Install ROS 2 Humble Hawksbill

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Set Up NVIDIA Isaac ROS Dependencies

```bash
# Install NVIDIA container toolkit
curl -sL https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(./etc/os-release && echo $ID$VERSION_ID)
curl -sL https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker

# Verify GPU access in containers
docker run --rm --gpus all nvidia/cuda:11.8-devel-ubuntu22.04 nvidia-smi
```

### 3. Install Gazebo and Simulation Tools

```bash
# Install Gazebo Garden
sudo apt install gazebo libgazebo-dev
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install robot simulation packages
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-broadcaster ros-humble-velocity-controllers
```

### 4. Set Up Development Workspace

```bash
# Create workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Clone book repository and dependencies
git clone https://github.com/[your-repo]/physical-ai-book.git src/

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
echo "source ~/physical_ai_ws/install/setup.bash" >> ~/.bashrc
source ~/physical_ai_ws/install/setup.bash
```

## Running Your First Example

### 1. Basic ROS 2 Communication Test

```bash
# Terminal 1: Start ROS 2 daemon
source ~/physical_ai_ws/install/setup.bash
ros2 daemon start

# Terminal 2: Run a simple publisher
source ~/physical_ai_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 3: Run a simple subscriber
source ~/physical_ai_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

### 2. Robot Simulation Example

```bash
# Terminal 1: Start Gazebo simulation
source ~/physical_ai_ws/install/setup.bash
ros2 launch physical_ai_examples simple_robot.launch.py

# Terminal 2: Send commands to simulated robot
source ~/physical_ai_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

### 3. Perception Pipeline Test

```bash
# Terminal 1: Start perception nodes
source ~/physical_ai_ws/install/setup.bash
ros2 launch physical_ai_examples perception_pipeline.launch.py

# Terminal 2: View processed images
source ~/physical_ai_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

## Development Environment Setup

### VS Code Configuration

1. Install the [ROS extension pack](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
2. Create `.vscode/settings.json` in your workspace:

```json
{
    "ros.distro": "humble",
    "cmake.cmakePath": "/usr/bin/cmake",
    "terminal.integrated.profiles.linux": {
        "bash": {
            "path": "/bin/bash",
            "args": ["-c", "source /opt/ros/humble/setup.bash && source ~/physical_ai_ws/install/setup.bash && exec bash"]
        }
    }
}
```

### Docker Development Environment

```bash
# Build development container
cd ~/physical_ai_ws
docker build -f docker/development.Dockerfile -t physical-ai-dev .

# Run container with GPU support
docker run --gpus all --rm -it \
  --name physical_ai_dev \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  physical-ai-dev:latest
```

## Verification Steps

### 1. Check ROS 2 Installation
```bash
# Verify ROS 2 installation
ros2 --version

# Check available packages
ros2 pkg list | grep physical_ai
```

### 2. Verify Simulation Environment
```bash
# Check Gazebo installation
gazebo --version

# Test Gazebo with ROS 2 bridge
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
```

### 3. Verify Perception Pipeline
```bash
# Check Isaac ROS packages
ros2 pkg list | grep isaac_ros

# Test camera processing
ros2 run image_view image_view image:=/camera/image_raw
```

## Troubleshooting Common Issues

### GPU Not Detected
- Ensure NVIDIA drivers are properly installed: `nvidia-smi`
- Verify CUDA installation: `nvcc --version`
- Check Docker GPU access: `docker run --gpus all nvidia/cuda:11.8-devel-ubuntu22.04 nvidia-smi`

### ROS 2 Nodes Not Communicating
- Verify same ROS_DOMAIN_ID: `echo $ROS_DOMAIN_ID`
- Check network configuration: `ping` between machines if using multi-robot setup
- Ensure ROS 2 daemon is running: `ros2 daemon status`

### Simulation Performance Issues
- Check GPU memory usage: `nvidia-smi`
- Reduce simulation complexity or quality settings
- Ensure sufficient CPU resources allocated to simulation

## Next Steps

After completing this quickstart, you're ready to:
1. Explore the book modules in sequence (Module 1-4)
2. Run the complete capstone example
3. Modify and extend the provided examples
4. Deploy to real hardware using the Jetson configuration

The complete book content and examples are organized according to the 4-module structure:
- Module 1: ROS 2 communication and architecture
- Module 2: Digital twin simulation with Gazebo and Unity
- Module 3: AI perception and navigation with NVIDIA Isaac
- Module 4: Vision-Language-Action integration for the capstone

Continue with Module 1 examples to build your foundational understanding of the robotic nervous system.