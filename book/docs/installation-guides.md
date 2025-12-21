---
title: Installation Guides
description: Step-by-step setup instructions for all required components in Physical AI & Humanoid Robotics.
sidebar_position: 16
---

# Installation Guides

This section provides detailed, step-by-step instructions for installing all required components to implement the Physical AI & Humanoid Robotics concepts covered in this book. Follow these guides in sequence to properly set up your development environment.

## Prerequisites

Before beginning the installation process, ensure you have:

- A computer running Ubuntu 22.04 LTS
- Administrative (sudo) access to your system
- Stable internet connection
- At least 100GB of free disk space (varies based on optional components)
- NVIDIA GPU with RTX 4070 Ti or higher (for simulation)

## 1. Ubuntu 22.04 LTS Setup

### Option A: Fresh Installation
1. Download Ubuntu 22.04 LTS ISO from [ubuntu.com](https://releases.ubuntu.com/jammy/)
2. Create a bootable USB drive using:
   - Rufus (Windows)
   - Etcher (Cross-platform)
   - Startup Disk Creator (Ubuntu)
3. Boot from the USB drive and follow the installation wizard
4. During installation, ensure you:
   - Select "Install third-party software for graphics and Wi-Fi hardware"
   - Set up a user account with a strong password

### Option B: Dual Boot Setup
1. Shrink your existing partition to make space for Ubuntu (at least 50GB)
2. Follow the same steps as Option A but select "Install Ubuntu alongside [existing OS]"

## 2. System Updates and Basic Tools

After installing Ubuntu, update your system and install basic tools:

```bash
# Update package lists
sudo apt update && sudo apt upgrade -y

# Install basic development tools
sudo apt install -y build-essential cmake git curl wget vim htop
```

## 3. NVIDIA GPU Drivers Installation

1. Check if your system detects the NVIDIA GPU:
   ```bash
   lspci | grep -i nvidia
   ```

2. Install NVIDIA drivers:
   ```bash
   sudo apt install -y nvidia-driver-535 nvidia-utils-535
   ```

3. Reboot your system:
   ```bash
   sudo reboot
   ```

4. Verify the installation:
   ```bash
   nvidia-smi
   ```

## 4. ROS 2 Humble Hawksbill Installation

1. Set up your sources.list:
   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   ```

2. Add the ROS 2 GPG key:
   ```bash
   sudo apt update && sudo apt install -y curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

3. Add the repository to your sources list:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. Install ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   sudo apt install -y ros-humble-ros-base
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

5. Initialize rosdep:
   ```bash
   sudo rosdep init
   rosdep update
   ```

6. Source the ROS 2 environment:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## 5. Python 3.11 Setup

ROS 2 Humble requires Python 3.10 or 3.11:

```bash
# Install Python 3.11 if not already installed
sudo apt install -y python3.11 python3.11-dev python3.11-venv python3-pip

# Set Python 3.11 as default for this session
alias python3=python3.11
```

## 6. Colcon Build Tool Installation

```bash
sudo apt install -y python3-colcon-common-extensions python3-colcon-mixin
```

## 7. Gazebo Installation

1. Install Gazebo Garden:
   ```bash
   wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
   sudo cp /tmp/gazebo.gpg /usr/share/keyrings/
   echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   sudo apt update
   sudo apt install gz-harmonic
   ```

2. Verify installation:
   ```bash
   gz sim
   ```

## 8. NVIDIA Isaac SDK Installation

1. Install prerequisites:
   ```bash
   sudo apt install -y python3-pip python3-dev python3-venv
   pip3 install --user setuptools
   ```

2. Install Isaac ROS dependencies:
   ```bash
   sudo apt install -y ros-humble-isaac-ros-gems
   sudo apt install -y ros-humble-isaac-ros-common
   ```

3. For Isaac Sim, follow the installation guide from NVIDIA's official documentation as it requires an NVIDIA Developer account.

## 9. Development Environment Setup

1. Install VS Code:
   ```bash
   wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
   sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
   sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
   rm -f packages.microsoft.gpg
   sudo apt install -y apt-transport-https
   sudo apt update
   sudo apt install -y code
   ```

2. Install ROS 2 extensions for VS Code:
   - Open VS Code
   - Go to Extensions (Ctrl+Shift+X)
   - Install "ROS" extension by xaver

## 10. Testing Your Installation

1. Create a test workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. Test basic ROS 2 functionality:
   ```bash
   # In one terminal
   ros2 run demo_nodes_cpp talker
   
   # In another terminal
   ros2 run demo_nodes_py listener
   ```

3. Test Gazebo:
   ```bash
   gz sim shapes.sdf
   ```

## 11. Optional: Docker Setup (For Isolated Development)

1. Install Docker:
   ```bash
   sudo apt install -y docker.io
   sudo usermod -aG docker $USER
   ```

2. Log out and back in for group changes to take effect

3. Verify Docker installation:
   ```bash
   docker run hello-world
   ```

## 12. Setting Up Your First ROS 2 Package

1. Navigate to your workspace:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package:
   ```bash
   ros2 pkg create --build-type ament_python my_robot_controller --dependencies rclpy std_msgs geometry_msgs sensor_msgs
   ```

3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_controller
   source install/setup.bash
   ```

## Verification Checklist

After completing all installation steps, verify your setup with this checklist:

- [ ] Ubuntu 22.04 LTS is running
- [ ] NVIDIA drivers are properly installed (`nvidia-smi` works)
- [ ] ROS 2 Humble is installed and sourced
- [ ] `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener` communicate successfully
- [ ] Gazebo runs without errors (`gz sim`)
- [ ] Python 3.11 is available
- [ ] Colcon builds packages without errors
- [ ] Isaac ROS packages are installed
- [ ] Development environment is set up

If all items are checked, your system is ready for the Physical AI & Humanoid Robotics curriculum!