---
title: Troubleshooting Common Setup Issues
description: Solutions to common problems encountered during setup of Physical AI & Humanoid Robotics environment.
sidebar_position: 17
---

# Troubleshooting Common Setup Issues

This section provides solutions to common problems encountered during the setup of your Physical AI & Humanoid Robotics development environment. Use this guide when you encounter issues during installation or operation.

## General Troubleshooting Approach

When encountering issues, follow this systematic approach:

1. **Identify the problem**: Clearly define what is not working
2. **Check system requirements**: Verify your hardware and software meet requirements
3. **Review logs**: Examine error messages and log files
4. **Search for solutions**: Look for similar issues in documentation or community forums
5. **Test fixes incrementally**: Apply solutions one at a time
6. **Verify the fix**: Confirm the issue is resolved

## Common Ubuntu and System Issues

### Issue: Boot problems after Ubuntu installation
**Symptoms**: Computer won't boot or gets stuck at manufacturer logo
**Solutions**:
- Disable Secure Boot in BIOS/UEFI settings
- Ensure CSM/Legacy Boot is disabled if using UEFI
- Check that Ubuntu was installed in the correct mode (UEFI vs Legacy)

### Issue: No internet connection
**Symptoms**: Cannot connect to internet during installation
**Solutions**:
- Try different network cables or ports
- Use mobile hotspot as temporary solution
- Install drivers manually after connecting to internet

### Issue: Low disk space during installation
**Symptoms**: Installation fails due to insufficient space
**Solutions**:
- Free up space on your drive before installation
- Ensure at least 100GB of free space for full development environment
- Consider using an external drive for large simulation assets

## NVIDIA GPU Driver Issues

### Issue: Black screen after NVIDIA driver installation
**Symptoms**: System boots to black screen with no display
**Solutions**:
1. Boot into recovery mode (hold Shift during boot)
2. Select "Enable networking"
3. Open terminal and run:
   ```bash
   sudo apt purge nvidia-* 
   sudo apt install nvidia-driver-535
   sudo reboot
   ```

### Issue: `nvidia-smi` command not found
**Symptoms**: Command 'nvidia-smi' not found
**Solutions**:
- Verify NVIDIA GPU is properly installed in hardware
- Reinstall NVIDIA drivers:
  ```bash
  sudo apt update
  sudo apt install nvidia-driver-535
  sudo reboot
  ```

### Issue: GPU not detected by CUDA
**Symptoms**: CUDA applications fail to detect GPU
**Solutions**:
- Check if GPU is properly seated in PCIe slot
- Verify drivers are up to date
- Run: `nvidia-smi` to confirm GPU detection
- Check if secure boot is disabled in BIOS

## ROS 2 Installation Issues

### Issue: ROS 2 packages not found
**Symptoms**: Commands like `ros2` not found after installation
**Solutions**:
1. Verify installation:
   ```bash
   echo $ROS_DISTRO
   ```
2. If empty, source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Permanently add to your shell:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Issue: rosdep initialization error
**Symptoms**: Error during `sudo rosdep init`
**Solutions**:
1. Check internet connection
2. Try alternative mirror:
   ```bash
   sudo rosdep init
   rosdep update --rosdistro humble
   ```
3. If still failing, manually create the file:
   ```bash
   sudo mkdir -p /etc/ros/rosdep/sources.list.d
   sudo rosdep init
   rosdep update
   ```

### Issue: Package installation fails
**Symptoms**: `apt install` fails with dependency errors
**Solutions**:
1. Update package lists:
   ```bash
   sudo apt update
   ```
2. Fix broken packages:
   ```bash
   sudo apt --fix-broken install
   ```
3. Clean package cache:
   ```bash
   sudo apt clean
   sudo apt autoclean
   ```

## Gazebo Installation Issues

### Issue: Gazebo fails to launch
**Symptoms**: `gz sim` command fails or shows no window
**Solutions**:
1. Check for missing dependencies:
   ```bash
   sudo apt install --reinstall gz-harmonic
   ```
2. Check graphics drivers:
   ```bash
   glxinfo | grep -i opengl
   ```
3. Try with software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```

### Issue: Gazebo runs very slowly
**Symptoms**: Low frame rate, laggy simulation
**Solutions**:
1. Check GPU usage: `nvidia-smi`
2. Reduce simulation complexity
3. Close other GPU-intensive applications
4. Verify you're using hardware acceleration

## Isaac SDK Issues

### Issue: Isaac ROS packages not found
**Symptoms**: Cannot import Isaac ROS packages in Python
**Solutions**:
1. Verify installation:
   ```bash
   ros2 pkg list | grep isaac
   ```
2. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Check if packages were installed correctly:
   ```bash
   sudo apt list --installed | grep isaac
   ```

### Issue: Isaac Sim installation problems
**Symptoms**: Cannot install or run Isaac Sim
**Solutions**:
1. Verify NVIDIA Developer account and access
2. Check system requirements (RTX GPU required)
3. Follow NVIDIA's official installation guide
4. Ensure Omniverse system requirements are met

## Python and Development Environment Issues

### Issue: Python version conflicts
**Symptoms**: Wrong Python version being used
**Solutions**:
1. Check Python version:
   ```bash
   python3 --version
   ```
2. Set Python 3.11 as default:
   ```bash
   sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1
   ```
3. Create virtual environment:
   ```bash
   python3 -m venv ~/ros2_env
   source ~/ros2_env/bin/activate
   ```

### Issue: pip installation problems
**Symptoms**: pip fails to install packages
**Solutions**:
1. Upgrade pip:
   ```bash
   python3 -m pip install --upgrade pip
   ```
2. Use user flag:
   ```bash
   python3 -m pip install --user package_name
   ```
3. Check for conflicting packages

## Network and Communication Issues

### Issue: ROS 2 nodes can't communicate
**Symptoms**: Nodes on same machine can't communicate
**Solutions**:
1. Check ROS 2 domain ID:
   ```bash
   echo $ROS_DOMAIN_ID
   ```
2. Ensure both nodes are on same domain
3. Check firewall settings
4. Verify network configuration

### Issue: High latency in robot control
**Symptoms**: Delayed response from robot
**Solutions**:
1. Check network bandwidth
2. Reduce data transmission rates
3. Use wired connection instead of Wi-Fi
4. Optimize code for performance

## Performance Issues

### Issue: High CPU usage
**Symptoms**: System becomes unresponsive during simulation
**Solutions**:
1. Monitor with `htop` to identify processes
2. Reduce simulation complexity
3. Close unnecessary applications
4. Check for infinite loops in code

### Issue: High memory usage
**Symptoms**: System runs out of memory during operation
**Solutions**:
1. Monitor with `htop` or `free -h`
2. Implement proper memory management
3. Reduce simulation complexity
4. Increase swap space if needed

## Verification and Testing Issues

### Issue: Test nodes don't communicate
**Symptoms**: `talker` and `listener` don't work
**Solutions**:
1. Verify ROS 2 is sourced
2. Check if both terminals are in same ROS domain
3. Verify no firewall blocking communication
4. Check ROS 2 installation with:
   ```bash
   ros2 doctor
   ```

## Recovery and System Restoration

### Issue: System becomes unstable after multiple installations
**Symptoms**: Random crashes, performance degradation
**Solutions**:
1. Clean package cache: `sudo apt clean`
2. Remove unused packages: `sudo apt autoremove`
3. Check system logs: `journalctl -xe`
4. Consider reinstalling if issues persist

## Getting Help

If you encounter issues not covered in this guide:

1. Check the official documentation:
   - [ROS 2 Documentation](https://docs.ros.org/en/humble/)
   - [Gazebo Documentation](https://gazebosim.org/docs)
   - [NVIDIA Isaac Documentation](https://nvidia-isaac-ros.github.io/)

2. Search community forums:
   - ROS Answers
   - NVIDIA Developer Forums
   - Gazebo Community

3. Review the logs in `/var/log/` or `~/.ros/log/`

4. Create a minimal example that reproduces the issue

Remember: Most installation issues are related to system requirements, dependencies, or environment configuration. Take time to carefully follow the installation guides and verify each step before proceeding.