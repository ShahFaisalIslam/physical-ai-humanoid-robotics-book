---
title: Hardware Requirements
description: Detailed hardware specifications for workstations, edge platforms, sensors, and robot platforms.
sidebar_position: 13
---

# Hardware Requirements

This section provides detailed specifications for all hardware components needed to implement the concepts covered in this book. These requirements are based on the computational demands of physics simulation, visual perception, and generative AI that characterize Physical AI systems.

## 1. The "Digital Twin" Workstation (Required per Student)

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops (MacBooks or non-RTX Windows machines) **will not work**.

### GPU (The Bottleneck)
- **Minimum**: NVIDIA RTX 4070 Ti (12GB VRAM)
- **Recommended**: RTX 3090 or 4090 (24GB VRAM) for smoother "Sim-to-Real" training
- **Why**: You need high VRAM to load the USD (Universal Scene Description) assets for the robot and environment, plus run the VLA (Vision-Language-Action) models simultaneously.

### CPU
- **Minimum**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **Why**: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive.

### RAM
- **Minimum**: 32 GB DDR5 (absolute minimum, may crash during complex scene rendering)
- **Recommended**: 64 GB DDR5
- **Why**: Simulation environments with multiple robots and sensors require substantial memory.

### OS
- **Required**: Ubuntu 22.04 LTS
- **Note**: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience.

## 2. The "Physical AI" Edge Kit

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the *nervous system* on a desk before deploying it to a robot. This kit covers Module 3 (Isaac ROS) and Module 4 (VLA).

### The Brain
- **Option 1**: NVIDIA Jetson Orin Nano (8GB)
- **Option 2**: NVIDIA Jetson Orin NX (16GB)
- **Role**: Industry standard for embodied AI. Students will deploy their ROS 2 nodes here to understand resource constraints vs. their powerful workstations.

### The Eyes (Vision)
- **Option 1**: Intel RealSense D435i (Recommended)
- **Option 2**: Intel RealSense D455
- **Role**: Provides RGB (Color) and Depth (Distance) data. Essential for the VSLAM and Perception modules.

### The Inner Ear (Balance)
- **Generic USB IMU (BNO055)**: Often built into the RealSense D435i or Jetson boards, but a separate module helps teach IMU calibration.

### Voice Interface
- **USB Microphone/Speaker array**: e.g., ReSpeaker for the "Voice-to-Action" Whisper integration.

## 3. The Robot Lab

For the "Physical" part of the course, you have three tiers of options depending on budget.

### Option A: The "Proxy" Approach (Recommended for Budget)
Use a quadruped (dog) or a robotic arm as a proxy. The software principles (ROS 2, VSLAM, Isaac Sim) transfer 90% effectively to humanoids.
- **Robot**: Unitree Go2 Edu (~$1,800 - $3,000)
- **Pros**: Highly durable, excellent ROS 2 support, affordable enough to have multiple units
- **Cons**: Not a biped (humanoid)

### Option B: The "Miniature Humanoid" Approach
Small, table-top humanoids.
- **Robot**: Unitree H1 is too expensive ($90k+), so look at Unitree G1 (~$16k) or Robotis OP3 (older, but stable, ~$12k)
- **Budget Alternative**: Hiwonder TonyPi Pro (~$600)
- **Warning**: The cheap kits (Hiwonder) usually run on Raspberry Pi, which **cannot** run NVIDIA Isaac ROS efficiently. You would use these only for kinematics (walking) and use the Jetson kits for AI.

### Option C: The "Premium" Lab (Sim-to-Real specific)
If the goal is to actually deploy the Capstone to a real humanoid:
- **Robot**: Unitree G1 Humanoid
- **Why**: It is one of the few commercially available humanoids that can actually walk dynamically and has an SDK open enough for students to inject their own ROS 2 controllers.

## 4. Summary of Architecture

To teach this successfully, your lab infrastructure should look like this:

| Component | Hardware | Function |
| :---- | :---- | :---- |
| **Sim Rig** | PC with RTX 4080 + Ubuntu 22.04 | Runs Isaac Sim, Gazebo, Unity, and trains LLM/VLA models |
| **Edge Brain** | Jetson Orin Nano | Runs the "Inference" stack. Students deploy their code here |
| **Sensors** | RealSense Camera + Lidar | Connected to the Jetson to feed real-world data to the AI |
| **Actuator** | Unitree Go2 or G1 (Shared) | Receives motor commands from the Jetson |

## 5. Alternative: Cloud-Native Lab (High OpEx)

If you do not have access to RTX-enabled workstations, you must restructure the course to rely entirely on cloud-based instances (like AWS RoboMaker or NVIDIA's cloud delivery for Omniverse), though this introduces significant latency and cost complexity.

### Cloud Workstations (AWS/Azure)
Instead of buying PCs, you rent instances.
- **Instance Type**: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge
- **Software**: NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI)

### Local "Bridge" Hardware
You cannot eliminate hardware entirely for "Physical AI." You still need the edge devices to deploy the code physically.
- **Edge AI Kits**: You still need the Jetson Kit for the physical deployment phase (~$700 one-time purchase)
- **Robot**: You still need one physical robot for the final demo (~$3,000 Unitree Go2 Standard)

### The Economy Jetson Student Kit

Best for: Learning ROS 2, Basic Computer Vision, and Sim-to-Real control.

| Component | Model | Price (Approx.) | Notes |
| :---- | :---- | :---- | :---- |
| **The Brain** | **NVIDIA Jetson Orin Nano Super Dev Kit (8GB)** | **$249** | New official MSRP (Price dropped from ~$499). Capable of 40 TOPS |
| **The Eyes** | **Intel RealSense D435i** | **$349** | Includes IMU (essential for SLAM). Do not buy the D435 (non-i) |
| **The Ears** | **ReSpeaker USB Mic Array v2.0** | **$69** | Far-field microphone for voice commands (Module 4) |
| **Wi-Fi** | (Included in Dev Kit) | $0 | The new "Super" kit includes the Wi-Fi module pre-installed |
| **Power/Misc** | SD Card (128GB) + Jumper Wires | $30 | High-endurance microSD card required for the OS |
| **TOTAL** | | **~$700 per kit** | |

### The Latency Trap (Hidden Cost)
Simulating in the cloud works well, but *controlling* a real robot from a cloud instance is dangerous due to latency.
- **Solution**: Students train in the Cloud, download the model (weights), and flash it to the local Jetson kit.