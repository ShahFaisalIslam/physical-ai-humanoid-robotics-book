---
title: Capstone Troubleshooting Guide
description: Troubleshooting guide for common challenges in the capstone project
sidebar_position: 5
---

# Capstone Troubleshooting Guide

## Overview

This guide addresses common challenges and troubleshooting steps for implementing the Autonomous Humanoid Capstone Project. It covers issues across all integrated components: voice processing, navigation, perception, manipulation, and system integration.

## 1. Voice Processing Challenges

### Challenge 1: Poor Speech Recognition Accuracy

**Symptoms:**
- Whisper frequently fails to transcribe speech correctly
- Recognition works well in quiet environments but poorly in robotic settings
- Misunderstanding of domain-specific terms

**Solutions:**
1. **Audio Preprocessing:**
   ```python
   import numpy as np
   import librosa
   
   def preprocess_audio(audio_data, sample_rate=16000):
       # Resample if needed
       if sample_rate != 16000:
           audio_data = librosa.resample(audio_data, orig_sr=sample_rate, target_sr=16000)
       
       # Apply noise reduction
       # (Implement noise reduction algorithm or use library like noisereduce)
       
       # Normalize audio
       audio_data = audio_data / np.max(np.abs(audio_data))
       
       return audio_data
   ```

2. **Microphone Placement:**
   - Position microphone 15-30cm from speaker
   - Use directional microphones to reduce environmental noise
   - Implement acoustic echo cancellation if using speakers

3. **Model Selection:**
   - Try different Whisper model sizes (tiny, base, small, medium, large)
   - For domain-specific terms, consider fine-tuning or using custom language models

### Challenge 2: High Latency in Voice Processing

**Symptoms:**
- Long delay between speaking and robot response
- Poor real-time performance

**Solutions:**
1. **Optimize Model Loading:**
   ```python
   # Load model once at startup, not per request
   class VoiceProcessor:
       def __init__(self):
           self.model = whisper.load_model("base")  # Load once
   ```

2. **Use Streaming Processing:**
   - Implement partial transcription for real-time feedback
   - Use voice activity detection to reduce unnecessary processing

3. **Hardware Acceleration:**
   - Use GPU for Whisper processing if available
   - Consider edge AI accelerators for dedicated speech processing

### Challenge 3: Wake Word Detection Issues

**Symptoms:**
- Robot responds to random sounds
- Fails to detect wake word consistently

**Solutions:**
1. **Use Dedicated Wake Word Detection:**
   ```python
   import pvporcupine
   import pyaudio
   
   class WakeWordDetector:
       def __init__(self, keyword_paths):
           self.porcupine = pvporcupine.create(keyword_paths)
           self.pa = pyaudio.PyAudio()
           self.audio_stream = self.pa.open(
               rate=self.porcupine.sample_rate,
               channels=1,
               format=pyaudio.paInt16,
               input=True,
               frames_per_buffer=self.porcupine.frame_length
           )
       
       def detect_wake_word(self):
           while True:
               pcm = self.audio_stream.read(self.porcupine.frame_length)
               pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)
               
               keyword_index = self.porcupine.process(pcm)
               if keyword_index >= 0:
                   return True  # Wake word detected
   ```

2. **Adjust Sensitivity:**
   - Tune detection thresholds based on environment
   - Use multiple keywords to reduce false positives

## 2. Navigation and Path Planning Challenges

### Challenge 1: Navigation Fails in Dynamic Environments

**Symptoms:**
- Robot gets stuck or takes inefficient paths
- Poor obstacle avoidance
- Localization failures

**Solutions:**
1. **Improve Costmap Configuration:**
   ```yaml
   # In your costmap configuration
   inflation_layer:
     inflation_radius: 0.5  # Adjust based on robot size
     cost_scaling_factor: 5.0  # Higher for more conservative inflation
   
   obstacle_layer:
     observation_sources: laser_scan
     laser_scan: 
       topic: /scan
       max_obstacle_height: 2.0
       clearing: true
       marking: true
   ```

2. **Implement Dynamic Obstacle Tracking:**
   ```python
   class DynamicObstacleHandler:
       def __init__(self):
           self.tracked_objects = {}
           self.last_update_time = {}
       
       def update_tracked_objects(self, detection_data):
           # Update tracked objects with new detection data
           for obj in detection_data:
               obj_id = obj['id']
               self.tracked_objects[obj_id] = {
                   'position': obj['position'],
                   'velocity': self.estimate_velocity(obj_id, obj['position']),
                   'timestamp': time.time()
               }
       
       def estimate_velocity(self, obj_id, new_position):
           # Estimate velocity based on previous positions
           if obj_id in self.tracked_objects:
               prev_pos = self.tracked_objects[obj_id]['position']
               dt = time.time() - self.last_update_time.get(obj_id, time.time())
               if dt > 0:
                   velocity = (new_position - prev_pos) / dt
                   return velocity
           return np.array([0, 0])
   ```

3. **Use Appropriate Global Planner:**
   - For dynamic environments, consider using D* Lite or other replanning algorithms
   - Adjust global planner frequency to account for environment changes

### Challenge 2: Local Minima and Getting Stuck

**Symptoms:**
- Robot gets trapped between obstacles
- Fails to find path out of tight spaces

**Solutions:**
1. **Implement Escape Behaviors:**
   ```python
   class EscapeBehavior:
       def __init__(self, robot_interface):
           self.robot_interface = robot_interface
           self.escape_attempts = 0
       
       def check_stuck_condition(self):
           # Check if robot hasn't made progress in a while
           current_pos = self.robot_interface.get_position()
           if self.has_not_moved_significantly(current_pos):
               return True
           return False
       
       def execute_escape_maneuver(self):
           # Back up and turn
           self.robot_interface.move_backward(0.3)  # Move back 30cm
           self.robot_interface.turn_in_place(90)   # Turn 90 degrees
           self.escape_attempts += 1
   ```

2. **Adjust Local Planner Parameters:**
   - Increase local planner lookahead distance
   - Modify trajectory scoring to favor forward motion
   - Add random sampling to escape local minima

### Challenge 3: Localization Drift

**Symptoms:**
- Robot's estimated position diverges from actual position
- Navigation to known locations fails

**Solutions:**
1. **Improve AMCL Configuration:**
   ```yaml
   # AMCL configuration
   amcl:
     ros__parameters:
       use_map_topic: true
       alpha1: 0.2
       alpha2: 0.2
       alpha3: 0.2
       alpha4: 0.2
       alpha5: 0.2
       base_frame_id: "base_footprint"
       beam_count: 60
       do_beamskip: false
       lambda_short: 0.1
       laser_likelihood_max_dist: 2.0
       laser_max_range: 10.0
       laser_min_range: -1.0
       max_beams: 60
       max_particles: 2000
       min_particles: 500
       odom_frame_id: "odom"
       pf_err: 0.05
       pf_z: 0.99
       recovery_alpha_fast: 0.0
       recovery_alpha_slow: 0.0
       resample_interval: 1
       robot_model_type: "differential"
       save_pose_rate: 0.5
       sigma_hit: 0.2
       tf_broadcast: true
       transform_tolerance: 1.0
       update_min_a: 0.2
       update_min_d: 0.1
   ```

2. **Use Additional Sensors:**
   - Integrate IMU data for better motion prediction
   - Use visual landmarks for global localization
   - Implement loop closure detection

## 3. Perception Challenges

### Challenge 1: Poor Object Detection in Varied Lighting

**Symptoms:**
- Object detection fails under different lighting conditions
- Inconsistent detection results

**Solutions:**
1. **Implement Adaptive Preprocessing:**
   ```python
   import cv2
   
   def adaptive_image_enhancement(image):
       # Convert to LAB color space
       lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
       
       # Apply CLAHE to L channel
       clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
       lab[:,:,0] = clahe.apply(lab[:,:,0])
       
       # Convert back to BGR
       enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
       
       return enhanced
   ```

2. **Use Multiple Detection Models:**
   - Train models on diverse lighting conditions
   - Use ensemble methods to combine multiple models
   - Implement domain adaptation techniques

3. **Add Synthetic Data Training:**
   - Use Gazebo simulation to generate diverse training data
   - Apply domain randomization techniques

### Challenge 2: Slow Detection Performance

**Symptoms:**
- High latency in object detection
- Poor real-time performance

**Solutions:**
1. **Optimize Model Architecture:**
   - Use efficient models like MobileNet-SSD or YOLOv4-tiny
   - Implement model quantization for faster inference
   - Use TensorRT for NVIDIA GPU acceleration

2. **Implement Detection Throttling:**
   ```python
   class ThrottledDetector:
       def __init__(self, detection_interval=0.5):
           self.detection_interval = detection_interval
           self.last_detection_time = 0
       
       def should_detect(self):
           current_time = time.time()
           if current_time - self.last_detection_time > self.detection_interval:
               self.last_detection_time = current_time
               return True
           return False
   ```

3. **Use Multi-threading:**
   - Run detection in separate thread/process
   - Use result caching for consistent performance

### Challenge 3: False Positives and Missed Detections

**Symptoms:**
- Detecting non-existent objects
- Missing objects that are clearly visible

**Solutions:**
1. **Implement Temporal Consistency:**
   ```python
   class TemporalConsistencyFilter:
       def __init__(self, min_observations=3, max_age=5):
           self.tracked_objects = {}
           self.min_observations = min_observations
           self.max_age = max_age
       
       def update_detections(self, new_detections):
           # Update tracked objects
           for detection in new_detections:
               obj_id = self.get_object_id(detection)
               if obj_id in self.tracked_objects:
                   self.tracked_objects[obj_id]['observations'] += 1
                   self.tracked_objects[obj_id]['last_seen'] = time.time()
               else:
                   self.tracked_objects[obj_id] = {
                       'detection': detection,
                       'observations': 1,
                       'first_seen': time.time(),
                       'last_seen': time.time()
                   }
           
           # Remove old objects
           current_time = time.time()
           self.tracked_objects = {
               k: v for k, v in self.tracked_objects.items()
               if current_time - v['last_seen'] < self.max_age
           }
           
           # Return confirmed objects (seen multiple times)
           confirmed = [
               v['detection'] for v in self.tracked_objects.values()
               if v['observations'] >= self.min_observations
           ]
           
           return confirmed
   ```

2. **Adjust Confidence Thresholds:**
   - Balance between precision and recall based on application
   - Use adaptive thresholds based on context

## 4. Manipulation Challenges

### Challenge 1: Grasp Failures

**Symptoms:**
- Robot fails to pick up objects consistently
- Dropping objects after successful grasp

**Solutions:**
1. **Implement Grasp Quality Assessment:**
   ```python
   class GraspQualityAssessment:
       def evaluate_grasp_stability(self, grasp_params, object_properties):
           # Calculate grasp stability metrics
           grasp_width = grasp_params['width']
           object_width = object_properties['width']
           
           # Check grasp stability conditions
           if grasp_width < object_width * 0.3:
               return {'stable': False, 'reason': 'Gripper too wide'}
           if grasp_width > object_width * 0.9:
               return {'stable': False, 'reason': 'Gripper too narrow'}
           
           # Check force distribution
           contact_points = grasp_params['contact_points']
           if not self.has_good_force_closure(contact_points):
               return {'stable': False, 'reason': 'Poor force closure'}
           
           return {'stable': True, 'reason': 'Good grasp quality'}
   ```

2. **Use Multi-Grasp Strategy:**
   - Plan multiple grasp points for the same object
   - Execute alternative grasps if first attempt fails
   - Implement grasp verification after execution

3. **Adjust Gripper Parameters:**
   - Tune grip force based on object weight/material
   - Implement force control for delicate objects
   - Use tactile feedback if available

### Challenge 2: Inverse Kinematics Failures

**Symptoms:**
- Robot cannot reach target positions
- Joint limit violations
- Singular configurations

**Solutions:**
1. **Use Multiple IK Solvers:**
   ```python
   class MultiIKSolver:
       def __init__(self):
           self.solvers = [
               self.analytical_ik,  # Fast, specific cases
               self.jacobian_ik,    # General purpose
               self.rrt_ik          # Sampling-based for complex cases
           ]
       
       def solve_ik(self, target_pose):
           for solver in self.solvers:
               try:
                   solution = solver(target_pose)
                   if self.is_valid_solution(solution):
                       return solution
               except:
                   continue
           
           return None  # No valid solution found
   ```

2. **Implement Reachability Checking:**
   - Pre-check if target is reachable before attempting IK
   - Use workspace visualization to identify unreachable areas

3. **Use Redundancy Resolution:**
   - For redundant robots, optimize secondary objectives
   - Avoid joint limits and singularities

### Challenge 3: Collision Avoidance During Manipulation

**Symptoms:**
- Robot collides with environment during manipulation
- Self-collisions between robot parts

**Solutions:**
1. **Implement Trajectory Collision Checking:**
   ```python
   class TrajectoryCollisionChecker:
       def check_trajectory_for_collisions(self, trajectory, environment_model):
           for waypoint in trajectory:
               if self.is_collision_state(waypoint, environment_model):
                   return False
           return True
       
       def is_collision_state(self, joint_state, env_model):
           # Check robot links against environment
           robot_poses = self.forward_kinematics(joint_state)
           for link_pose in robot_poses:
               if self.check_link_collision(link_pose, env_model):
                   return True
           return False
   ```

2. **Use Motion Planning with Collision Avoidance:**
   - Integrate with MoveIt! for collision-aware planning
   - Use CHOMP or STOMP for smooth, collision-free trajectories

## 5. System Integration Challenges

### Challenge 1: Component Synchronization Issues

**Symptoms:**
- Race conditions between components
- Out-of-order message processing
- State inconsistency

**Solutions:**
1. **Use Proper Message Timestamping:**
   ```python
   from rclpy.time import Time
   from builtin_interfaces.msg import Time as TimeMsg
   
   def create_timestamped_message(data):
       msg = YourMessageType()
       msg.header.stamp = self.get_clock().now().to_msg()
       msg.data = data
       return msg
   ```

2. **Implement State Machines:**
   ```python
   from enum import Enum
   
   class RobotState(Enum):
       IDLE = 1
       NAVIGATING = 2
       MANIPULATING = 3
       ERROR = 4
       SAFETY_STOP = 5
   
   class StateManager:
       def __init__(self):
           self.current_state = RobotState.IDLE
       
       def transition_to(self, new_state):
           # Define valid state transitions
           valid_transitions = {
               RobotState.IDLE: [RobotState.NAVIGATING, RobotState.MANIPULATING, RobotState.SAFETY_STOP],
               RobotState.NAVIGATING: [RobotState.IDLE, RobotState.SAFETY_STOP],
               RobotState.MANIPULATING: [RobotState.IDLE, RobotState.SAFETY_STOP],
               RobotState.SAFETY_STOP: [RobotState.IDLE]
           }
           
           if new_state in valid_transitions.get(self.current_state, []):
               self.current_state = new_state
               return True
           return False
   ```

3. **Use Action Servers for Long-Running Tasks:**
   - Actions provide feedback and goal preemption
   - Better than simple topics for complex operations

### Challenge 2: Resource Management Conflicts

**Symptoms:**
- CPU/GPU overutilization
- Memory leaks in long-running processes
- Competition for hardware resources

**Solutions:**
1. **Implement Resource Monitoring:**
   ```python
   import psutil
   import GPUtil
   
   class ResourceManager:
       def monitor_resources(self):
           # CPU usage
           cpu_percent = psutil.cpu_percent(interval=1)
           
           # Memory usage
           memory_percent = psutil.virtual_memory().percent
           
           # GPU usage (if available)
           gpus = GPUtil.getGPUs()
           if gpus:
               gpu_load = gpus[0].load * 100
               gpu_memory = gpus[0].memoryUtil * 100
           
           # Log or handle high resource usage
           if cpu_percent > 90 or memory_percent > 90:
               self.throttle_non_critical_processes()
   ```

2. **Use Process Isolation:**
   - Separate resource-intensive components into different processes
   - Use different priority levels for different components

### Challenge 3: Safety System Interference

**Symptoms:**
- Safety system stopping robot during valid operations
- Overly conservative safety responses
- Difficulty in tuning safety parameters

**Solutions:**
1. **Implement Layered Safety Approach:**
   ```python
   class SafetyManager:
       def __init__(self):
           self.safety_layers = [
               self.hardware_safety,      # Immediate physical safety
               self.control_safety,       # Motion constraints
               self.software_safety,      # Task-level safety
               self.ai_safety             # High-level safety reasoning
           ]
       
       def check_safety(self, action):
           for layer in self.safety_layers:
               if not layer(action):
                   return False
           return True
   ```

2. **Use Safety Parameter Tuning:**
   - Create parameter server for safety thresholds
   - Allow runtime adjustment during testing
   - Log safety interventions for analysis

## 6. Performance Optimization

### Challenge 1: High Computational Load

**Symptoms:**
- Slow response times
- Frame drops in perception
- Navigation latency

**Solutions:**
1. **Implement Component Prioritization:**
   ```python
   import threading
   import queue
   
   class PriorityProcessor:
       def __init__(self):
           self.high_priority_queue = queue.PriorityQueue()
           self.low_priority_queue = queue.Queue()
           self.processing_thread = threading.Thread(target=self.process)
       
       def submit_task(self, task, priority=0):
           if priority > 0:
               self.high_priority_queue.put((priority, task))
           else:
               self.low_priority_queue.put(task)
   ```

2. **Use Asynchronous Processing:**
   - Process sensor data asynchronously
   - Use callbacks for non-blocking operations
   - Implement result caching

### Challenge 2: Network Communication Bottlenecks

**Symptoms:**
- Message delays in distributed systems
- Bandwidth limitations with sensor data
- Connection timeouts

**Solutions:**
1. **Optimize Message Sizes:**
   - Compress large data (images, point clouds)
   - Use appropriate QoS settings
   - Implement message throttling

2. **Use Edge Computing:**
   - Process data locally when possible
   - Only transmit essential information
   - Implement data summarization techniques

## 7. Debugging and Monitoring

### Essential Debugging Tools

1. **ROS 2 Tools:**
   ```bash
   # Monitor topics
   ros2 topic echo /topic_name
   
   # Visualize with RViz
   rviz2
   
   # Check system status
   ros2 run rqt_graph rqt_graph
   ```

2. **Custom Logging:**
   ```python
   import rclpy
   from rclpy.node import Node
   
   class DebuggableNode(Node):
       def __init__(self):
           super().__init__('debuggable_node')
           self.debug_publisher = self.create_publisher(String, 'debug_info', 10)
       
       def debug_log(self, message, level='info'):
           self.get_logger().info(f"[DEBUG] {message}")
           
           # Publish to debug topic for external monitoring
           debug_msg = String()
           debug_msg.data = f"{level}: {message}"
           self.debug_publisher.publish(debug_msg)
   ```

### Common Debugging Approaches

1. **Component Isolation:**
   - Test components individually
   - Use mock data for inputs
   - Verify outputs independently

2. **Incremental Integration:**
   - Start with simple, working components
   - Add complexity gradually
   - Test at each integration step

3. **Simulation Before Hardware:**
   - Validate in Gazebo simulation first
   - Use realistic physics parameters
   - Test edge cases in simulation

This troubleshooting guide provides systematic approaches to common challenges in the capstone project. When encountering issues, start with the most likely causes and work through the solutions methodically.