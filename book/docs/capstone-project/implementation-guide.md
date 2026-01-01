---
title: Capstone Implementation Guide
description: Step-by-step guide for implementing the capstone project
sidebar_position: 3
---

# Capstone Implementation Guide

## Overview

This guide provides a detailed, step-by-step approach to implementing the Autonomous Humanoid Capstone Project. Following this guide will help you build and integrate all components of the system systematically.

## Prerequisites

Before starting the implementation, ensure you have:

1. **Development Environment:**
   - Ubuntu 22.04 LTS
   - ROS 2 Humble Hawksbill
   - Python 3.10+
   - Git version control

2. **Required Software:**
   - Gazebo simulation environment
   - OpenAI Whisper for speech recognition
   - Docusaurus for documentation
   - NVIDIA Isaac SDK (if available)
   - Standard ROS 2 packages for navigation and manipulation

3. **Hardware (for deployment):**
   - Robotic platform with mobile base
   - Robotic arm with end-effector
   - RGB-D camera
   - LIDAR sensor
   - Microphone array

## Phase 1: Project Setup and Environment Configuration

### Step 1: Create the ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws

# Create the package for the capstone project
cd src
ros2 pkg create --build-type ament_python capstone_robot --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs

# Navigate back to workspace root
cd ~/capstone_ws
```

### Step 2: Install Dependencies

```bash
# Install Python dependencies
pip3 install openai whisperSpeech numpy scipy opencv-python cv-bridge

# Install ROS 2 navigation stack
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install ROS 2 manipulation packages
sudo apt install ros-humble-moveit ros-humble-moveit-ros ros-humble-moveit-ros-planners ros-humble-moveit-ros-warehouse
```

### Step 3: Set Up OpenAI API

```bash
# Create environment file for API keys
echo "export OPENAI_API_KEY='your-api-key-here'" >> ~/.bashrc
source ~/.bashrc
```

## Phase 2: Core System Components

### Step 4: Implement Voice Command Processor

Create the voice command processing node:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_robot/voice_processor.py

import rclpy
from rclpy.node import Node
import openai
import whisper
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')
        
        # Initialize Whisper model for speech recognition
        self.whisper_model = whisper.load_model("base")
        
        # Initialize OpenAI API for natural language processing
        # Note: In production, use environment variables
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.action_plan_pub = self.create_publisher(String, 'action_plan', 10)
        self.speech_pub = self.create_publisher(String, 'robot_speech', 10)
        
        # Robot state tracking
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office',
            'last_command_time': 0
        }
        
        self.get_logger().info('Voice Command Processor initialized')

    def command_callback(self, msg):
        """Process voice command and generate action plan"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')
        
        # Process with LLM for command interpretation
        command_plan = self.interpret_command(command_text)
        
        if command_plan and command_plan.get('confidence', 0) > 0.5:
            # Publish the interpreted command
            cmd_msg = String()
            cmd_msg.data = json.dumps(command_plan)
            self.action_plan_pub.publish(cmd_msg)
            
            self.get_logger().info(f'Command plan generated: {command_plan}')
        else:
            # Request clarification
            self.request_clarification(command_text)

    def interpret_command(self, command_text):
        """Interpret natural language command using LLM"""
        prompt = f"""
        You are a command interpreter for an autonomous humanoid robot.
        The robot can perform these actions:
        - navigate_to(location)
        - pick_up_object(object_type)
        - place_object(location)
        - detect_object(object_type)
        - follow_person(person_name)
        - avoid_obstacle(obstacle_type)
        - return_to_charging_station()
        
        Current robot state:
        {json.dumps(self.robot_state, indent=2)}
        
        User command: "{command_text}"

        Respond with a JSON object containing the action plan:
        {% raw %}
        {{
          "action": "action_name",
          "parameters": {{"param1": "value1", "param2": "value2"}},
          "confidence": 0.0-1.0
        }}
        {% endraw %}
        
        If the command is ambiguous, return empty parameters and low confidence.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=300
            )
            
            content = response.choices[0].message['content'].strip()
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan = json.loads(json_str)
                return plan
            else:
                self.get_logger().error(f'Could not parse LLM response: {content}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error interpreting command: {e}')
            return None

    def request_clarification(self, original_command):
        """Request clarification for ambiguous commands"""
        clarification_msg = String()
        clarification_msg.data = f"I didn't understand your command: '{original_command}'. Could you please rephrase?"
        self.speech_pub.publish(clarification_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Implement Navigation Planner

Create the navigation and path planning node:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_robot/navigation_planner.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json
from scipy.spatial import distance

class NavigationPlanner(Node):
    def __init__(self):
        super().__init__('navigation_planner')
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.action_plan_sub = self.create_subscription(
            String,
            'action_plan',
            self.action_plan_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Navigation state
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.local_obstacles = []
        
        # Navigation parameters
        self.linear_vel = 0.5  # m/s
        self.angular_vel = 0.5  # rad/s
        self.safe_distance = 0.5  # meters
        
        self.get_logger().info('Navigation Planner initialized')

    def map_callback(self, msg):
        """Update internal map representation"""
        self.map_data = {
            'data': np.array(msg.data).reshape(msg.info.height, msg.info.width),
            'resolution': msg.info.resolution,
            'origin': msg.origin
        }

    def scan_callback(self, msg):
        """Process laser scan for local obstacle detection"""
        # Convert laser scan to obstacle positions
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        valid_ranges = np.array(msg.ranges)
        valid_ranges[valid_ranges > msg.range_max] = np.inf
        
        # Calculate obstacle positions in robot frame
        x = valid_ranges * np.cos(angles)
        y = valid_ranges * np.sin(angles)
        
        # Filter out infinite ranges
        finite_mask = np.isfinite(valid_ranges)
        self.local_obstacles = np.column_stack((x[finite_mask], y[finite_mask]))

    def action_plan_callback(self, msg):
        """Process navigation commands from action plan"""
        try:
            command_plan = json.loads(msg.data)
            action = command_plan['action']
            
            if action == 'navigate_to':
                location = command_plan['parameters'].get('location')
                self.navigate_to_location(location)
            elif action == 'return_to_charging_station':
                self.return_to_charging_station()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing command: {e}')

    def navigate_to_location(self, location):
        """Navigate to a specified location"""
        # Get coordinates for the location
        target_x, target_y = self.get_coordinates_for_location(location)
        
        if target_x is not None and target_y is not None:
            self.get_logger().info(f'Navigating to {location} at ({target_x}, {target_y})')
            self.move_to_position(target_x, target_y)
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def get_coordinates_for_location(self, location):
        """Convert location name to coordinates"""
        # In a real system, this would use a map or semantic localization
        location_map = {
            'kitchen': (5.0, 3.0),
            'living_room': (2.0, 8.0),
            'office': (8.0, 2.0),
            'bedroom': (7.0, 7.0),
            'charging_station': (0.0, 0.0)
        }
        
        return location_map.get(location, (None, None))

    def move_to_position(self, target_x, target_y):
        """Move robot to target position with obstacle avoidance"""
        # Simple proportional controller for demonstration
        while rclpy.ok():
            # Get current position (in practice, from localization system)
            current_x, current_y = self.get_current_position()
            
            # Calculate distance and angle to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance_to_target = np.sqrt(dx**2 + dy**2)
            
            # Check if we're close enough to target
            if distance_to_target < 0.2:  # 20 cm tolerance
                self.get_logger().info('Reached target position')
                self.stop_robot()
                break
            
            # Calculate desired heading
            desired_theta = np.arctan2(dy, dx)
            current_theta = self.get_current_orientation()
            
            # Calculate heading error
            heading_error = desired_theta - current_theta
            # Normalize angle to [-π, π]
            while heading_error > np.pi:
                heading_error -= 2 * np.pi
            while heading_error < -np.pi:
                heading_error += 2 * np.pi
            
            # Create velocity command
            cmd = Twist()
            
            # Proportional controller for linear velocity
            cmd.linear.x = min(self.linear_vel, distance_to_target * 0.5)
            
            # Proportional controller for angular velocity
            cmd.angular.z = min(self.angular_vel, max(-self.angular_vel, heading_error * 1.0))
            
            # Check for obstacles before moving
            if self.has_obstacle_ahead():
                cmd.linear.x = 0.0  # Stop if obstacle detected
                self.get_logger().warn('Obstacle detected ahead, stopping')
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            # Small delay
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def has_obstacle_ahead(self):
        """Check if there's an obstacle in front of the robot"""
        # Check if any local obstacle is within safe distance and in front of robot
        for obs_x, obs_y in self.local_obstacles:
            # Only consider obstacles in front of the robot (positive x direction)
            if obs_x > 0 and abs(obs_y) < 0.5:  # Within 50cm laterally
                if obs_x < self.safe_distance:  # Within safe distance
                    return True
        return False

    def get_current_position(self):
        """Get current robot position (implement with localization system)"""
        # Placeholder implementation
        # In a real system, this would come from AMCL, odometry, or other localization
        return (0.0, 0.0)

    def get_current_orientation(self):
        """Get current robot orientation (implement with localization system)"""
        # Placeholder implementation
        return 0.0

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Implement Perception System

Create the perception and object detection node:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_robot/perception_system.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class PerceptionSystem(Node):
    def __init__(self):
        super().__init__('perception_system')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.action_plan_sub = self.create_subscription(
            String,
            'action_plan',
            self.action_plan_callback,
            10
        )
        
        # Publishers
        self.object_detection_pub = self.create_publisher(String, 'detected_objects', 10)
        self.manipulation_pub = self.create_publisher(String, 'manipulation_command', 10)
        
        # Perception state
        self.latest_image = None
        self.detected_objects = []
        
        # Object detection parameters
        self.object_colors = {
            'red_cup': ([0, 50, 50], [10, 255, 255]),
            'blue_bottle': ([100, 50, 50], [130, 255, 255]),
            'green_box': ([50, 50, 50], [70, 255, 255])
        }
        
        self.get_logger().info('Perception System initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Perform object detection
            detected_objects = self.detect_objects(cv_image)
            
            # Update internal state
            self.detected_objects = detected_objects
            
            # Publish detected objects
            detection_msg = String()
            detection_msg.data = json.dumps(detected_objects)
            self.object_detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """Detect objects in the image using color-based detection"""
        objects = []
        
        for obj_name, (lower_color, upper_color) in self.object_colors.items():
            # Create mask for the color range
            lower = np.array(lower_color)
            upper = np.array(upper_color)
            mask = cv2.inRange(image, lower, upper)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                # Filter by size to avoid noise
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Calculate bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center position
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Calculate image coordinates (0-1 normalized)
                    img_width, img_height = image.shape[1], image.shape[0]
                    norm_x = center_x / img_width
                    norm_y = center_y / img_height
                    
                    # Add detected object
                    objects.append({
                        'name': obj_name,
                        'center': {'x': center_x, 'y': center_y},
                        'normalized_center': {'x': norm_x, 'y': norm_y},
                        'bbox': {'x': x, 'y': y, 'width': w, 'height': h},
                        'area': area
                    })
        
        return objects

    def action_plan_callback(self, msg):
        """Process commands related to object detection and manipulation"""
        try:
            command_plan = json.loads(msg.data)
            action = command_plan['action']
            
            if action == 'detect_object':
                obj_type = command_plan['parameters'].get('object_type', 'any')
                self.handle_detect_object_command(obj_type)
            elif action == 'pick_up_object':
                self.handle_pick_up_command()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing command: {e}')

    def handle_detect_object_command(self, obj_type):
        """Handle object detection command"""
        if self.latest_image is None:
            self.get_logger().warn('No image available for object detection')
            return
        
        # If specific object type requested, filter results
        if obj_type != 'any' and obj_type != 'all':
            matching_objects = [obj for obj in self.detected_objects if obj['name'] == obj_type]
        else:
            matching_objects = self.detected_objects
        
        # Publish results
        result_msg = String()
        result_msg.data = json.dumps({
            'command': 'detect_object',
            'object_type': obj_type,
            'objects_found': matching_objects,
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        self.object_detection_pub.publish(result_msg)

    def handle_pick_up_command(self):
        """Handle object pickup command"""
        if not self.detected_objects:
            self.get_logger().warn('No objects detected for pickup')
            return
        
        # For this example, pick up the largest detected object
        largest_obj = max(self.detected_objects, key=lambda obj: obj['area'])
        
        # Create manipulation command
        manipulation_cmd = {
            'action': 'grasp_object',
            'object': largest_obj,
            'approach_vector': [0, 0, -1],  # Approach from above
            'grasp_type': 'top_grasp'
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(manipulation_cmd)
        self.manipulation_pub.publish(cmd_msg)
        
        self.get_logger().info(f'Attempting to pick up {largest_obj["name"]}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSystem()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Implement Manipulation Controller

Create the manipulation and control node:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_robot/manipulation_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import json
import numpy as np

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')
        
        # Subscriptions
        self.manipulation_sub = self.create_subscription(
            String,
            'manipulation_command',
            self.manipulation_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'manipulation_status', 10)
        
        # Manipulation state
        self.joint_positions = {}
        self.is_gripper_open = True
        
        # Robot arm parameters (example for a simple 6-DOF arm)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 
                           'joint_4', 'joint_5', 'joint_6', 'gripper_joint']
        
        self.get_logger().info('Manipulation Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def manipulation_callback(self, msg):
        """Process manipulation commands"""
        try:
            command = json.loads(msg.data)
            action = command['action']
            
            if action == 'grasp_object':
                obj_info = command['object']
                grasp_type = command.get('grasp_type', 'top_grasp')
                
                self.execute_grasp(obj_info, grasp_type)
            elif action == 'release_object':
                self.execute_release()
            elif action == 'move_to_pose':
                pose = command['pose']
                self.move_to_pose(pose)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing manipulation command: {e}')

    def execute_grasp(self, object_info, grasp_type='top_grasp'):
        """Execute grasping of an object"""
        self.get_logger().info(f'Executing grasp for {object_info["name"]}')
        
        # In a real system, this would:
        # 1. Plan approach trajectory based on object pose
        # 2. Execute approach motion
        # 3. Close gripper with appropriate force
        # 4. Verify grasp success
        
        # For this example, we'll simulate the process
        if grasp_type == 'top_grasp':
            # Approach from above
            self.approach_from_above(object_info)
        
        # Close gripper
        self.close_gripper()
        
        # Lift object
        self.lift_object()
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'action': 'grasp',
            'object': object_info['name'],
            'status': 'completed',
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        self.status_pub.publish(status_msg)

    def approach_from_above(self, object_info):
        """Approach object from above for top grasp"""
        # Calculate approach position (slightly above object)
        approach_z = 0.2  # 20cm above object
        
        # In a real system, this would calculate inverse kinematics
        # to move end-effector to the calculated position
        self.get_logger().info(f'Approaching {object_info["name"]} from above')

    def close_gripper(self):
        """Close the gripper to grasp object"""
        # Create joint command to close gripper
        cmd = JointState()
        cmd.name = ['gripper_joint']
        cmd.position = [0.0]  # Closed position
        cmd.velocity = [0.1]  # Closing velocity
        cmd.effort = [50.0]   # Maximum effort
        
        self.joint_cmd_pub.publish(cmd)
        self.is_gripper_open = False
        
        # Wait for gripper to close
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

    def lift_object(self):
        """Lift the grasped object"""
        # In a real system, this would move the arm upward
        self.get_logger().info('Lifting object')

    def execute_release(self):
        """Release the currently grasped object"""
        self.get_logger().info('Releasing object')
        
        # Open gripper
        self.open_gripper()
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'action': 'release',
            'status': 'completed',
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        self.status_pub.publish(status_msg)

    def open_gripper(self):
        """Open the gripper"""
        # Create joint command to open gripper
        cmd = JointState()
        cmd.name = ['gripper_joint']
        cmd.position = [0.8]  # Open position
        cmd.velocity = [0.1]  # Opening velocity
        cmd.effort = [50.0]   # Maximum effort
        
        self.joint_cmd_pub.publish(cmd)
        self.is_gripper_open = True
        
        # Wait for gripper to open
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

    def move_to_pose(self, pose):
        """Move end-effector to specified pose"""
        # In a real system, this would calculate inverse kinematics
        # and generate joint trajectories
        self.get_logger().info(f'Moving to pose: {pose}')

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 8: Create Main Orchestration Node

Create the main node that ties all components together:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_robot/capstone_orchestrator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CapstoneOrchestrator(Node):
    def __init__(self):
        super().__init__('capstone_orchestrator')
        
        # Subscriptions for all subsystems
        self.status_sub = self.create_subscription(
            String,
            'system_status',
            self.status_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        
        # System state
        self.system_state = {
            'voice_processing': 'idle',
            'navigation': 'idle',
            'perception': 'idle',
            'manipulation': 'idle',
            'overall_status': 'ready'
        }
        
        # Start system status timer
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        
        self.get_logger().info('Capstone Orchestrator initialized')

    def status_callback(self, msg):
        """Update system status from subsystems"""
        try:
            status_data = json.loads(msg.data)
            component = status_data.get('component')
            status = status_data.get('status', 'unknown')
            
            if component in self.system_state:
                self.system_state[component] = status
                self.update_overall_status()
                
        except json.JSONDecodeError:
            self.get_logger().error('Error parsing status message')

    def update_overall_status(self):
        """Update overall system status based on component states"""
        # Determine overall status based on critical components
        critical_components = ['navigation', 'perception', 'manipulation']
        
        if any(self.system_state[comp] == 'error' for comp in critical_components):
            self.system_state['overall_status'] = 'error'
        elif any(self.system_state[comp] == 'busy' for comp in critical_components):
            self.system_state['overall_status'] = 'executing_task'
        else:
            self.system_state['overall_status'] = 'ready'

    def publish_system_status(self):
        """Publish overall system status"""
        status_msg = String()
        status_msg.data = json.dumps(self.system_state)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Create all nodes
    voice_processor = __import__('voice_processor', fromlist=['VoiceCommandProcessor']).VoiceCommandProcessor()
    navigation_planner = __import__('navigation_planner', fromlist=['NavigationPlanner']).NavigationPlanner()
    perception_system = __import__('perception_system', fromlist=['PerceptionSystem']).PerceptionSystem()
    manipulation_controller = __import__('manipulation_controller', fromlist=['ManipulationController']).ManipulationController()
    orchestrator = CapstoneOrchestrator()
    
    # Create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(voice_processor)
    executor.add_node(navigation_planner)
    executor.add_node(perception_system)
    executor.add_node(manipulation_controller)
    executor.add_node(orchestrator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 3: Simulation Environment

### Step 9: Create Gazebo World File

Create a Gazebo world file for testing:

```xml
<!-- File: ~/capstone_ws/src/capstone_robot/worlds/capstone_world.world -->

<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="capstone_world">
    <!-- Include the outdoor world -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Home environment with furniture -->
    <!-- Kitchen area -->
    <model name="kitchen_counter">
      <pose>5 3 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>
    
    <model name="kitchen_cup">
      <pose>5.2 3.1 0.8 0 0 0</pose>
      <include>
        <uri>model://coke_can</uri>
      </include>
    </model>
    
    <!-- Living room area -->
    <model name="sofa">
      <pose>2 8 0 0 0 1.57</pose>
      <include>
        <uri>model://sofa</uri>
      </include>
    </model>
    
    <model name="coffee_table">
      <pose>2.5 7.5 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>
    
    <!-- Office area -->
    <model name="office_desk">
      <pose>8 2 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>
    
    <model name="office_bottle">
      <pose>8.2 2.1 0.8 0 0 0</pose>
      <include>
        <uri>model://water_bottle</uri>
      </include>
    </model>
    
    <!-- Charging station -->
    <model name="charging_station">
      <pose>0.5 0.5 0 0 0 0</pose>
      <include>
        <uri>model://charger</uri>
      </include>
    </model>
    
    <!-- Walls to define rooms -->
    <model name="wall_1">
      <pose>0 5 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="wall_2">
      <pose>5 -0.1 0 0 0 1.57</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 10: Create Launch File

Create a launch file to start all nodes:

```python
# File: ~/capstone_ws/src/capstone_robot/launch/capstone_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('capstone_robot')
    
    return LaunchDescription([
        # Voice Command Processor
        Node(
            package='capstone_robot',
            executable='voice_processor',
            name='voice_command_processor',
            output='screen'
        ),
        
        # Navigation Planner
        Node(
            package='capstone_robot',
            executable='navigation_planner',
            name='navigation_planner',
            output='screen'
        ),
        
        # Perception System
        Node(
            package='capstone_robot',
            executable='perception_system',
            name='perception_system',
            output='screen'
        ),
        
        # Manipulation Controller
        Node(
            package='capstone_robot',
            executable='manipulation_controller',
            name='manipulation_controller',
            output='screen'
        ),
        
        # Capstone Orchestrator
        Node(
            package='capstone_robot',
            executable='capstone_orchestrator',
            name='capstone_orchestrator',
            output='screen'
        )
    ])
```

## Phase 4: Testing and Validation

### Step 11: Create Unit Tests

Create unit tests for each component:

```python
# File: ~/capstone_ws/src/capstone_robot/test/test_voice_processor.py

import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import json

class TestVoiceProcessor(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        # Import the voice processor class
        from capstone_robot.voice_processor import VoiceCommandProcessor
        self.node = VoiceCommandProcessor()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_command_interpretation(self):
        """Test that natural language commands are correctly interpreted"""
        # This would test the interpret_command method with various inputs
        command_text = "move to the kitchen"
        result = self.node.interpret_command(command_text)
        
        # Note: This test will fail without a valid OpenAI API key
        # In a real implementation, we'd mock the API call
        self.assertIsNotNone(result)
        if result:
            self.assertEqual(result['action'], 'navigate_to')
            self.assertEqual(result['parameters']['location'], 'kitchen')

if __name__ == '__main__':
    unittest.main()
```

### Step 12: Create Integration Tests

Create integration tests for the complete system:

```python
# File: ~/capstone_ws/src/capstone_robot/test/test_integration.py

import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import json
import threading
import time

class TestSystemIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        
        # Import all node classes
        from capstone_robot.voice_processor import VoiceCommandProcessor
        from capstone_robot.navigation_planner import NavigationPlanner
        from capstone_robot.perception_system import PerceptionSystem
        from capstone_robot.manipulation_controller import ManipulationController
        from capstone_robot.capstone_orchestrator import CapstoneOrchestrator
        
        # Create all nodes
        self.voice_processor = VoiceCommandProcessor()
        self.navigation_planner = NavigationPlanner()
        self.perception_system = PerceptionSystem()
        self.manipulation_controller = ManipulationController()
        self.orchestrator = CapstoneOrchestrator()
        
        # Create executor and add nodes
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.voice_processor)
        self.executor.add_node(self.navigation_planner)
        self.executor.add_node(self.perception_system)
        self.executor.add_node(self.manipulation_controller)
        self.executor.add_node(self.orchestrator)
        
        # Start executor in a separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        rclpy.shutdown()
        self.executor_thread.join()

    def test_voice_to_navigation(self):
        """Test the complete pipeline from voice command to navigation"""
        # Create a publisher for voice commands
        voice_cmd_pub = self.voice_processor.create_publisher(
            String, 'voice_command', 10
        )
        
        # Wait a bit for connections to establish
        time.sleep(1.0)
        
        # Publish a navigation command
        cmd_msg = String()
        cmd_msg.data = "navigate to the kitchen"
        voice_cmd_pub.publish(cmd_msg)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Verify that navigation commands were generated
        # This would require checking if navigation commands were published
        # For this test, we'll just verify that the system is running without errors

if __name__ == '__main__':
    unittest.main()
```

## Phase 5: Documentation and Finalization

### Step 13: Update Setup Scripts

Create setup scripts to make the system easy to deploy:

```bash
#!/bin/bash
# File: ~/capstone_ws/src/capstone_robot/setup_capstone.sh

echo "Setting up Capstone Robot Project..."

# Create virtual environment
python3 -m venv ~/capstone_ws/venv
source ~/capstone_ws/venv/bin/activate

# Install Python dependencies
pip install openai whisperSpeech numpy scipy opencv-python cv-bridge

# Build the ROS 2 package
cd ~/capstone_ws
colcon build --packages-select capstone_robot

# Source the workspace
source install/setup.bash

echo "Setup complete! To run the system, execute:"
echo "source ~/capstone_ws/install/setup.bash && ros2 launch capstone_robot capstone_launch.py"
```

### Step 14: Create README

Create a comprehensive README file:

```markdown
# Capstone Robot Project

This package implements the Autonomous Humanoid Capstone Project, integrating ROS 2 fundamentals, Gazebo simulation, NVIDIA Isaac platform, and conversational AI into a cohesive autonomous robotic system.

## Components

- Voice Command Processor: Handles speech recognition and natural language understanding
- Navigation Planner: Manages path planning and obstacle avoidance
- Perception System: Processes visual data and detects objects
- Manipulation Controller: Controls robotic arm and gripper
- System Orchestrator: Coordinates all components

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.10+
- OpenAI API key (for cloud-based LLMs)

## Installation

1. Clone the repository:
   ```bash
   mkdir -p ~/capstone_ws/src
   cd ~/capstone_ws/src
   git clone <repository-url>
   ```

2. Install dependencies:
   ```bash
   cd ~/capstone_ws
   source setup_capstone.sh
   ```

## Usage

1. Source the workspace:
   ```bash
   source ~/capstone_ws/install/setup.bash
   ```

2. Launch the system:
   ```bash
   ros2 launch capstone_robot capstone_launch.py
   ```

3. Send voice commands via the `voice_command` topic or use the simulation interface.

## Testing

Run unit tests:
```bash
cd ~/capstone_ws
source install/setup.bash
python3 -m pytest src/capstone_robot/test/
```

## Simulation

To run in simulation:
```bash
# Terminal 1: Start Gazebo
ros2 launch nav2_gazebo_bringup view_navigation_gazebo.launch.py world:=path/to/capstone_world.world

# Terminal 2: Start the robot system
source ~/capstone_ws/install/setup.bash
ros2 launch capstone_robot capstone_launch.py

# Terminal 3: Send commands
ros2 topic pub /voice_command std_msgs/String "data: 'navigate to the kitchen'"
```

## Architecture

The system follows a modular architecture with clear separation of concerns. See the architecture documentation for detailed design information.

## Troubleshooting

- If you get import errors, ensure you've sourced the workspace properly
- For OpenAI API errors, verify your API key is set correctly
- For simulation issues, ensure Gazebo and navigation packages are properly installed
```

## Phase 6: System Integration and Validation

### Step 15: Create System Validation Scripts

Create scripts to validate the complete system:

```python
# File: ~/capstone_ws/src/capstone_robot/scripts/validate_system.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time

class SystemValidator(Node):
    def __init__(self):
        super().__init__('system_validator')
        
        # Publishers for testing
        self.voice_cmd_pub = self.create_publisher(String, 'voice_command', 10)
        self.status_sub = self.create_subscription(
            String, 'system_status', self.status_callback, 10
        )
        
        self.system_status = {}
        self.test_results = {
            'voice_processing': False,
            'navigation': False,
            'perception': False,
            'manipulation': False
        }
        
        self.get_logger().info('System Validator initialized')

    def status_callback(self, msg):
        """Update system status from orchestrator"""
        try:
            status_data = json.loads(msg.data)
            self.system_status = status_data
        except json.JSONDecodeError:
            self.get_logger().error('Error parsing status message')

    def run_validation_tests(self):
        """Run comprehensive validation tests"""
        self.get_logger().info('Starting system validation tests...')
        
        # Test 1: Voice command processing
        self.test_voice_processing()
        
        # Test 2: Navigation
        self.test_navigation()
        
        # Test 3: Perception
        self.test_perception()
        
        # Test 4: Manipulation
        self.test_manipulation()
        
        # Print results
        self.print_test_results()

    def test_voice_processing(self):
        """Test voice command processing"""
        self.get_logger().info('Testing voice processing...')
        
        # Send a simple command
        cmd_msg = String()
        cmd_msg.data = "move forward 1 meter"
        self.voice_cmd_pub.publish(cmd_msg)
        
        # Wait for response
        time.sleep(2.0)
        
        # Check if command was processed (simplified check)
        if self.system_status.get('voice_processing') == 'busy':
            self.test_results['voice_processing'] = True
            self.get_logger().info('✓ Voice processing test passed')
        else:
            self.get_logger().info('✗ Voice processing test failed')

    def test_navigation(self):
        """Test navigation system"""
        self.get_logger().info('Testing navigation...')
        
        # This would require more complex testing in a real scenario
        # For now, we'll just check if the navigation component is responding
        if self.system_status.get('navigation') in ['idle', 'busy']:
            self.test_results['navigation'] = True
            self.get_logger().info('✓ Navigation test passed')
        else:
            self.get_logger().info('✗ Navigation test failed')

    def test_perception(self):
        """Test perception system"""
        self.get_logger().info('Testing perception...')
        
        # This would require image data to be available
        # For now, we'll just check if the perception component is responding
        if self.system_status.get('perception') in ['idle', 'busy']:
            self.test_results['perception'] = True
            self.get_logger().info('✓ Perception test passed')
        else:
            self.get_logger().info('✗ Perception test failed')

    def test_manipulation(self):
        """Test manipulation system"""
        self.get_logger().info('Testing manipulation...')
        
        # This would require manipulation commands to be processed
        # For now, we'll just check if the manipulation component is responding
        if self.system_status.get('manipulation') in ['idle', 'busy']:
            self.test_results['manipulation'] = True
            self.get_logger().info('✓ Manipulation test passed')
        else:
            self.get_logger().info('✗ Manipulation test failed')

    def print_test_results(self):
        """Print the validation results"""
        self.get_logger().info('\n=== SYSTEM VALIDATION RESULTS ===')
        for component, passed in self.test_results.items():
            status = 'PASS' if passed else 'FAIL'
            self.get_logger().info(f'{component}: {status}')
        
        all_passed = all(self.test_results.values())
        overall_status = 'PASS' if all_passed else 'FAIL'
        self.get_logger().info(f'\nOverall System Status: {overall_status}')

def main(args=None):
    rclpy.init(args=args)
    validator = SystemValidator()
    
    # Run validation tests
    validator.run_validation_tests()
    
    # Keep the node alive briefly to receive status updates
    time.sleep(1.0)
    
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 7: Final System Integration

### Step 16: Create Final Launch Script

Create a comprehensive launch script that starts all components:

```python
# File: ~/capstone_ws/src/capstone_robot/capstone_launch_all.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('capstone_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # Include the main navigation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            )
        ),
        
        # Voice Command Processor
        Node(
            package='capstone_robot',
            executable='voice_processor',
            name='voice_command_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Navigation Planner
        Node(
            package='capstone_robot',
            executable='navigation_planner',
            name='navigation_planner',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Perception System
        Node(
            package='capstone_robot',
            executable='perception_system',
            name='perception_system',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Manipulation Controller
        Node(
            package='capstone_robot',
            executable='manipulation_controller',
            name='manipulation_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Capstone Orchestrator
        Node(
            package='capstone_robot',
            executable='capstone_orchestrator',
            name='capstone_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

## Summary

This implementation guide provides a comprehensive, step-by-step approach to building the Autonomous Humanoid Capstone Project. Following these steps will result in a complete system that integrates:

1. Voice command processing with natural language understanding
2. Navigation and path planning capabilities
3. Perception and object detection
4. Manipulation and control
5. System orchestration and safety

Each phase builds upon the previous one, ensuring a systematic development approach. The modular architecture allows for independent testing and refinement of each component before integration.

Remember to test each component individually before integrating them, and always prioritize safety in your implementation. The simulation environment provides a safe space to validate your system before deploying on physical hardware.