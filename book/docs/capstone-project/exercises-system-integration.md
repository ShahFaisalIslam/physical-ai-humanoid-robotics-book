---
title: Capstone Exercises for System Integration
description: Exercises to understand system integration in the capstone project
sidebar_position: 6
---

# Capstone Exercises for System Integration

## Overview

This section provides practical exercises to help you understand and implement system integration in the Autonomous Humanoid Capstone Project. These exercises build on all previous modules and focus on connecting voice processing, navigation, perception, and manipulation components into a cohesive system.

## Exercise 1: Basic Component Communication

### Objective
Implement basic communication between two components using ROS 2 topics.

### Task
Create a simple publisher-subscriber pair where one node publishes voice commands and another node receives and processes them.

### Implementation Steps
1. Create a publisher node that publishes simple commands like "move forward", "turn left", "stop"
2. Create a subscriber node that receives these commands and logs them
3. Test the communication between nodes

### Sample Code Structure
```python
# Publisher node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher = self.create_publisher(String, 'voice_commands', 10)
        self.timer = self.create_timer(2.0, self.publish_command)
        self.commands = ["move forward", "turn left", "stop"]
        self.index = 0

    def publish_command(self):
        msg = String()
        msg.data = self.commands[self.index % len(self.commands)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.index += 1

# Subscriber node
class CommandProcessor(Node):
    def __init__(self):
        super().__init__('command_processor')
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command here

def main(args=None):
    rclpy.init(args=args)
    
    publisher = VoiceCommandPublisher()
    processor = CommandProcessor()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(processor)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

### Validation
- Verify that commands are published and received correctly
- Check that both nodes can run simultaneously
- Test with different message frequencies

## Exercise 2: Multi-Component Coordination

### Objective
Coordinate three components: voice processing, navigation, and system monitoring.

### Task
Create a system where voice commands trigger navigation actions and system status is monitored.

### Implementation Steps
1. Implement a voice command interpreter that converts natural language to navigation goals
2. Create a navigation controller that receives goals and executes them
3. Implement a system monitor that tracks component status
4. Connect all components using appropriate ROS 2 interfaces

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import json

class VoiceNavigationCoordinator(Node):
    def __init__(self):
        super().__init__('voice_navigation_coordinator')
        
        # Subscriptions
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        
        # Internal state
        self.robot_position = None
        self.current_goal = None
        
        self.get_logger().info('Voice Navigation Coordinator initialized')

    def voice_callback(self, msg):
        """Process voice command and generate navigation goal"""
        command = msg.data.lower()
        
        # Simple command parsing
        if 'kitchen' in command:
            self.navigate_to('kitchen')
        elif 'living room' in command:
            self.navigate_to('living_room')
        elif 'stop' in command:
            self.stop_navigation()
        else:
            self.get_logger().info(f'Unknown command: {command}')

    def navigate_to(self, location):
        """Navigate to predefined location"""
        # Define location coordinates
        locations = {
            'kitchen': (5.0, 3.0, 0.0),
            'living_room': (2.0, 8.0, 0.0),
            'office': (8.0, 2.0, 0.0)
        }
        
        if location in locations:
            x, y, theta = locations[location]
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation.z = theta
            
            self.goal_pub.publish(goal_msg)
            self.current_goal = (x, y)
            
            self.get_logger().info(f'Navigating to {location} at ({x}, {y})')
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def stop_navigation(self):
        """Stop current navigation"""
        # Implementation would send stop command
        self.get_logger().info('Navigation stopped')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Check if reached goal
        if self.current_goal:
            dx = self.current_goal[0] - self.robot_position[0]
            dy = self.current_goal[1] - self.robot_position[1]
            distance = (dx**2 + dy**2)**0.5
            
            if distance < 0.5:  # Within 50cm of goal
                self.get_logger().info('Reached goal')
                self.current_goal = None

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavigationCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Validation
- Test voice commands that trigger navigation
- Verify that robot position updates correctly
- Check that system status is published appropriately

## Exercise 3: Perception-Navigation Integration

### Objective
Integrate perception data with navigation planning to enable obstacle-aware navigation.

### Task
Create a system that uses simulated perception data to update navigation plans in real-time.

### Implementation Steps
1. Create a perception simulator that publishes obstacle locations
2. Implement an obstacle avoidance system that modifies navigation plans
3. Integrate with the navigation system from Exercise 2

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np

class PerceptionNavigationIntegrator(Node):
    def __init__(self):
        super().__init__('perception_navigation_integrator')
        
        # Subscriptions
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.local_map_pub = self.create_publisher(OccupancyGrid, 'local_costmap', 10)
        
        # Internal state
        self.current_goal = None
        self.obstacles = []
        self.is_navigating = False
        
        self.get_logger().info('Perception Navigation Integrator initialized')

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Convert laser ranges to obstacle positions
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        valid_ranges = np.array(msg.ranges)
        
        # Filter out invalid ranges
        valid_mask = (valid_ranges > msg.range_min) & (valid_ranges < msg.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = valid_ranges[valid_mask]
        
        # Calculate obstacle positions in robot frame
        obstacle_x = valid_ranges * np.cos(valid_angles)
        obstacle_y = valid_ranges * np.sin(valid_angles)
        
        # Store obstacle positions
        self.obstacles = list(zip(obstacle_x, obstacle_y))
        
        # Update local costmap
        self.update_local_costmap()

    def goal_callback(self, msg):
        """Receive navigation goal and start navigation"""
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.is_navigating = True
        self.get_logger().info(f'New goal received: {self.current_goal}')

    def update_local_costmap(self):
        """Update local costmap with detected obstacles"""
        # Create a simple occupancy grid for local map
        resolution = 0.1  # 10cm resolution
        width = 20  # 2m wide
        height = 20  # 2m high
        
        # Initialize grid (all free space)
        grid_data = [0] * (width * height)  # 0 = free, 100 = occupied
        
        # Mark obstacle positions
        for obs_x, obs_y in self.obstacles:
            if -1.0 <= obs_x <= 1.0 and -1.0 <= obs_y <= 1.0:
                grid_x = int((obs_x + 1.0) / resolution)
                grid_y = int((obs_y + 1.0) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    grid_idx = grid_y * width + grid_x
                    if 0 <= grid_idx < len(grid_data):
                        grid_data[grid_idx] = 100  # Mark as occupied

        # Create and publish occupancy grid
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'base_link'
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = -1.0
        map_msg.info.origin.position.y = -1.0
        map_msg.data = grid_data
        
        self.local_map_pub.publish(map_msg)

    def navigate_with_obstacle_avoidance(self):
        """Navigate while avoiding obstacles"""
        if not self.current_goal or not self.is_navigating:
            return
            
        # Simple proportional controller with obstacle avoidance
        cmd = Twist()
        
        # Calculate direction to goal
        goal_x, goal_y = self.current_goal
        to_goal_angle = np.arctan2(goal_y, goal_x)
        
        # Check for obstacles in front
        front_obstacles = [(x, y) for x, y in self.obstacles if x > 0 and abs(y) < 0.5]
        
        if front_obstacles:
            # If obstacles in front, turn away
            cmd.angular.z = 0.5  # Turn right
            cmd.linear.x = 0.0   # Stop forward motion
        else:
            # Head toward goal
            cmd.angular.z = 0.5 * to_goal_angle
            cmd.linear.x = 0.2   # Move forward slowly
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNavigationIntegrator()
    
    # Timer to run navigation control
    node.create_timer(0.1, node.navigate_with_obstacle_avoidance)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Validation
- Verify that obstacles are detected from laser scan data
- Check that navigation adapts to avoid obstacles
- Test with different obstacle configurations

## Exercise 4: Voice-Perception-Manipulation Pipeline

### Objective
Create a complete pipeline from voice command to object manipulation using perception data.

### Task
Implement a system that takes a voice command like "pick up the red cup" and coordinates perception and manipulation to execute the task.

### Implementation Steps
1. Create a voice command parser that identifies manipulation tasks
2. Implement object detection to find the requested object
3. Create a manipulation controller to execute the grasp
4. Coordinate all components with appropriate feedback

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import json
import numpy as np

class VoicePerceptionManipulationPipeline(Node):
    def __init__(self):
        super().__init__('voice_perception_manipulation_pipeline')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Subscriptions
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Publishers
        self.manipulation_pub = self.create_publisher(String, 'manipulation_commands', 10)
        self.object_detection_pub = self.create_publisher(String, 'detected_objects', 10)
        
        # Internal state
        self.latest_image = None
        self.joint_positions = {}
        self.requested_object = None
        self.detected_objects = []
        
        # Object color definitions
        self.object_colors = {
            'red_cup': ([0, 50, 50], [10, 255, 255]),
            'blue_bottle': ([100, 50, 50], [130, 255, 255]),
            'green_box': ([50, 50, 50], [70, 255, 255])
        }
        
        self.get_logger().info('Voice Perception Manipulation Pipeline initialized')

    def voice_callback(self, msg):
        """Process voice command for manipulation"""
        command = msg.data.lower()
        
        # Check if command is for manipulation
        if 'pick up' in command or 'grasp' in command or 'take' in command:
            # Extract object type
            obj_type = self.extract_object_type(command)
            if obj_type:
                self.requested_object = obj_type
                self.get_logger().info(f'Requested to pick up: {obj_type}')
                
                # If we have recent image data, try to detect the object
                if self.latest_image is not None:
                    self.process_current_image_for_object(obj_type)

    def extract_object_type(self, command):
        """Extract object type from voice command"""
        # Simple keyword matching
        if 'red cup' in command or 'red_cup' in command:
            return 'red_cup'
        elif 'blue bottle' in command or 'blue_bottle' in command:
            return 'blue_bottle'
        elif 'green box' in command or 'green_box' in command:
            return 'green_box'
        else:
            # Try to match any known object type
            for obj_type in self.object_colors.keys():
                if obj_type.replace('_', ' ') in command:
                    return obj_type
        return None

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # If we're looking for a specific object, process immediately
            if self.requested_object:
                self.process_current_image_for_object(self.requested_object)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_current_image_for_object(self, obj_type):
        """Process current image to find the requested object"""
        if self.latest_image is None:
            self.get_logger().warn('No image available to process')
            return
        
        # Detect objects in image
        detected_objects = self.detect_objects_in_image(self.latest_image)
        
        # Filter for requested object type
        target_objects = [obj for obj in detected_objects if obj['name'] == obj_type]
        
        if target_objects:
            # Sort by size (largest first - likely closest)
            target_objects.sort(key=lambda x: x['area'], reverse=True)
            target_obj = target_objects[0]
            
            self.get_logger().info(f'Found {obj_type}: {target_obj}')
            
            # Send manipulation command
            self.execute_manipulation_for_object(target_obj)
        else:
            self.get_logger().info(f'{obj_type} not found in current view')

    def detect_objects_in_image(self, image):
        """Detect objects in an image using color-based detection"""
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
                if area > 300:  # Minimum area threshold
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

    def execute_manipulation_for_object(self, obj):
        """Execute manipulation to pick up the detected object"""
        # Create manipulation command
        manipulation_cmd = {
            'action': 'grasp_object',
            'object_info': obj,
            'approach_vector': [0, 0, -1],  # Approach from above
            'grasp_type': 'top_grasp'
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(manipulation_cmd)
        self.manipulation_pub.publish(cmd_msg)
        
        self.get_logger().info(f'Sent manipulation command for {obj["name"]}')

    def joint_state_callback(self, msg):
        """Update joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

def main(args=None):
    rclpy.init(args=args)
    node = VoicePerceptionManipulationPipeline()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Validation
- Test with voice commands requesting specific objects
- Verify object detection works correctly
- Check that manipulation commands are generated appropriately

## Exercise 5: System State Management

### Objective
Implement a system state manager that coordinates all components and handles transitions between operational states.

### Task
Create a state machine that manages the robot's operational states and coordinates component activities based on the current state.

### Implementation Steps
1. Define robot operational states (idle, navigating, manipulating, error, etc.)
2. Implement state transition logic
3. Coordinate component activities based on current state
4. Handle error states and recovery

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from enum import Enum
import json

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    ERROR = "error"
    SAFETY_STOP = "safety_stop"
    WAITING_FOR_COMMAND = "waiting_for_command"

class SystemStateManager(Node):
    def __init__(self):
        super().__init__('system_state_manager')
        
        # Subscriptions
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)
        
        self.status_sub = self.create_subscription(
            String, 'component_status', self.status_callback, 10)
        
        self.error_sub = self.create_subscription(
            String, 'error_reports', self.error_callback, 10)
        
        # Publishers
        self.state_pub = self.create_publisher(String, 'system_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize state
        self.current_state = RobotState.WAITING_FOR_COMMAND
        self.state_history = [self.current_state]
        
        # Component status tracking
        self.component_status = {
            'voice_processor': 'unknown',
            'navigator': 'unknown', 
            'manipulator': 'unknown',
            'perception': 'unknown'
        }
        
        # Start state monitoring
        self.state_timer = self.create_timer(1.0, self.monitor_state)
        
        self.get_logger().info('System State Manager initialized')

    def voice_callback(self, msg):
        """Handle voice commands based on current state"""
        if self.current_state == RobotState.WAITING_FOR_COMMAND:
            command = msg.data.lower()
            
            if 'navigate' in command or 'go to' in command:
                self.transition_to_state(RobotState.NAVIGATING)
            elif 'pick up' in command or 'grasp' in command:
                self.transition_to_state(RobotState.MANIPULATING)
            else:
                self.get_logger().info(f'Received command in {self.current_state.value}: {command}')
        else:
            self.get_logger().info(f'Ignoring command in state {self.current_state.value}')

    def status_callback(self, msg):
        """Update component status"""
        try:
            status_data = json.loads(msg.data)
            component = status_data.get('component')
            status = status_data.get('status')
            
            if component in self.component_status:
                self.component_status[component] = status
        except json.JSONDecodeError:
            self.get_logger().error('Invalid status message format')

    def error_callback(self, msg):
        """Handle error reports"""
        self.get_logger().error(f'Error reported: {msg.data}')
        self.transition_to_state(RobotState.ERROR)

    def transition_to_state(self, new_state):
        """Safely transition to a new state"""
        # Define valid state transitions
        valid_transitions = {
            RobotState.WAITING_FOR_COMMAND: [RobotState.NAVIGATING, RobotState.MANIPULATING, RobotState.SAFETY_STOP],
            RobotState.NAVIGATING: [RobotState.IDLE, RobotState.SAFETY_STOP, RobotState.ERROR],
            RobotState.MANIPULATING: [RobotState.IDLE, RobotState.SAFETY_STOP, RobotState.ERROR],
            RobotState.IDLE: [RobotState.NAVIGATING, RobotState.MANIPULATING, RobotState.WAITING_FOR_COMMAND, RobotState.SAFETY_STOP],
            RobotState.ERROR: [RobotState.SAFETY_STOP, RobotState.IDLE],
            RobotState.SAFETY_STOP: [RobotState.IDLE, RobotState.ERROR]
        }
        
        if new_state in valid_transitions.get(self.current_state, []):
            old_state = self.current_state
            self.current_state = new_state
            self.state_history.append(new_state)
            
            self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')
            
            # Execute state-specific actions
            self.execute_state_actions(new_state)
            
            # Publish new state
            state_msg = String()
            state_msg.data = json.dumps({
                'current_state': new_state.value,
                'timestamp': self.get_clock().now().nanoseconds
            })
            self.state_pub.publish(state_msg)
            
            return True
        else:
            self.get_logger().warn(f'Invalid state transition: {self.current_state.value} -> {new_state.value}')
            return False

    def execute_state_actions(self, state):
        """Execute actions specific to the current state"""
        if state == RobotState.SAFETY_STOP:
            # Emergency stop - halt all motion
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
        elif state == RobotState.IDLE:
            # In idle state, prepare for next command
            self.get_logger().info('Robot is idle, waiting for command')
        elif state == RobotState.NAVIGATING:
            # Navigation-specific actions would go here
            self.get_logger().info('Robot is navigating')
        elif state == RobotState.MANIPULATING:
            # Manipulation-specific actions would go here
            self.get_logger().info('Robot is manipulating')

    def monitor_state(self):
        """Monitor system state and handle any necessary transitions"""
        # Check component health
        for component, status in self.component_status.items():
            if status == 'error':
                self.get_logger().warn(f'{component} reported error status')
                if self.current_state != RobotState.ERROR:
                    self.transition_to_state(RobotState.ERROR)
        
        # Check for timeout in certain states
        if self.current_state in [RobotState.NAVIGATING, RobotState.MANIPULATING]:
            # Implementation would check for task completion or timeout
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SystemStateManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Validation
- Test state transitions with different commands
- Verify that appropriate actions are taken in each state
- Check error handling and recovery

## Exercise 6: Complete Integration Challenge

### Objective
Combine all previous exercises into a complete integrated system.

### Task
Create a main orchestrator node that integrates voice processing, navigation, perception, manipulation, and state management into a cohesive system.

### Implementation Steps
1. Create an orchestrator node that manages all components
2. Implement proper message routing between components
3. Add system-level error handling and recovery
4. Test complete end-to-end functionality

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import json
import numpy as np

class CapstoneOrchestrator(Node):
    def __init__(self):
        super().__init__('capstone_orchestrator')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Subscriptions
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)
        
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.manipulation_pub = self.create_publisher(String, 'manipulation_commands', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        
        # Internal state
        self.robot_position = None
        self.joint_positions = {}
        self.latest_image = None
        self.obstacles = []
        self.detected_objects = []
        
        # System state
        self.current_task = None
        self.task_queue = []
        
        # Object detection parameters
        self.object_colors = {
            'red_cup': ([0, 50, 50], [10, 255, 255]),
            'blue_bottle': ([100, 50, 50], [130, 255, 255]),
            'green_box': ([50, 50, 50], [70, 255, 255])
        }
        
        # Start system monitoring
        self.monitor_timer = self.create_timer(0.5, self.monitor_system)
        
        self.get_logger().info('Capstone Orchestrator initialized')

    def voice_callback(self, msg):
        """Process voice command and queue appropriate task"""
        command = msg.data.lower()
        self.get_logger().info(f'Received voice command: {command}')
        
        # Parse command and create appropriate task
        if 'navigate to' in command or 'go to' in command:
            location = self.extract_location(command)
            if location:
                task = {
                    'type': 'navigation',
                    'location': location,
                    'priority': 1
                }
                self.task_queue.append(task)
                
        elif 'pick up' in command or 'grasp' in command:
            obj_type = self.extract_object_type(command)
            if obj_type:
                task = {
                    'type': 'manipulation', 
                    'object_type': obj_type,
                    'priority': 2
                }
                self.task_queue.append(task)
        
        elif 'stop' in command:
            self.emergency_stop()
        
        # Publish system status
        self.publish_system_status()

    def extract_location(self, command):
        """Extract location from navigation command"""
        locations = ['kitchen', 'living room', 'office', 'bedroom', 'charging station']
        for loc in locations:
            if loc in command:
                return loc.replace(' ', '_')
        return None

    def extract_object_type(self, command):
        """Extract object type from manipulation command"""
        for obj_type in self.object_colors.keys():
            if obj_type.replace('_', ' ') in command:
                return obj_type
        return None

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Convert to obstacle positions (same as previous exercise)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        valid_ranges = np.array(msg.ranges)
        valid_mask = (valid_ranges > msg.range_min) & (valid_ranges < msg.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = valid_ranges[valid_mask]
        
        obstacle_x = valid_ranges * np.cos(valid_angles)
        obstacle_y = valid_ranges * np.sin(valid_angles)
        
        self.obstacles = list(zip(obstacle_x, obstacle_y))

    def image_callback(self, msg):
        """Process camera image for object detection"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detected_objects = self.detect_objects_in_image(self.latest_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def odom_callback(self, msg):
        """Update robot position"""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def joint_state_callback(self, msg):
        """Update joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def monitor_system(self):
        """Monitor system and execute tasks"""
        # Execute highest priority task if system is ready
        if self.task_queue:
            # Sort by priority (lower number = higher priority)
            self.task_queue.sort(key=lambda x: x['priority'])
            current_task = self.task_queue[0]
            
            if current_task['type'] == 'navigation':
                self.execute_navigation_task(current_task)
            elif current_task['type'] == 'manipulation':
                self.execute_manipulation_task(current_task)
            
            # Remove completed task
            self.task_queue.pop(0)

    def execute_navigation_task(self, task):
        """Execute navigation task"""
        location = task['location']
        
        # Define location coordinates
        location_map = {
            'kitchen': (5.0, 3.0, 0.0),
            'living_room': (2.0, 8.0, 0.0),
            'office': (8.0, 2.0, 0.0),
            'bedroom': (7.0, 7.0, 0.0),
            'charging_station': (0.0, 0.0, 0.0)
        }
        
        if location in location_map:
            x, y, theta = location_map[location]
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation.z = theta
            
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f'Navigating to {location}')
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def execute_manipulation_task(self, task):
        """Execute manipulation task"""
        obj_type = task['object_type']
        
        # Find the requested object in detected objects
        target_objects = [obj for obj in self.detected_objects if obj['name'] == obj_type]
        
        if target_objects:
            # Use the largest (closest) object
            target_obj = max(target_objects, key=lambda x: x['area'])
            
            # Create manipulation command
            manipulation_cmd = {
                'action': 'grasp_object',
                'object_info': target_obj,
                'approach_vector': [0, 0, -1],
                'grasp_type': 'top_grasp'
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(manipulation_cmd)
            self.manipulation_pub.publish(cmd_msg)
            
            self.get_logger().info(f'Attempting to grasp {obj_type}')
        else:
            self.get_logger().info(f'{obj_type} not detected, cannot grasp')

    def detect_objects_in_image(self, image):
        """Detect objects in image (same as previous exercise)"""
        objects = []
        
        for obj_name, (lower_color, upper_color) in self.object_colors.items():
            lower = np.array(lower_color)
            upper = np.array(upper_color)
            mask = cv2.inRange(image, lower, upper)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x, center_y = x + w // 2, y + h // 2
                    
                    img_width, img_height = image.shape[1], image.shape[0]
                    norm_x, norm_y = center_x / img_width, center_y / img_height
                    
                    objects.append({
                        'name': obj_name,
                        'center': {'x': center_x, 'y': center_y},
                        'normalized_center': {'x': norm_x, 'y': norm_y},
                        'bbox': {'x': x, 'y': y, 'width': w, 'height': h},
                        'area': area
                    })
        
        return objects

    def emergency_stop(self):
        """Emergency stop all robot motion"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Clear task queue
        self.task_queue.clear()
        
        self.get_logger().info('Emergency stop activated')

    def publish_system_status(self):
        """Publish comprehensive system status"""
        status = {
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_position': self.robot_position,
            'joint_positions': self.joint_positions,
            'detected_objects_count': len(self.detected_objects),
            'obstacles_count': len(self.obstacles),
            'task_queue_size': len(self.task_queue),
            'current_task': self.current_task
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CapstoneOrchestrator()
    
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

### Validation
- Test complete end-to-end functionality
- Verify all components work together
- Check system response to various commands
- Validate error handling and recovery

## Exercise 7: Performance and Optimization

### Objective
Analyze and optimize the integrated system for better performance.

### Task
Implement performance monitoring and optimization techniques for the integrated system.

### Implementation Steps
1. Add performance monitoring to track system metrics
2. Implement optimization techniques for resource usage
3. Add quality of service (QoS) configurations
4. Test system performance under various loads

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import time
import threading

class PerformanceOptimizer(Node):
    def __init__(self):
        super().__init__('performance_optimizer')
        
        # Create QoS profiles for different data types
        self.high_reliability_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.low_latency_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions with appropriate QoS
        self.critical_sub = self.create_subscription(
            String, 'critical_data', self.critical_callback, 
            self.high_reliability_qos)
        
        self.performance_sub = self.create_subscription(
            String, 'performance_data', self.performance_callback,
            self.low_latency_qos)
        
        # Publishers
        self.metrics_pub = self.create_publisher(String, 'performance_metrics', 10)
        
        # Performance tracking
        self.message_times = {}
        self.cpu_usage = []
        self.memory_usage = []
        
        # Start performance monitoring
        self.monitor_timer = self.create_timer(1.0, self.report_performance)
        
        self.get_logger().info('Performance Optimizer initialized')

    def critical_callback(self, msg):
        """Handle critical messages with performance tracking"""
        start_time = time.time()
        
        # Process critical message
        self.process_critical_data(msg)
        
        # Track processing time
        processing_time = time.time() - start_time
        self.message_times['critical'] = processing_time
        
        if processing_time > 0.1:  # More than 100ms
            self.get_logger().warn(f'Critical message processing took {processing_time:.3f}s')

    def performance_callback(self, msg):
        """Handle performance data"""
        # Track performance metrics
        pass

    def process_critical_data(self, msg):
        """Process critical data with optimization"""
        # Implementation would process critical data
        pass

    def report_performance(self):
        """Report system performance metrics"""
        metrics = {
            'timestamp': time.time(),
            'message_processing_times': self.message_times.copy(),
            'average_cpu_usage': sum(self.cpu_usage[-10:]) / min(len(self.cpu_usage), 10) if self.cpu_usage else 0,
            'latest_memory_usage': self.memory_usage[-1] if self.memory_usage else 0
        }
        
        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.metrics_pub.publish(metrics_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceOptimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Validation
- Monitor system performance metrics
- Verify optimization techniques are effective
- Test system under various load conditions

## Assessment Questions

1. How does component communication work in a ROS 2-based integrated system?
2. What are the key challenges in coordinating multiple robotic subsystems?
3. How do you handle state management in an integrated robotic system?
4. What safety considerations are important when integrating perception and navigation?
5. How would you optimize an integrated system for real-time performance?
6. What debugging strategies are effective for integrated robotic systems?
7. How do you handle error propagation between integrated components?
8. What role does system architecture play in successful integration?
9. How would you validate that all components are properly integrated?
10. What are the trade-offs between different integration approaches?

These exercises progressively build from basic component communication to a complete integrated system, providing hands-on experience with the challenges and solutions involved in system integration for the capstone project.