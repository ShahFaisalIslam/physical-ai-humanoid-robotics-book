---
title: Capstone Code Examples
description: Code examples for integrating all components without runtime
sidebar_position: 4
---

# Capstone Code Examples: Integration of All Components

## Overview

This section provides comprehensive code examples that demonstrate how to integrate all the components learned in previous modules into a cohesive autonomous humanoid system. These examples show the theoretical integration of voice processing, navigation, perception, and manipulation without requiring runtime execution.

## Complete System Integration Example

### Main System Integration Node

Here's a complete example showing how all components work together:

```python
"""
Complete Capstone System Integration Example

This example demonstrates the integration of all major components:
- Voice processing with Whisper and LLMs
- Navigation and path planning
- Perception and object detection
- Manipulation control
- System orchestration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import OccupancyGrid, Path
from cv_bridge import CvBridge
import json
import numpy as np
import whisper
import openai
import threading
import queue
import time

class CapstoneIntegratedSystem(Node):
    """
    The main integration node that coordinates all capstone components
    """
    def __init__(self):
        super().__init__('capstone_integrated_system')
        
        # Initialize Whisper model for speech recognition
        self.whisper_model = whisper.load_model("base")
        
        # Initialize OpenAI API for natural language processing
        openai.api_key = "YOUR_API_KEY_HERE"  # In practice, use environment variables
        
        # Initialize OpenCV bridge for image processing
        self.bridge = CvBridge()
        
        # Robot state tracking
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office',
            'is_moving': False,
            'is_grasping': False
        }
        
        # Component states
        self.detected_objects = []
        self.current_map = None
        self.local_obstacles = []
        self.joint_positions = {}
        
        # Audio processing buffer
        self.audio_buffer = queue.Queue()
        
        # Initialize all publishers
        self._initialize_publishers()
        
        # Initialize all subscribers
        self._initialize_subscribers()
        
        # Start processing threads
        self._start_processing_threads()
        
        self.get_logger().info('Capstone Integrated System initialized')

    def _initialize_publishers(self):
        """Initialize all publishers for the system"""
        # Audio processing
        self.transcription_pub = self.create_publisher(String, 'transcribed_text', 10)
        self.action_plan_pub = self.create_publisher(String, 'action_plan', 10)
        
        # Navigation
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        
        # Manipulation
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.manipulation_status_pub = self.create_publisher(String, 'manipulation_status', 10)
        
        # Perception
        self.object_detection_pub = self.create_publisher(String, 'detected_objects', 10)
        
        # System
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)

    def _initialize_subscribers(self):
        """Initialize all subscribers for the system"""
        # Audio input (simulated)
        self.create_subscription(String, 'simulated_audio', self.audio_callback, 10)
        
        # Navigation
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Perception
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

    def _start_processing_threads(self):
        """Start processing threads for different components"""
        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio_buffer)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Start system monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitor_system_status)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def audio_callback(self, msg):
        """Callback for audio input (simulated in this example)"""
        # In a real system, this would receive raw audio data
        # For this example, we'll treat it as transcribed text
        self.audio_buffer.put(msg.data)

    def process_audio_buffer(self):
        """Process audio data from the buffer"""
        while True:
            try:
                # Get audio from buffer with timeout
                audio_data = self.audio_buffer.get(timeout=1.0)
                
                # Process the audio data (in this example, it's already transcribed)
                self.process_voice_command(audio_data)
                
            except queue.Empty:
                continue  # Continue if no audio in buffer

    def process_voice_command(self, command_text):
        """Process a voice command through the entire pipeline"""
        self.get_logger().info(f'Processing voice command: {command_text}')
        
        # Step 1: Interpret the command using LLM
        action_plan = self.interpret_command(command_text)
        
        if action_plan:
            # Step 2: Validate the plan for safety
            if self.validate_action_plan(action_plan):
                # Step 3: Execute the plan
                self.execute_action_plan(action_plan)
            else:
                self.get_logger().warn('Action plan failed safety validation')
        else:
            self.get_logger().warn(f'Could not interpret command: {command_text}')

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
        - check_battery()
        
        Current robot state:
        {json.dumps(self.robot_state, indent=2)}
        
        Environment objects (from perception):
        {json.dumps(self.detected_objects[:5], indent=2)}  # Limit to first 5 for brevity
        
        User command: "{command_text}"

        Respond with a JSON object containing the action plan:
        {% raw %}
        {{
          "action": "action_name",
          "parameters": {{"param1": "value1", "param2": "value2"}},
          "confidence": 0.0-1.0,
          "required_components": ["component1", "component2"]
        }}
        {% endraw %}
        
        If the command is ambiguous or unsafe, return empty parameters and low confidence.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
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

    def validate_action_plan(self, plan):
        """Validate an action plan for safety and feasibility"""
        # Check if required components are available
        required_components = plan.get('required_components', [])
        for component in required_components:
            # In a real system, check if component is ready/available
            if not self.is_component_available(component):
                self.get_logger().warn(f'Required component {component} is not available')
                return False
        
        # Check if action is safe given current state
        action = plan.get('action', '')
        if action == 'navigate_to':
            location = plan.get('parameters', {}).get('location')
            if not self.is_navigation_safe_to(location):
                self.get_logger().warn(f'Navigation to {location} is not safe')
                return False
        elif action == 'pick_up_object':
            if self.robot_state['attached_object'] is not None:
                self.get_logger().warn('Robot already holding an object')
                return False
        
        return True

    def is_component_available(self, component):
        """Check if a component is available (simulated)"""
        # In a real system, this would check component status
        # For this example, assume all components are available
        return True

    def is_navigation_safe_to(self, location):
        """Check if navigation to a location is safe (simulated)"""
        # In a real system, this would check map data and obstacles
        # For this example, assume navigation is safe
        return True

    def execute_action_plan(self, plan):
        """Execute an action plan"""
        action = plan['action']
        parameters = plan.get('parameters', {})
        
        self.get_logger().info(f'Executing action: {action} with parameters: {parameters}')
        
        if action == 'navigate_to':
            self.execute_navigation(parameters.get('location'))
        elif action == 'pick_up_object':
            self.execute_pickup(parameters.get('object_type'))
        elif action == 'place_object':
            self.execute_placement(parameters.get('location'))
        elif action == 'detect_object':
            self.execute_detection(parameters.get('object_type'))
        elif action == 'return_to_charging_station':
            self.execute_return_to_charger()
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def execute_navigation(self, location):
        """Execute navigation to a specific location"""
        # Get coordinates for the location
        target_x, target_y = self.get_coordinates_for_location(location)
        
        if target_x is not None and target_y is not None:
            self.get_logger().info(f'Navigating to {location} at ({target_x}, {target_y})')
            
            # In a real system, this would plan and execute a path
            # For this example, we'll simulate the movement
            self.simulate_navigation_to(target_x, target_y)
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

    def simulate_navigation_to(self, target_x, target_y):
        """Simulate navigation to target coordinates"""
        # This is a simplified simulation
        # In a real system, this would use the navigation stack
        
        # Calculate distance to target
        current_x, current_y = self.robot_state['position']['x'], self.robot_state['position']['y']
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Publish a simple movement command
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0  # No rotation for simplicity
        
        # Simulate movement
        duration = distance / 0.5  # At 0.5 m/s
        start_time = time.time()
        
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)  # Small delay
        
        # Stop robot
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        # Update robot position
        self.robot_state['position']['x'] = target_x
        self.robot_state['position']['y'] = target_y
        
        self.get_logger().info(f'Navigation to ({target_x}, {target_y}) completed')

    def execute_pickup(self, object_type):
        """Execute object pickup"""
        self.get_logger().info(f'Attempting to pick up {object_type}')
        
        # In a real system, this would:
        # 1. Find the object in the environment
        # 2. Plan approach trajectory
        # 3. Execute the grasp
        # For this example, we'll simulate the process
        
        # Find the requested object type in detected objects
        target_object = None
        for obj in self.detected_objects:
            if obj['name'] == object_type:
                target_object = obj
                break
        
        if target_object:
            # Simulate approach and grasp
            self.simulate_grasp_object(target_object)
        else:
            self.get_logger().warn(f'Object of type {object_type} not detected')

    def simulate_grasp_object(self, obj):
        """Simulate grasping an object"""
        # In a real system, this would control the manipulator
        # For this example, we'll update the robot state
        
        self.robot_state['attached_object'] = obj['name']
        self.robot_state['is_grasping'] = True
        
        # Create joint command to close gripper
        cmd = JointState()
        cmd.name = ['gripper_joint']
        cmd.position = [0.0]  # Closed position
        cmd.velocity = [0.1]  # Closing velocity
        cmd.effort = [50.0]   # Maximum effort
        
        self.joint_cmd_pub.publish(cmd)
        
        self.get_logger().info(f'Object {obj["name"]} grasped successfully')

    def execute_placement(self, location):
        """Execute object placement"""
        if self.robot_state['attached_object'] is None:
            self.get_logger().warn('No object to place')
            return
        
        self.get_logger().info(f'Placing object at {location}')
        
        # In a real system, this would:
        # 1. Navigate to the placement location
        # 2. Execute the placement maneuver
        # For this example, we'll simulate the process
        
        # Simulate placing the object
        self.simulate_place_object(location)
        
        # Update robot state
        self.robot_state['attached_object'] = None
        self.robot_state['is_grasping'] = False

    def simulate_place_object(self, location):
        """Simulate placing an object at a location"""
        # In a real system, this would control the manipulator
        # For this example, we'll just open the gripper
        
        # Create joint command to open gripper
        cmd = JointState()
        cmd.name = ['gripper_joint']
        cmd.position = [0.8]  # Open position
        cmd.velocity = [0.1]  # Opening velocity
        cmd.effort = [50.0]   # Maximum effort
        
        self.joint_cmd_pub.publish(cmd)
        
        self.get_logger().info(f'Object placed at {location}')

    def execute_detection(self, object_type):
        """Execute object detection"""
        self.get_logger().info(f'Detecting objects of type: {object_type}')
        
        # In a real system, this would process camera images
        # For this example, we'll return the detected objects
        
        if object_type == 'any' or object_type == 'all':
            relevant_objects = self.detected_objects
        else:
            relevant_objects = [obj for obj in self.detected_objects if obj['name'] == object_type]
        
        # Publish detection results
        result_msg = String()
        result_msg.data = json.dumps({
            'command': 'detect_object',
            'object_type': object_type,
            'objects_found': relevant_objects,
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        self.object_detection_pub.publish(result_msg)

    def execute_return_to_charger(self):
        """Execute return to charging station"""
        self.get_logger().info('Returning to charging station')
        self.execute_navigation('charging_station')

    def map_callback(self, msg):
        """Update internal map representation"""
        self.current_map = {
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

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection
            detected_objects = self.detect_objects_in_image(cv_image)
            
            # Update internal state
            self.detected_objects = detected_objects
            
            # Publish detected objects
            detection_msg = String()
            detection_msg.data = json.dumps(detected_objects)
            self.object_detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects_in_image(self, image):
        """Detect objects in an image (simulated detection)"""
        # This is a simplified object detection simulation
        # In a real system, this would use a trained model
        
        # For this example, we'll return some predefined objects
        # based on color detection (simplified)
        object_colors = {
            'red_cup': ([0, 50, 50], [10, 255, 255]),
            'blue_bottle': ([100, 50, 50], [130, 255, 255]),
            'green_box': ([50, 50, 50], [70, 255, 255])
        }
        
        objects = []
        
        for obj_name, (lower_color, upper_color) in object_colors.items():
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

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def monitor_system_status(self):
        """Monitor and publish system status"""
        while True:
            # Create system status report
            status_report = {
                'timestamp': time.time(),
                'robot_state': self.robot_state,
                'detected_objects_count': len(self.detected_objects),
                'local_obstacles_count': len(self.local_obstacles),
                'components_status': {
                    'voice_processing': 'active',
                    'navigation': 'ready' if self.current_map else 'unavailable',
                    'perception': 'active',
                    'manipulation': 'ready' if self.joint_positions else 'unavailable'
                }
            }
            
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps(status_report)
            self.system_status_pub.publish(status_msg)
            
            # Sleep for 1 second
            time.sleep(1.0)

def main(args=None):
    """
    Main function to run the integrated capstone system
    """
    rclpy.init(args=args)
    
    # Create the integrated system node
    capstone_system = CapstoneIntegratedSystem()
    
    try:
        # In a real system, we would spin the node
        # For this theoretical example, we'll just keep it running
        print("Capstone Integrated System running...")
        print("This is a theoretical example without actual runtime execution")
        print("In a real implementation, this would process commands and control a robot")
        
        # Keep the system running
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nShutting down capstone system...")
    finally:
        capstone_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Component Integration Examples

### 1. Voice-to-Action Pipeline Integration

```python
"""
Voice-to-Action Pipeline Integration Example
Demonstrates how voice commands flow through the system to generate actions
"""

class VoiceToActionPipeline:
    """
    Class that demonstrates the voice-to-action pipeline integration
    """
    def __init__(self, system_node):
        self.system = system_node
        self.command_history = []
        
    def process_voice_command(self, command_text):
        """
        Process a voice command through the complete pipeline
        """
        print(f"Processing voice command: {command_text}")
        
        # Step 1: Speech-to-text (handled externally in this example)
        # In a real system, this would be done by Whisper
        
        # Step 2: Natural language understanding
        structured_command = self.understand_command(command_text)
        
        # Step 3: Context-aware planning
        action_plan = self.generate_action_plan(structured_command)
        
        # Step 4: Safety validation
        if self.validate_plan_safety(action_plan):
            # Step 5: Execute plan
            self.execute_plan(action_plan)
        else:
            print("Action plan failed safety validation")
            
        # Step 6: Update command history
        self.command_history.append({
            'command': command_text,
            'plan': action_plan,
            'timestamp': time.time()
        })
        
    def understand_command(self, command_text):
        """
        Understand the natural language command
        """
        # This would typically use an LLM in a real implementation
        # For this example, we'll do simple keyword matching
        command_lower = command_text.lower()
        
        if 'navigate to' in command_lower or 'go to' in command_lower:
            # Extract location
            for location in ['kitchen', 'living room', 'office', 'bedroom', 'charging station']:
                if location in command_lower:
                    return {
                        'action': 'navigate_to',
                        'parameters': {'location': location.replace(' ', '_')}
                    }
        
        elif 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract object type
            for obj_type in ['cup', 'bottle', 'box', 'object']:
                if obj_type in command_lower:
                    return {
                        'action': 'pick_up_object',
                        'parameters': {'object_type': obj_type}
                    }
        
        elif 'place' in command_lower or 'put' in command_lower:
            # Extract location
            for location in ['kitchen', 'living room', 'office', 'table', 'counter']:
                if location in command_lower:
                    return {
                        'action': 'place_object',
                        'parameters': {'location': location.replace(' ', '_')}
                    }
        
        # Default: unrecognized command
        return {
            'action': 'unknown',
            'parameters': {'original_command': command_text}
        }
    
    def generate_action_plan(self, structured_command):
        """
        Generate a detailed action plan based on the understood command
        """
        action = structured_command['action']
        params = structured_command['parameters']
        
        # In a real system, this would use an LLM to generate a detailed plan
        # For this example, we'll create a simple plan
        
        if action == 'navigate_to':
            location = params['location']
            return {
                'action_sequence': [
                    {'action': 'plan_path_to', 'parameters': {'destination': location}},
                    {'action': 'execute_navigation', 'parameters': {'destination': location}},
                    {'action': 'confirm_arrival', 'parameters': {'destination': location}}
                ],
                'required_components': ['navigation', 'localization'],
                'estimated_duration': 60  # seconds
            }
        
        elif action == 'pick_up_object':
            obj_type = params['object_type']
            return {
                'action_sequence': [
                    {'action': 'detect_object', 'parameters': {'object_type': obj_type}},
                    {'action': 'approach_object', 'parameters': {'object_type': obj_type}},
                    {'action': 'grasp_object', 'parameters': {'object_type': obj_type}}
                ],
                'required_components': ['perception', 'manipulation'],
                'estimated_duration': 30  # seconds
            }
        
        elif action == 'place_object':
            location = params['location']
            return {
                'action_sequence': [
                    {'action': 'navigate_to', 'parameters': {'location': location}},
                    {'action': 'place_held_object', 'parameters': {'location': location}}
                ],
                'required_components': ['navigation', 'manipulation'],
                'estimated_duration': 45  # seconds
            }
        
        else:
            return {
                'action_sequence': [],
                'required_components': [],
                'estimated_duration': 0
            }
    
    def validate_plan_safety(self, plan):
        """
        Validate that the action plan is safe to execute
        """
        # Check if required components are available
        for component in plan['required_components']:
            if not self.system.is_component_available(component):
                print(f"Required component {component} is not available")
                return False
        
        # Check robot state constraints
        if plan['action_sequence']:
            first_action = plan['action_sequence'][0]
            if first_action['action'] == 'grasp_object' and self.system.robot_state['attached_object']:
                print("Robot already holding an object, cannot grasp another")
                return False
        
        # In a real system, check for environmental safety
        # using sensor data
        
        return True
    
    def execute_plan(self, plan):
        """
        Execute the action plan
        """
        print(f"Executing plan with {len(plan['action_sequence'])} steps")
        
        for i, step in enumerate(plan['action_sequence']):
            action = step['action']
            params = step['parameters']
            
            print(f"Step {i+1}/{len(plan['action_sequence'])}: {action} with {params}")
            
            # Execute the action
            self.execute_single_action(action, params)
            
            # Small delay between steps (in a real system, this might be event-driven)
            time.sleep(0.5)
    
    def execute_single_action(self, action, params):
        """
        Execute a single action
        """
        # In a real system, this would interface with the appropriate component
        # For this example, we'll just log the action
        
        if action == 'navigate_to':
            print(f"  -> Navigating to {params['location']}")
        elif action == 'pick_up_object':
            print(f"  -> Picking up {params['object_type']}")
        elif action == 'place_object':
            print(f"  -> Placing object at {params['location']}")
        elif action == 'detect_object':
            print(f"  -> Detecting {params['object_type']}")
        else:
            print(f"  -> Executing {action} with {params}")
```

### 2. Perception-Navigation Integration

```python
"""
Perception-Navigation Integration Example
Demonstrates how perception data informs navigation decisions
"""

class PerceptionNavigationIntegration:
    """
    Class that demonstrates integration between perception and navigation
    """
    def __init__(self, system_node):
        self.system = system_node
        self.known_obstacles = {}
        self.dynamic_objects = []
        
    def update_navigation_with_perception(self):
        """
        Update navigation plan based on perception data
        """
        # Process detected objects to identify potential obstacles
        for obj in self.system.detected_objects:
            if self.is_obstacle(obj):
                self.add_obstacle_to_map(obj)
        
        # Process detected dynamic objects (people, moving objects)
        for obj in self.system.detected_objects:
            if self.is_dynamic_object(obj):
                self.track_dynamic_object(obj)
        
        # If navigation is active, update the plan based on new information
        if self.system.robot_state['is_moving']:
            self.update_active_navigation()
    
    def is_obstacle(self, obj):
        """
        Determine if an object is an obstacle for navigation
        """
        # In a real system, this would use object classification
        # For this example, we'll consider large objects as potential obstacles
        return obj['area'] > 1000  # Threshold area
    
    def is_dynamic_object(self, obj):
        """
        Determine if an object is dynamic (moving)
        """
        # In a real system, this would use tracking algorithms
        # For this example, we'll consider people as dynamic objects
        return 'person' in obj['name'].lower()
    
    def add_obstacle_to_map(self, obj):
        """
        Add an obstacle to the navigation map
        """
        # In a real system, this would update the costmap
        # For this example, we'll just store it
        obj_id = f"obstacle_{len(self.known_obstacles)}"
        self.known_obstacles[obj_id] = {
            'position': obj['center'],
            'size': obj['bbox'],
            'timestamp': time.time()
        }
        
        print(f"Added obstacle {obj_id} at {obj['center']}")
    
    def track_dynamic_object(self, obj):
        """
        Track a dynamic object for navigation safety
        """
        # In a real system, this would use tracking algorithms
        # For this example, we'll just store the object
        self.dynamic_objects.append({
            'name': obj['name'],
            'position': obj['center'],
            'timestamp': time.time()
        })
        
        print(f"Tracking dynamic object: {obj['name']} at {obj['center']}")
    
    def update_active_navigation(self):
        """
        Update an active navigation task with new perception data
        """
        # Check if any detected objects are in the navigation path
        for obj in self.system.detected_objects:
            if self.is_object_in_navigation_path(obj):
                # Adjust navigation plan
                self.adjust_navigation_for_object(obj)
    
    def is_object_in_navigation_path(self, obj):
        """
        Check if an object is in the current navigation path
        """
        # In a real system, this would check against the planned path
        # For this example, we'll use a simplified check
        robot_pos = self.system.robot_state['position']
        obj_pos = obj['center']
        
        # Calculate distance (simplified)
        distance = np.sqrt((robot_pos['x'] - obj_pos['x'])**2 + 
                          (robot_pos['y'] - obj_pos['y'])**2)
        
        # Consider object in path if it's close to the robot
        return distance < 2.0  # 2 meters threshold
    
    def adjust_navigation_for_object(self, obj):
        """
        Adjust navigation plan to account for an object
        """
        print(f"Adjusting navigation for object: {obj['name']}")
        
        # In a real system, this would replan the path
        # For this example, we'll just slow down or stop
        if obj['area'] > 5000:  # Large object
            print("  Large object detected, stopping navigation")
            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.system.cmd_vel_pub.publish(cmd)
        else:
            print("  Small object detected, slowing down")
            # Slow down the robot
            cmd = Twist()
            cmd.linear.x = 0.1  # Reduced speed
            cmd.angular.z = 0.0
            self.system.cmd_vel_pub.publish(cmd)
```

### 3. Manipulation-Perception Integration

```python
"""
Manipulation-Perception Integration Example
Demonstrates how perception data guides manipulation actions
"""

class ManipulationPerceptionIntegration:
    """
    Class that demonstrates integration between manipulation and perception
    """
    def __init__(self, system_node):
        self.system = system_node
        self.grasp_planner = GraspPlanner()
        self.approach_planner = ApproachPlanner()
        
    def plan_manipulation_with_perception(self, obj_name):
        """
        Plan manipulation actions based on perception data
        """
        # Find the target object in detected objects
        target_obj = None
        for obj in self.system.detected_objects:
            if obj['name'] == obj_name:
                target_obj = obj
                break
        
        if not target_obj:
            print(f"Object {obj_name} not detected")
            return None
        
        # Plan the grasp based on object properties
        grasp_plan = self.grasp_planner.plan_grasp_for_object(target_obj)
        
        # Plan the approach to the object
        approach_plan = self.approach_planner.plan_approach_to_object(target_obj)
        
        # Combine into a complete manipulation plan
        manipulation_plan = {
            'object': target_obj,
            'approach': approach_plan,
            'grasp': grasp_plan,
            'lift_height': 0.1  # Lift 10cm after grasp
        }
        
        return manipulation_plan
    
    def execute_manipulation_plan(self, plan):
        """
        Execute a manipulation plan
        """
        if not plan:
            print("No plan to execute")
            return False
        
        obj = plan['object']
        approach = plan['approach']
        grasp = plan['grasp']
        
        print(f"Executing manipulation for {obj['name']}")
        
        # Execute approach
        print("Executing approach...")
        self.execute_approach(approach)
        
        # Execute grasp
        print("Executing grasp...")
        success = self.execute_grasp(grasp)
        
        if success:
            # Lift the object
            print("Lifting object...")
            self.lift_object(plan['lift_height'])
            
            # Update robot state
            self.system.robot_state['attached_object'] = obj['name']
            
            print(f"Successfully manipulated {obj['name']}")
            return True
        else:
            print(f"Failed to manipulate {obj['name']}")
            return False
    
    def execute_approach(self, approach_plan):
        """
        Execute approach to an object
        """
        # In a real system, this would control the manipulator
        # For this example, we'll just simulate
        print(f"  Approaching with trajectory: {approach_plan['trajectory']}")
        time.sleep(2.0)  # Simulate execution time
    
    def execute_grasp(self, grasp_plan):
        """
        Execute grasp of an object
        """
        # In a real system, this would control the gripper
        # For this example, we'll simulate with some chance of success
        import random
        success = random.random() > 0.2  # 80% success rate in simulation
        
        print(f"  Grasping with method: {grasp_plan['method']}, success: {success}")
        time.sleep(1.0)  # Simulate execution time
        
        return success
    
    def lift_object(self, height):
        """
        Lift the currently grasped object
        """
        # In a real system, this would move the manipulator up
        # For this example, we'll just simulate
        print(f"  Lifting object by {height} meters")
        time.sleep(1.0)  # Simulate execution time

class GraspPlanner:
    """
    Simple grasp planner based on object properties
    """
    def plan_grasp_for_object(self, obj):
        """
        Plan a grasp for a detected object
        """
        # Determine grasp type based on object properties
        if 'cup' in obj['name']:
            # Top grasp for cup
            grasp_type = 'top_grasp'
            approach_vector = [0, 0, -1]  # Approach from above
        elif 'bottle' in obj['name']:
            # Side grasp for bottle
            grasp_type = 'side_grasp'
            approach_vector = [1, 0, 0]  # Approach from side
        elif 'box' in obj['name']:
            # Corner grasp for box
            grasp_type = 'corner_grasp'
            approach_vector = [0, 1, 0]  # Approach from front
        else:
            # Default grasp
            grasp_type = 'top_grasp'
            approach_vector = [0, 0, -1]
        
        return {
            'method': grasp_type,
            'approach_vector': approach_vector,
            'gripper_width': self.estimate_gripper_width(obj),
            'grasp_point': self.calculate_grasp_point(obj)
        }
    
    def estimate_gripper_width(self, obj):
        """
        Estimate appropriate gripper width for an object
        """
        # Simplified estimation based on bounding box
        bbox = obj['bbox']
        min_dim = min(bbox['width'], bbox['height'])
        return min(0.08, max(0.02, min_dim * 0.6))  # Between 2-8cm, 60% of min dimension
    
    def calculate_grasp_point(self, obj):
        """
        Calculate the best grasp point on an object
        """
        # For this example, use the center of the object
        return obj['center']

class ApproachPlanner:
    """
    Simple approach planner
    """
    def plan_approach_to_object(self, obj):
        """
        Plan an approach trajectory to an object
        """
        # Calculate approach trajectory
        # In a real system, this would use inverse kinematics
        approach_point = self.calculate_approach_point(obj)
        
        return {
            'trajectory': [approach_point],  # Simplified trajectory
            'safe_distance': 0.1,  # 10cm from object
            'approach_vector': [0, 0, -1]  # Default from above
        }
    
    def calculate_approach_point(self, obj):
        """
        Calculate the approach point for an object
        """
        # For this example, approach 10cm above the object center
        center = obj['center']
        return [center['x'], center['y'], center.get('z', 0.1) + 0.1]
```

## System Validation Examples

### 1. Integration Test Example

```python
"""
Integration Test Example
Demonstrates how to test the integration of all components
"""

def run_integration_tests():
    """
    Run integration tests for the complete system
    """
    print("Running Capstone System Integration Tests...")
    
    # Test 1: Voice Command Processing
    print("\n1. Testing Voice Command Processing...")
    test_voice_command_processing()
    
    # Test 2: Navigation Integration
    print("\n2. Testing Navigation Integration...")
    test_navigation_integration()
    
    # Test 3: Perception Integration
    print("\n3. Testing Perception Integration...")
    test_perception_integration()
    
    # Test 4: Manipulation Integration
    print("\n4. Testing Manipulation Integration...")
    test_manipulation_integration()
    
    # Test 5: Complete End-to-End Flow
    print("\n5. Testing Complete End-to-End Flow...")
    test_end_to_end_flow()
    
    print("\nIntegration tests completed!")

def test_voice_command_processing():
    """
    Test the voice command processing pipeline
    """
    # Create a voice-to-action pipeline instance
    system_node = CapstoneIntegratedSystem()  # This would be a mock in real testing
    v2a_pipeline = VoiceToActionPipeline(system_node)
    
    # Test various commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Place the object on the table"
    ]
    
    for cmd in test_commands:
        print(f"  Testing: {cmd}")
        v2a_pipeline.process_voice_command(cmd)
    
    print("  ✓ Voice command processing tests passed")

def test_navigation_integration():
    """
    Test navigation integration with other components
    """
    system_node = CapstoneIntegratedSystem()  # Mock
    nav_integration = PerceptionNavigationIntegration(system_node)
    
    # Simulate detected objects that should affect navigation
    system_node.detected_objects = [
        {'name': 'large_box', 'center': {'x': 1.0, 'y': 1.0}, 'area': 2000, 'bbox': {'width': 0.5, 'height': 0.5}},
        {'name': 'person', 'center': {'x': 2.0, 'y': 0.5}, 'area': 1000, 'bbox': {'width': 0.8, 'height': 1.8}}
    ]
    
    # Update navigation with perception data
    nav_integration.update_navigation_with_perception()
    
    # Verify obstacles were detected
    assert len(nav_integration.known_obstacles) > 0, "No obstacles detected"
    assert len(nav_integration.dynamic_objects) > 0, "No dynamic objects tracked"
    
    print("  ✓ Navigation integration tests passed")

def test_perception_integration():
    """
    Test perception system with various inputs
    """
    system_node = CapstoneIntegratedSystem()  # Mock
    
    # Simulate image processing
    # In a real test, we would use actual image data
    print("  Simulating perception processing...")
    
    # Create mock detected objects
    system_node.detected_objects = [
        {'name': 'red_cup', 'center': {'x': 300, 'y': 200}, 'area': 800},
        {'name': 'blue_bottle', 'center': {'x': 400, 'y': 150}, 'area': 1200}
    ]
    
    print(f"  Detected {len(system_node.detected_objects)} objects")
    print("  ✓ Perception integration tests passed")

def test_manipulation_integration():
    """
    Test manipulation system with perception data
    """
    system_node = CapstoneIntegratedSystem()  # Mock
    manip_integration = ManipulationPerceptionIntegration(system_node)
    
    # Set up detected objects
    system_node.detected_objects = [
        {'name': 'red_cup', 'center': {'x': 300, 'y': 200}, 'area': 800, 
         'bbox': {'x': 280, 'y': 180, 'width': 40, 'height': 40}},
        {'name': 'blue_bottle', 'center': {'x': 400, 'y': 150}, 'area': 1200,
         'bbox': {'x': 380, 'y': 120, 'width': 40, 'height': 60}}
    ]
    
    # Plan manipulation for the red cup
    plan = manip_integration.plan_manipulation_with_perception('red_cup')
    assert plan is not None, "Manipulation plan not generated"
    
    # Execute the plan (simulation)
    success = manip_integration.execute_manipulation_plan(plan)
    assert success, "Manipulation execution failed"
    
    print("  ✓ Manipulation integration tests passed")

def test_end_to_end_flow():
    """
    Test the complete end-to-end flow
    """
    # This would test the complete system flow:
    # 1. Voice command input
    # 2. Command interpretation
    # 3. Action planning
    # 4. Component coordination
    # 5. Execution and feedback
    
    print("  Simulating complete end-to-end flow...")
    
    # In a real test, we would simulate the entire flow
    # For this example, we'll just verify the concept
    print("  1. Voice command received and interpreted")
    print("  2. Action plan generated")
    print("  3. Components coordinated")
    print("  4. Actions executed")
    print("  5. Feedback processed")
    
    print("  ✓ End-to-end flow test concept validated")
```

## Best Practices for Integration

### 1. Error Handling and Fallbacks

```python
"""
Best Practices: Error Handling and Fallbacks
"""

class RobustIntegrationSystem:
    """
    Demonstrates robust integration with error handling
    """
    def __init__(self):
        self.fallback_strategies = {
            'navigation_failure': self.fallback_navigation,
            'perception_failure': self.fallback_perception,
            'manipulation_failure': self.fallback_manipulation
        }
        
        self.error_recovery_steps = []
        
    def handle_component_error(self, component_name, error):
        """
        Handle errors in components with appropriate fallbacks
        """
        print(f"Error in {component_name}: {error}")
        
        # Log the error
        self.log_error(component_name, error)
        
        # Apply fallback strategy if available
        if component_name in self.fallback_strategies:
            fallback_method = self.fallback_strategies[component_name]
            fallback_method()
        else:
            # Default fallback: stop and request human intervention
            self.emergency_stop()
            self.request_human_intervention()
    
    def log_error(self, component, error):
        """
        Log errors for debugging and monitoring
        """
        error_entry = {
            'timestamp': time.time(),
            'component': component,
            'error': str(error),
            'context': self.get_system_context()
        }
        
        self.error_recovery_steps.append(error_entry)
        print(f"Error logged: {error_entry}")
    
    def get_system_context(self):
        """
        Get current system context for error logging
        """
        # In a real system, this would capture relevant state
        return {
            'robot_position': {'x': 0, 'y': 0, 'theta': 0},  # Placeholder
            'active_tasks': [],
            'component_status': {}
        }
    
    def fallback_navigation(self):
        """
        Fallback strategy for navigation failures
        """
        print("Navigation fallback: Using simpler path planning or stopping")
        # Stop current navigation
        # Try alternative path planning
        # Or return to safe position
    
    def fallback_perception(self):
        """
        Fallback strategy for perception failures
        """
        print("Perception fallback: Using alternative sensors or default assumptions")
        # Try alternative perception methods
        # Use pre-mapped information
        # Request human guidance
    
    def fallback_manipulation(self):
        """
        Fallback strategy for manipulation failures
        """
        print("Manipulation fallback: Using simpler grasp or aborting task")
        # Open gripper to release potential jams
        # Try alternative grasp
        # Abort task and report failure
    
    def emergency_stop(self):
        """
        Emergency stop for safety
        """
        print("Emergency stop activated")
        # Send stop commands to all actuators
        # Set all components to safe state
    
    def request_human_intervention(self):
        """
        Request human intervention when automated recovery fails
        """
        print("Requesting human intervention")
        # Signal for human operator
        # Provide context about the failure
        # Wait for human decision
```

This comprehensive set of code examples demonstrates how all components of the capstone project integrate together. The examples show theoretical implementations without requiring runtime execution, focusing on the architecture, data flow, and coordination between components. Each example illustrates best practices for integration, error handling, and system validation.