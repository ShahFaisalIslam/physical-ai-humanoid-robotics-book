---
title: LLM Cognitive Planning Code Examples
description: Practical code examples for implementing LLM-based cognitive planning in robotics
sidebar_position: 5
---

# LLM Cognitive Planning Code Examples

## Overview

This section provides practical code examples for implementing Large Language Model (LLM) based cognitive planning in robotics applications. These examples demonstrate how to use LLMs to interpret natural language commands and generate executable robotic plans.

## Basic LLM Integration

### Simple LLM Planner Node

```python
import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SimpleLLMPlanner(Node):
    def __init__(self):
        super().__init__('simple_llm_planner')
        
        # Set OpenAI API key (in practice, use environment variables)
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        self.get_logger().info('Simple LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command and generate robot actions"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Generate plan using LLM
        plan = self.generate_plan(command)
        
        if plan:
            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().warn('Could not generate a plan for the command')

    def generate_plan(self, command):
        """Generate a step-by-step plan using an LLM"""
        prompt = f"""
        You are a cognitive planner for a TurtleBot3 robot operating in an indoor office environment.
        The robot can perform the following actions:
        - move_forward(distance_in_meters)
        - move_backward(distance_in_meters)
        - turn_left(degrees)
        - turn_right(degrees)
        - stop()
        - pick_up_object()
        - place_object()
        - detect_object(object_type)
        - check_battery()
        
        Current robot state:
        {json.dumps(self.robot_state, indent=2)}
        
        Human command: "{command}"
        
        Respond with a JSON object containing a step-by-step plan. Each step should be a dictionary with an 'action' and 'parameters'.
        Format:
        {{
          "plan": [
            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
            {{"action": "pick_up_object", "parameters": {{}}}}
          ]
        }}
        
        Be specific with parameters and ensure the plan is executable.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )
            
            # Extract the plan from the response
            content = response.choices[0].message['content'].strip()
            
            # Extract JSON from the response
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan_data = json.loads(json_str)
                return plan_data['plan']
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {content}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {str(e)}')
            return None

    def execute_plan(self, plan):
        """Execute the plan step by step"""
        for step in plan:
            action = step['action']
            params = step.get('parameters', {})
            
            self.get_logger().info(f'Executing action: {action} with params: {params}')
            
            if action == 'move_forward':
                self.move_forward(params.get('distance', 1.0))
            elif action == 'move_backward':
                self.move_backward(params.get('distance', 1.0))
            elif action == 'turn_left':
                self.turn_left(params.get('degrees', 90))
            elif action == 'turn_right':
                self.turn_right(params.get('degrees', 90))
            elif action == 'stop':
                self.stop_robot()
            elif action == 'pick_up_object':
                self.pick_up_object()
            elif action == 'place_object':
                self.place_object()
            elif action == 'detect_object':
                self.detect_object(params.get('object_type', 'any'))
            elif action == 'check_battery':
                self.check_battery()
            else:
                self.get_logger().warn(f'Unknown action: {action}')
            
            # Add a small delay between actions
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def move_backward(self, distance):
        """Move the robot backward by the specified distance"""
        msg = Twist()
        msg.linear.x = -0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_left(self, degrees):
        """Turn the robot left by the specified degrees"""
        msg = Twist()
        msg.angular.z = 0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_right(self, degrees):
        """Turn the robot right by the specified degrees"""
        msg = Twist()
        msg.angular.z = -0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def pick_up_object(self):
        """Simulate picking up an object"""
        self.get_logger().info('Picking up object...')
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

## Advanced LLM Integration with Context

### Context-Aware LLM Planner

```python
import rclpy
from rclpy.node import Node
import openai
import json
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from collections import deque

class ContextAwareLLMPlanner(Node):
    def __init__(self):
        super().__init__('context_aware_llm_planner')
        
        # Set OpenAI API key
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state with more details
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office',
            'last_seen_objects': [],
            'is_moving': False,
            'safety_status': 'safe'
        }
        
        # Maintain conversation history
        self.conversation_history = deque(maxlen=10)
        
        # Store latest sensor data
        self.latest_scan = None
        
        self.get_logger().info('Context-Aware LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command with context"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Add to conversation history
        self.conversation_history.append({
            'type': 'user',
            'content': command,
            'timestamp': time.time()
        })
        
        # Generate plan using LLM with context
        plan = self.generate_plan_with_context(command)
        
        if plan:
            # Execute the plan
            success = self.execute_plan_safely(plan)
            
            # Add result to conversation history
            self.conversation_history.append({
                'type': 'system',
                'content': f"Plan executed successfully: {success}",
                'timestamp': time.time()
            })
        else:
            self.get_logger().warn('Could not generate a plan for the command')

    def scan_callback(self, msg):
        """Update robot state with latest sensor data"""
        self.latest_scan = msg
        # Update safety status based on scan data
        self.update_safety_status()

    def update_safety_status(self):
        """Update robot safety status based on sensor data"""
        if self.latest_scan is not None:
            # Check for obstacles in front of the robot (simplified)
            front_range = self.latest_scan.ranges[len(self.latest_scan.ranges)//2]  # Front reading
            if front_range < 0.5:  # Obstacle within 0.5m
                self.robot_state['safety_status'] = 'obstacle_detected'
            else:
                self.robot_state['safety_status'] = 'safe'

    def generate_plan_with_context(self, command):
        """Generate a plan considering conversation history and context"""
        # Get recent conversation context
        context = list(self.conversation_history)[-5:]  # Last 5 interactions
        
        prompt = f"""
        You are a cognitive planner for a TurtleBot3 robot operating in an indoor office environment.
        The robot can perform the following actions:
        - move_forward(distance_in_meters)
        - move_backward(distance_in_meters)
        - turn_left(degrees)
        - turn_right(degrees)
        - stop()
        - pick_up_object()
        - place_object()
        - detect_object(object_type)
        - check_battery()
        
        Current robot state:
        {json.dumps(self.robot_state, indent=2)}
        
        Recent conversation history:
        {json.dumps(context, indent=2)}
        
        Current sensor data (LIDAR scan):
        - Min range: {min(self.latest_scan.ranges) if self.latest_scan else 'N/A'}
        - Max range: {max(self.latest_scan.ranges) if self.latest_scan else 'N/A'}
        - Front range: {self.latest_scan.ranges[len(self.latest_scan.ranges)//2] if self.latest_scan else 'N/A' }
        
        Human command: "{command}"
        
        Respond with a JSON object containing a step-by-step plan. Each step should be a dictionary with an 'action' and 'parameters'.
        Format:
        {{
          "plan": [
            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
            {{"action": "pick_up_object", "parameters": {{}}}}
          ]
        }}
        
        Consider the conversation context and current sensor data when generating the plan.
        Be specific with parameters and ensure the plan is executable and safe.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",  # Using GPT-4 for more complex reasoning
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=700
            )
            
            content = response.choices[0].message['content'].strip()
            
            # Extract JSON from the response
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan_data = json.loads(json_str)
                return plan_data['plan']
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {content}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {str(e)}')
            return None

    def execute_plan_safely(self, plan):
        """Execute the plan with safety checks"""
        for step in plan:
            # Check if environment is still safe
            if not self.check_environment_safety():
                self.get_logger().error('Environment is unsafe, stopping execution')
                self.stop_robot()
                return False
            
            # Execute the action
            self.execute_single_action(step)
            
            # Small delay between actions for safety
            time.sleep(0.5)
        
        return True

    def check_environment_safety(self):
        """Check if the environment is safe for movement"""
        # In a real robot, this would check sensor data
        # for obstacles, people, etc.
        return self.robot_state['safety_status'] == 'safe'

    def execute_single_action(self, step):
        """Execute a single action from the plan"""
        action = step['action']
        params = step.get('parameters', {})
        
        self.get_logger().info(f'Executing action: {action} with params: {params}')
        
        if action == 'move_forward':
            self.move_forward(params.get('distance', 1.0))
        elif action == 'move_backward':
            self.move_backward(params.get('distance', 1.0))
        elif action == 'turn_left':
            self.turn_left(params.get('degrees', 90))
        elif action == 'turn_right':
            self.turn_right(params.get('degrees', 90))
        elif action == 'stop':
            self.stop_robot()
        elif action == 'pick_up_object':
            self.pick_up_object()
        elif action == 'place_object':
            self.place_object()
        elif action == 'detect_object':
            self.detect_object(params.get('object_type', 'any'))
        elif action == 'check_battery':
            self.check_battery()
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    # Implementation of movement methods would be similar to SimpleLLMPlanner
    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def move_backward(self, distance):
        """Move the robot backward by the specified distance"""
        msg = Twist()
        msg.linear.x = -0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_left(self, degrees):
        """Turn the robot left by the specified degrees"""
        msg = Twist()
        msg.angular.z = 0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_right(self, degrees):
        """Turn the robot right by the specified degrees"""
        msg = Twist()
        msg.angular.z = -0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.robot_state['is_moving'] = False

    def pick_up_object(self):
        """Simulate picking up an object"""
        self.get_logger().info('Picking up object...')
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

## Local LLM Integration

### Running Open-Source LLMs Locally

```python
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
import torch
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class LocalLLMPlanner(Node):
    def __init__(self):
        super().__init__('local_llm_planner')
        
        # Load a pre-trained model (e.g., Llama, Mistral, or other)
        model_name = "microsoft/DialoGPT-medium"  # Example model - replace with appropriate model
        
        try:
            # Initialize the tokenizer and model
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModelForCausalLM.from_pretrained(model_name)
            
            # Set pad token if it doesn't exist
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token
            
            # Initialize text generation pipeline
            self.generator = pipeline(
                'text-generation',
                model=self.model,
                tokenizer=self.tokenizer,
                device=0 if torch.cuda.is_available() else -1  # Use GPU if available
            )
            
            self.get_logger().info(f'Loaded model: {model_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model {model_name}: {e}')
            return
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        self.get_logger().info('Local LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command using local LLM"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Generate plan using local LLM
        plan = self.generate_plan_local(command)
        
        if plan:
            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().warn('Could not generate a plan for the command')

    def generate_plan_local(self, command):
        """Generate a plan using a local LLM"""
        prompt = f"""
        You are a cognitive planner for a TurtleBot3 robot operating in an indoor office environment.
        The robot can perform the following actions:
        - move_forward(distance_in_meters)
        - move_backward(distance_in_meters)
        - turn_left(degrees)
        - turn_right(degrees)
        - stop()
        - pick_up_object()
        - place_object()
        - detect_object(object_type)
        - check_battery()
        
        Current robot state:
        {json.dumps(self.robot_state, indent=2)}
        
        Human command: "{command}"
        
        Respond with a JSON object containing a step-by-step plan. Each step should be a dictionary with an 'action' and 'parameters'.
        Format:
        {{
          "plan": [
            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
            {{"action": "pick_up_object", "parameters": {{}}}}
          ]
        }}
        
        Be specific with parameters and ensure the plan is executable.
        """
        
        try:
            # Generate response using the local model
            response = self.generator(
                prompt,
                max_length=500,
                num_return_sequences=1,
                temperature=0.3,
                pad_token_id=self.tokenizer.eos_token_id
            )
            
            content = response[0]['generated_text'][len(prompt):].strip()
            
            # Extract JSON from the response
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan_data = json.loads(json_str)
                return plan_data['plan']
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {content}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error generating plan with local LLM: {str(e)}')
            return None

    def execute_plan(self, plan):
        """Execute the plan step by step"""
        for step in plan:
            action = step['action']
            params = step.get('parameters', {})
            
            self.get_logger().info(f'Executing action: {action} with params: {params}')
            
            if action == 'move_forward':
                self.move_forward(params.get('distance', 1.0))
            elif action == 'move_backward':
                self.move_backward(params.get('distance', 1.0))
            elif action == 'turn_left':
                self.turn_left(params.get('degrees', 90))
            elif action == 'turn_right':
                self.turn_right(params.get('degrees', 90))
            elif action == 'stop':
                self.stop_robot()
            elif action == 'pick_up_object':
                self.pick_up_object()
            elif action == 'place_object':
                self.place_object()
            elif action == 'detect_object':
                self.detect_object(params.get('object_type', 'any'))
            elif action == 'check_battery':
                self.check_battery()
            else:
                self.get_logger().warn(f'Unknown action: {action}')
            
            # Add a small delay between actions
            time.sleep(0.5)

    # Movement methods would be similar to previous examples
    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def move_backward(self, distance):
        """Move the robot backward by the specified distance"""
        msg = Twist()
        msg.linear.x = -0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_left(self, degrees):
        """Turn the robot left by the specified degrees"""
        msg = Twist()
        msg.angular.z = 0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_right(self, degrees):
        """Turn the robot right by the specified degrees"""
        msg = Twist()
        msg.angular.z = -0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def pick_up_object(self):
        """Simulate picking up an object"""
        self.get_logger().info('Picking up object...')
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

## Multi-Modal LLM Integration

### Combining Vision and Language for Complex Tasks

```python
import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64

class MultiModalLLMPlanner(Node):
    def __init__(self):
        super().__init__('multimodal_llm_planner')
        
        # Set OpenAI API key
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        # Store latest image
        self.latest_image = None
        
        self.get_logger().info('Multi-Modal LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command with visual context"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Generate plan using LLM with both text and image
        plan = self.generate_plan_with_vision(command)
        
        if plan:
            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().warn('Could not generate a plan for the command')

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def encode_image(self, image):
        """Encode OpenCV image as base64 string"""
        _, buffer = cv2.imencode('.jpg', image)
        image_bytes = buffer.tobytes()
        base64_image = base64.b64encode(image_bytes).decode('utf-8')
        return base64_image

    def generate_plan_with_vision(self, command):
        """Generate a plan using both text command and visual input"""
        # Encode the latest image
        if self.latest_image is not None:
            # Resize image to save on tokens (GPT-4 Vision has limits)
            height, width = self.latest_image.shape[:2]
            if height > 480 or width > 640:
                # Maintain aspect ratio
                scale = min(480/height, 640/width)
                new_width = int(width * scale)
                new_height = int(height * scale)
                resized_image = cv2.resize(self.latest_image, (new_width, new_height))
            else:
                resized_image = self.latest_image
            
            base64_image = self.encode_image(resized_image)
        else:
            self.get_logger().warn('No image available for visual context')
            return None

        # Create the message with both text and image
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": f"""
                        You are a cognitive planner for a TurtleBot3 robot operating in an indoor office environment.
                        The robot can perform the following actions:
                        - move_forward(distance_in_meters)
                        - move_backward(distance_in_meters)
                        - turn_left(degrees)
                        - turn_right(degrees)
                        - stop()
                        - pick_up_object()
                        - place_object()
                        - detect_object(object_type)
                        - check_battery()

                        Current robot state:
                        {json.dumps(self.robot_state, indent=2)}

                        Human command: "{command}"

                        The image shows the robot's current view. Use this visual information to understand the environment and execute the command.

                        Respond with a JSON object containing a step-by-step plan. Each step should be a dictionary with an 'action' and 'parameters'.
                        Format:
                        {{
                          "plan": [
                            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
                            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
                            {{"action": "pick_up_object", "parameters": {{}}}}
                          ]
                        }}

                        Be specific with parameters and ensure the plan is executable.
                        """
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}"
                        }
                    }
                ]
            }
        ]

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4-vision-preview",  # Use GPT-4 with vision capabilities
                messages=messages,
                temperature=0.3,
                max_tokens=700
            )
            
            content = response.choices[0].message['content'].strip()
            
            # Extract JSON from the response
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan_data = json.loads(json_str)
                return plan_data['plan']
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {content}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling multimodal LLM: {str(e)}')
            return None

    def execute_plan(self, plan):
        """Execute the plan step by step"""
        for step in plan:
            action = step['action']
            params = step.get('parameters', {})
            
            self.get_logger().info(f'Executing action: {action} with params: {params}')
            
            if action == 'move_forward':
                self.move_forward(params.get('distance', 1.0))
            elif action == 'move_backward':
                self.move_backward(params.get('distance', 1.0))
            elif action == 'turn_left':
                self.turn_left(params.get('degrees', 90))
            elif action == 'turn_right':
                self.turn_right(params.get('degrees', 90))
            elif action == 'stop':
                self.stop_robot()
            elif action == 'pick_up_object':
                self.pick_up_object()
            elif action == 'place_object':
                self.place_object()
            elif action == 'detect_object':
                self.detect_object(params.get('object_type', 'any'))
            elif action == 'check_battery':
                self.check_battery()
            else:
                self.get_logger().warn(f'Unknown action: {action}')
            
            # Add a small delay between actions
            time.sleep(0.5)

    # Movement methods would be similar to previous examples
    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def move_backward(self, distance):
        """Move the robot backward by the specified distance"""
        msg = Twist()
        msg.linear.x = -0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_left(self, degrees):
        """Turn the robot left by the specified degrees"""
        msg = Twist()
        msg.angular.z = 0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_right(self, degrees):
        """Turn the robot right by the specified degrees"""
        msg = Twist()
        msg.angular.z = -0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def pick_up_object(self):
        """Simulate picking up an object"""
        self.get_logger().info('Picking up object...')
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

## Error Handling and Fallback Strategies

### Robust LLM Planner with Fallbacks

```python
import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import random

class RobustLLMPlanner(Node):
    def __init__(self):
        super().__init__('robust_llm_planner')
        
        # Set OpenAI API key
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        # Fallback command mappings for common requests
        self.fallback_commands = {
            'move forward': [{'action': 'move_forward', 'parameters': {'distance': 1.0}}],
            'go forward': [{'action': 'move_forward', 'parameters': {'distance': 1.0}}],
            'move back': [{'action': 'move_backward', 'parameters': {'distance': 1.0}}],
            'go back': [{'action': 'move_backward', 'parameters': {'distance': 1.0}}],
            'turn left': [{'action': 'turn_left', 'parameters': {'degrees': 90}}],
            'turn right': [{'action': 'turn_right', 'parameters': {'degrees': 90}}],
            'stop': [{'action': 'stop', 'parameters': {}}],
            'halt': [{'action': 'stop', 'parameters': {}}]
        }
        
        # Statistics for monitoring
        self.llm_calls = 0
        self.llm_failures = 0
        self.fallback_uses = 0
        
        # Start statistics timer
        self.stats_timer = self.create_timer(60.0, self.log_statistics)
        
        self.get_logger().info('Robust LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command with error handling and fallbacks"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Try to generate plan with LLM first
        plan = self.generate_plan_with_error_handling(command)
        
        if plan:
            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().warn('LLM failed to generate plan, trying fallback')
            
            # Try fallback method
            fallback_plan = self.get_fallback_plan(command)
            if fallback_plan:
                self.fallback_uses += 1
                self.get_logger().info('Using fallback plan')
                self.execute_plan(fallback_plan)
            else:
                self.get_logger().error('Both LLM and fallback methods failed')
                # Provide feedback to user
                feedback_msg = String()
                feedback_msg.data = f"Could not understand command: {command}"
                self.action_pub.publish(feedback_msg)

    def generate_plan_with_error_handling(self, command):
        """Generate a plan with comprehensive error handling"""
        self.llm_calls += 1
        
        try:
            prompt = f"""
            You are a cognitive planner for a TurtleBot3 robot operating in an indoor office environment.
            The robot can perform the following actions:
            - move_forward(distance_in_meters)
            - move_backward(distance_in_meters)
            - turn_left(degrees)
            - turn_right(degrees)
            - stop()
            - pick_up_object()
            - place_object()
            - detect_object(object_type)
            - check_battery()

            Current robot state:
            {json.dumps(self.robot_state, indent=2)}

            Human command: "{command}"

            Respond with a JSON object containing a step-by-step plan. Each step should be a dictionary with an 'action' and 'parameters'.
            Format:
            {{
              "plan": [
                {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
                {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
                {{"action": "pick_up_object", "parameters": {{}}}}
              ]
            }}

            Be specific with parameters and ensure the plan is executable.
            """
            
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500,
                timeout=15  # 15 second timeout
            )
            
            content = response.choices[0].message['content'].strip()
            
            # Extract JSON from the response
            start_idx = content.find('{')
            end_idx = content.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan_data = json.loads(json_str)
                return plan_data['plan']
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {content}')
                return None
                
        except openai.error.RateLimitError:
            self.get_logger().error('OpenAI rate limit exceeded')
            self.llm_failures += 1
            return None
        except openai.error.AuthenticationError:
            self.get_logger().error('OpenAI authentication failed')
            self.llm_failures += 1
            return None
        except openai.error.APIConnectionError:
            self.get_logger().error('OpenAI connection error')
            self.llm_failures += 1
            return None
        except openai.error.Timeout:
            self.get_logger().error('OpenAI request timed out')
            self.llm_failures += 1
            return None
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON response from LLM')
            self.llm_failures += 1
            return None
        except Exception as e:
            self.get_logger().error(f'Unexpected error in LLM call: {str(e)}')
            self.llm_failures += 1
            return None

    def get_fallback_plan(self, command):
        """Get a plan using simple keyword matching as fallback"""
        command_lower = command.lower()
        
        # Check for exact matches first
        if command_lower in self.fallback_commands:
            return self.fallback_commands[command_lower]
        
        # Check for partial matches
        for key, plan in self.fallback_commands.items():
            if key in command_lower:
                return plan
        
        # If no match found, try to parse simple commands
        if 'forward' in command_lower or 'ahead' in command_lower:
            # Extract distance if mentioned
            distance = 1.0  # default
            for word in command_lower.split():
                if word.replace('.', '').isdigit():
                    distance = float(word)
                    break
            return [{'action': 'move_forward', 'parameters': {'distance': distance}}]
        elif 'backward' in command_lower or 'back' in command_lower:
            distance = 1.0  # default
            for word in command_lower.split():
                if word.replace('.', '').isdigit():
                    distance = float(word)
                    break
            return [{'action': 'move_backward', 'parameters': {'distance': distance}}]
        elif 'left' in command_lower:
            degrees = 90  # default
            for word in command_lower.split():
                if word.replace('.', '').isdigit():
                    degrees = float(word)
                    break
            return [{'action': 'turn_left', 'parameters': {'degrees': degrees}}]
        elif 'right' in command_lower:
            degrees = 90  # default
            for word in command_lower.split():
                if word.replace('.', '').isdigit():
                    degrees = float(word)
                    break
            return [{'action': 'turn_right', 'parameters': {'degrees': degrees}}]
        
        # No fallback plan available
        return None

    def execute_plan(self, plan):
        """Execute the plan step by step"""
        for step in plan:
            action = step['action']
            params = step.get('parameters', {})
            
            self.get_logger().info(f'Executing action: {action} with params: {params}')
            
            if action == 'move_forward':
                self.move_forward(params.get('distance', 1.0))
            elif action == 'move_backward':
                self.move_backward(params.get('distance', 1.0))
            elif action == 'turn_left':
                self.turn_left(params.get('degrees', 90))
            elif action == 'turn_right':
                self.turn_right(params.get('degrees', 90))
            elif action == 'stop':
                self.stop_robot()
            elif action == 'pick_up_object':
                self.pick_up_object()
            elif action == 'place_object':
                self.place_object()
            elif action == 'detect_object':
                self.detect_object(params.get('object_type', 'any'))
            elif action == 'check_battery':
                self.check_battery()
            else:
                self.get_logger().warn(f'Unknown action: {action}')
            
            # Add a small delay between actions
            time.sleep(0.5)

    def log_statistics(self):
        """Log LLM usage statistics"""
        success_rate = 0
        if self.llm_calls > 0:
            success_rate = 100 * (self.llm_calls - self.llm_failures) / self.llm_calls
            
        self.get_logger().info(
            f'LLM Statistics: {self.llm_calls} calls, '
            f'{self.llm_failures} failures, '
            f'{self.fallback_uses} fallbacks used, '
            f'{success_rate:.1f}% success rate'
        )

    # Movement methods would be similar to previous examples
    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def move_backward(self, distance):
        """Move the robot backward by the specified distance"""
        msg = Twist()
        msg.linear.x = -0.2  # m/s
        duration = distance / 0.2
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_left(self, degrees):
        """Turn the robot left by the specified degrees"""
        msg = Twist()
        msg.angular.z = 0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def turn_right(self, degrees):
        """Turn the robot right by the specified degrees"""
        msg = Twist()
        msg.angular.z = -0.5  # rad/s
        duration = (degrees * 3.14159 / 180) / 0.5  # Convert degrees to radians and calculate time
        
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def pick_up_object(self):
        """Simulate picking up an object"""
        self.get_logger().info('Picking up object...')
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

## Summary

These code examples demonstrate various approaches to implementing LLM-based cognitive planning in robotics:

1. **Basic Integration**: Simple command-to-action mapping using LLMs
2. **Context-Aware Planning**: Using conversation history and sensor data
3. **Local LLMs**: Running open-source models locally for privacy/offline capability
4. **Multi-Modal**: Combining vision and language for complex tasks
5. **Robust Implementation**: Error handling and fallback strategies

Each approach has its trade-offs in terms of complexity, resource usage, privacy, and real-time performance. Choose the approach that best fits your specific robotics application requirements.