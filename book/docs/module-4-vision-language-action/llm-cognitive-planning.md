---
title: LLM Cognitive Planning for Robotics
description: Learn how to use Large Language Models for cognitive planning in robotics applications
sidebar_position: 2
---

# LLM Cognitive Planning for Robotics

## Overview

Large Language Models (LLMs) like GPT, Claude, or open-source alternatives such as Llama can serve as cognitive planners for robotic systems. They excel at understanding natural language commands and translating them into sequences of executable actions. This chapter explores how to integrate LLMs with your robotic system for high-level cognitive planning and task execution.

## The Role of LLMs in Robotic Cognition

LLMs provide several key capabilities that are valuable for robotics:

- **Natural Language Understanding**: Interpret human commands expressed in natural language
- **Reasoning and Planning**: Generate step-by-step plans to achieve complex goals
- **Knowledge Integration**: Leverage vast world knowledge to inform robotic actions
- **Adaptability**: Handle novel situations and commands through reasoning

### Architecture of LLM-Driven Robotic Planning

```
Human Command (Natural Language)
         ↓
    [LLM Cognitive Planner]
         ↓
High-Level Action Plan
         ↓
   [Action Interpreter]
         ↓
Low-Level Robot Commands
         ↓
    Robot Execution
```

## Implementing LLM Integration

### 1. OpenAI GPT Integration

Here's an example of how to integrate OpenAI's GPT model for robotic planning:

```python
import openai
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LLMBotPlanner(Node):
    def __init__(self):
        super().__init__('llm_bot_planner')
        
        # Set your OpenAI API key
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Subscriptions
        self.speech_sub = self.create_subscription(
            String,
            'recognized_speech',
            self.speech_callback,
            10
        )
        
        # Publishers for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Store robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        self.get_logger().info('LLM Bot Planner initialized')

    def speech_callback(self, msg):
        """Process recognized speech and generate robot actions"""
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
        # Prepare the prompt with context
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
        {% raw %}
        {{
          "plan": [
            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
            {{"action": "pick_up_object", "parameters": {{}}}}
          ]
        }}
        {% endraw %}
        
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
            
            # Extract JSON from the response (in case it includes other text)
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
        # Implementation for moving forward
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
        # In a real robot, this would control gripper servos
        self.robot_state['attached_object'] = 'object'

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object...')
        # In a real robot, this would control gripper servos
        self.robot_state['attached_object'] = None

    def detect_object(self, object_type):
        """Simulate object detection"""
        self.get_logger().info(f'Detecting {object_type}...')
        # In a real robot, this would process camera/sensor data

    def check_battery(self):
        """Check robot battery level"""
        self.get_logger().info(f'Battery level: {self.robot_state["battery_level"]}%')
```

### 2. Local LLM Integration with Hugging Face Transformers

For applications requiring privacy or offline capability, you can run open-source models locally:

```python
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
import torch
import json

class LocalLLMPlanner(Node):
    def __init__(self):
        super().__init__('local_llm_planner')
        
        # Load a pre-trained model (e.g., Llama, Mistral, or other)
        model_name = "microsoft/DialoGPT-medium"  # Example model
        
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
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office'
        }
        
        self.get_logger().info('Local LLM Planner initialized')

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
        {% raw %}
        {{
          "plan": [
            {{"action": "move_forward", "parameters": {{"distance": 1.0}}}},
            {{"action": "turn_left", "parameters": {{"degrees": 90}}}},
            {{"action": "pick_up_object", "parameters": {{}}}}
          ]
        }}
        {% endraw %}
        
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
```

## Handling Complex Commands and Context

LLMs excel at understanding complex, multi-step commands. Here's how to handle more sophisticated requests:

```python
def handle_complex_command(self, command):
    """Handle complex commands that require reasoning"""
    # Example: "Go to the kitchen, pick up the red cup, and bring it to John in the living room"
    
    # Break down the complex command into subgoals
    subgoals = self.break_down_command(command)
    
    # Generate a comprehensive plan
    plan = []
    for subgoal in subgoals:
        subplan = self.generate_plan(subgoal)
        plan.extend(subplan)
    
    return plan

def break_down_command(self, command):
    """Break a complex command into simpler subgoals"""
    prompt = f"""
    Break down the following complex command into simpler, sequential subgoals:
    Command: "{command}"
    
    Respond with a JSON array of subgoals.
    Example:
    ["Go to the kitchen", "Find the red cup", "Pick up the red cup", "Go to the living room", "Find John", "Give the cup to John"]
    """
    
    # Call LLM to break down the command
    # Implementation similar to generate_plan method
    pass
```

## Safety and Validation

When using LLMs for robotic control, safety is paramount:

```python
def validate_plan(self, plan):
    """Validate the plan for safety before execution"""
    for step in plan:
        action = step['action']
        params = step.get('parameters', {})
        
        # Check for potentially dangerous actions
        if action == 'move_forward' or action == 'move_backward':
            distance = params.get('distance', 0)
            if distance > 10.0:  # Arbitrary safety limit
                self.get_logger().warn(f'Plan includes unsafe movement: {distance}m')
                return False
        
        # Add other safety checks as needed
        if action == 'pick_up_object' and self.robot_state['attached_object'] is not None:
            self.get_logger().warn('Robot already holding an object, cannot pick up another')
            return False
    
    return True

def execute_plan_safely(self, plan):
    """Execute the plan with safety checks"""
    if not self.validate_plan(plan):
        self.get_logger().error('Plan failed safety validation')
        return False
    
    for step in plan:
        # Add sensor feedback checks during execution
        if not self.check_environment_safety():
            self.get_logger().error('Environment is unsafe, stopping execution')
            self.stop_robot()
            return False
        
        # Execute the action
        self.execute_single_action(step)
    
    return True

def check_environment_safety(self):
    """Check if the environment is safe for movement"""
    # In a real robot, this would check sensor data
    # for obstacles, people, etc.
    return True  # Simplified for example
```

## Performance Considerations

### API Costs
- Monitor API usage to manage costs
- Implement caching for common commands
- Consider using smaller, faster models for simple tasks

### Latency
- LLM calls introduce latency; design your system accordingly
- Consider pre-planning for time-sensitive operations
- Implement timeout mechanisms

### Reliability
- Implement fallback mechanisms if LLM calls fail
- Cache common responses for offline operation
- Provide manual override capabilities

## Summary

LLMs provide powerful cognitive planning capabilities for robotic systems, enabling natural language interaction and complex task decomposition. When integrating LLMs with your robot, consider the trade-offs between cloud and local processing, implement appropriate safety checks, and design your system to handle the latency and variability inherent in LLM responses.

The next section will explore how to combine speech recognition and LLM planning into a complete voice-to-action pipeline.