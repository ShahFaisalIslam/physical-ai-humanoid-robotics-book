---
title: Voice-to-Action Pipeline
description: Learn how to create a complete pipeline from voice commands to robotic actions
sidebar_position: 3
---

# Voice-to-Action Pipeline

## Overview

The voice-to-action pipeline combines speech recognition and cognitive planning to create robots that respond to natural language commands. This chapter details how to integrate OpenAI Whisper for speech recognition with Large Language Models for cognitive planning, creating a complete system for voice-controlled robotics.

## Architecture of the Voice-to-Action System

The complete voice-to-action pipeline consists of several interconnected components:

```
Voice Command
      ↓
[Audio Capture]
      ↓
[Preprocessing & Enhancement]
      ↓
[Whisper ASR - Speech to Text]
      ↓
[Natural Language Understanding]
      ↓
[LLM Cognitive Planning]
      ↓
[Action Validation & Safety Checks]
      ↓
[Low-Level Robot Control]
      ↓
Robot Action Execution
```

Each component plays a crucial role in transforming human speech into robot behavior.

## Implementing the Complete Pipeline

### 1. System Integration Node

Here's a complete ROS 2 node that integrates all components:

```python
import rclpy
from rclpy.node import Node
import numpy as np
import pyaudio
import whisper
import threading
import queue
import time
import json
import openai
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import librosa

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')
        
        # Initialize Whisper model for speech recognition
        self.whisper_model = whisper.load_model("base")
        
        # Set OpenAI API key for cognitive planning
        openai.api_key = "YOUR_API_KEY_HERE"
        
        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Publishers
        self.interim_text_pub = self.create_publisher(String, 'interim_speech_text', 10)
        self.final_text_pub = self.create_publisher(String, 'final_speech_text', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            'final_speech_text',
            self.voice_command_callback,
            10
        )
        
        # Robot state
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'battery_level': 100,
            'attached_object': None,
            'environment': 'indoor_office',
            'is_moving': False
        }
        
        # Audio buffer and control
        self.audio_queue = queue.Queue()
        self.is_listening = True
        self.wake_word_detected = False
        self.wake_word = "robot"
        
        # Start audio recording thread
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        self.get_logger().info('Voice-to-Action Node initialized')

    def record_audio(self):
        """Continuously record audio and add to queue"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        while self.is_listening:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)
        
        stream.stop_stream()
        stream.close()

    def process_audio_queue(self):
        """Process audio chunks from the queue"""
        audio_buffer = []
        buffer_duration = 3  # Process every 3 seconds of audio
        max_chunks = int(buffer_duration * self.rate / self.chunk)
        
        while self.is_listening:
            try:
                # Get audio chunk from queue (with timeout)
                audio_chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.append(audio_chunk)
                
                # If buffer is full, process the audio
                if len(audio_buffer) >= max_chunks:
                    # Convert buffer to numpy array
                    audio_data = b''.join(audio_buffer)
                    audio_np = np.frombuffer(audio_data, dtype=np.int16)
                    
                    # Normalize to [-1, 1]
                    audio_float = audio_np.astype(np.float32) / 32768.0
                    
                    # Check for wake word first
                    if not self.wake_word_detected:
                        self.check_wake_word(audio_float)
                    
                    # Process with Whisper if wake word was detected
                    elif self.wake_word_detected:
                        result = self.whisper_model.transcribe(audio_float)
                        recognized_text = result['text'].strip()
                        
                        if recognized_text:
                            # Publish interim text
                            interim_msg = String()
                            interim_msg.data = recognized_text
                            self.interim_text_pub.publish(interim_msg)
                            
                            # Check if this is a complete command (simple heuristic)
                            if self.is_complete_command(recognized_text):
                                # Publish final text for processing
                                final_msg = String()
                                final_msg.data = recognized_text
                                self.final_text_pub.publish(final_msg)
                                
                                # Reset wake word detection
                                self.wake_word_detected = False
                                self.get_logger().info(f'Command recognized: "{recognized_text}"')
                        
                        # Clear buffer
                        audio_buffer = []
                        
            except queue.Empty:
                continue

    def check_wake_word(self, audio_float):
        """Check if the wake word is present in the audio"""
        # For simplicity, we'll assume wake word detection is triggered externally
        # In a real implementation, you'd use a dedicated wake word detection model
        # or implement keyword spotting
        
        # For this example, we'll simulate wake word detection
        # by publishing a message to a topic when wake word is "detected"
        self.get_logger().info(f'Listening for wake word: {self.wake_word}')
        # In a real implementation, this would analyze audio_float for the wake word

    def is_complete_command(self, text):
        """Simple heuristic to determine if text is a complete command"""
        # In a real implementation, this would be more sophisticated
        # For now, we'll consider it complete if it's not just a wake word
        return len(text.strip()) > len(self.wake_word) and text.lower() != self.wake_word.lower()

    def voice_command_callback(self, msg):
        """Process recognized voice command and generate robot action"""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')
        
        # Generate plan using LLM
        plan = self.generate_plan(command)
        
        if plan:
            # Execute the plan safely
            self.execute_plan_safely(plan)
        else:
            self.get_logger().warn('Could not generate a plan for the command')
            # Provide feedback to user
            feedback_msg = String()
            feedback_msg.data = f"Sorry, I couldn't understand the command: {command}"
            self.interim_text_pub.publish(feedback_msg)

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
        if not self.validate_plan(plan):
            self.get_logger().error('Plan failed safety validation')
            return False
        
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

    def validate_plan(self, plan):
        """Validate the plan for safety before execution"""
        for step in plan:
            action = step['action']
            params = step.get('parameters', {})
            
            # Check for potentially dangerous actions
            if action in ['move_forward', 'move_backward']:
                distance = params.get('distance', 0)
                if distance > 10.0:  # Arbitrary safety limit
                    self.get_logger().warn(f'Plan includes unsafe movement: {distance}m')
                    return False
            
            # Check if trying to pick up when already holding an object
            if action == 'pick_up_object' and self.robot_state['attached_object'] is not None:
                self.get_logger().warn('Robot already holding an object, cannot pick up another')
                return False
        
        return True

    def check_environment_safety(self):
        """Check if the environment is safe for movement"""
        # In a real robot, this would check sensor data
        # for obstacles, people, etc.
        # For this example, we'll return True
        return True

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

    def destroy_node(self):
        """Clean up resources"""
        self.is_listening = False
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()
    
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

## Advanced Voice Command Processing

### Contextual Understanding

To improve the system's ability to handle context-dependent commands, implement a conversation memory:

```python
class ConversationMemory:
    def __init__(self, max_history=10):
        self.history = []
        self.max_history = max_history
    
    def add_interaction(self, user_input, robot_response):
        """Add a user-robot interaction to the history"""
        self.history.append({
            'user': user_input,
            'robot': robot_response,
            'timestamp': time.time()
        })
        
        # Keep only the most recent interactions
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history:]
    
    def get_context(self):
        """Get recent conversation context"""
        return self.history[-5:]  # Return last 5 interactions
    
    def clear_memory(self):
        """Clear the conversation history"""
        self.history = []

class VoiceToActionNodeWithMemory(VoiceToActionNode):
    def __init__(self):
        super().__init__()
        self.conversation_memory = ConversationMemory()
    
    def generate_plan(self, command):
        """Generate a plan considering conversation context"""
        context = self.conversation_memory.get_context()
        
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
        
        Consider the conversation context when interpreting the command.
        Be specific with parameters and ensure the plan is executable.
        """
        
        # Implementation similar to parent class
        # ... (rest of implementation)
```

### Intent Recognition and Entity Extraction

For more robust command processing, implement intent recognition:

```python
def extract_intent_and_entities(self, command):
    """Extract the intent and relevant entities from a command"""
    prompt = f"""
    Analyze the following command and extract the intent and entities:
    
    Command: "{command}"
    
    Return a JSON object with:
    - intent: The main action requested
    - entities: Relevant parameters (locations, objects, people, etc.)
    
    Example:
    {{
      "intent": "navigate_and_manipulate",
      "entities": {{
        "action": "pick_up",
        "object": "red cup",
        "destination": "kitchen",
        "recipient": "John"
      }}
    }}
    """
    
    # Call LLM to extract intent and entities
    # Implementation similar to other LLM calls
    pass
```

## Error Handling and Recovery

Implement robust error handling for the voice-to-action pipeline:

```python
class VoiceToActionNodeWithRecovery(VoiceToActionNode):
    def __init__(self):
        super().__init__()
        self.retry_count = 0
        self.max_retries = 3
        self.command_history = []
    
    def voice_command_callback(self, msg):
        """Process voice command with error handling and recovery"""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')
        
        # Add command to history
        self.command_history.append({
            'command': command,
            'timestamp': time.time()
        })
        
        # Limit history size
        if len(self.command_history) > 20:
            self.command_history = self.command_history[-20:]
        
        # Generate and execute plan with error handling
        try:
            plan = self.generate_plan(command)
            
            if plan:
                success = self.execute_plan_safely(plan)
                if not success:
                    self.handle_execution_failure(command, plan)
            else:
                self.handle_planning_failure(command)
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            self.handle_system_error(command, str(e))
    
    def handle_execution_failure(self, command, plan):
        """Handle when plan execution fails"""
        self.get_logger().warn(f'Plan execution failed for command: {command}')
        
        # Try to recover by simplifying the plan
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f'Attempting recovery, retry {self.retry_count}/{self.max_retries}')
            
            # Simplify the plan or try an alternative approach
            simplified_plan = self.simplify_plan(plan)
            if simplified_plan:
                self.execute_plan_safely(simplified_plan)
        else:
            self.get_logger().error('Max retries reached, giving up on command')
            self.retry_count = 0  # Reset for next command
    
    def handle_planning_failure(self, command):
        """Handle when LLM fails to generate a plan"""
        self.get_logger().warn(f'Could not generate plan for command: {command}')
        
        # Provide feedback to user
        feedback_msg = String()
        feedback_msg.data = f"Sorry, I couldn't understand how to execute: {command}. Could you rephrase?"
        self.interim_text_pub.publish(feedback_msg)
    
    def handle_system_error(self, command, error_msg):
        """Handle general system errors"""
        self.get_logger().error(f'System error for command "{command}": {error_msg}')
        
        # Provide feedback to user
        feedback_msg = String()
        feedback_msg.data = "Sorry, I encountered an error processing your command. Please try again."
        self.interim_text_pub.publish(feedback_msg)
        
        # Reset for next command
        self.retry_count = 0
```

## Performance Optimization

### Caching Common Commands

To reduce API calls and improve response time, cache common command interpretations:

```python
import hashlib

class CachedVoiceToActionNode(VoiceToActionNode):
    def __init__(self):
        super().__init__()
        self.command_cache = {}
        self.max_cache_size = 100
    
    def generate_plan(self, command):
        """Generate a plan with caching for common commands"""
        # Create a hash of the command for caching
        command_hash = hashlib.md5(command.encode()).hexdigest()
        
        # Check if we have a cached plan for this command
        if command_hash in self.command_cache:
            self.get_logger().info('Using cached plan for command')
            return self.command_cache[command_hash]
        
        # Generate new plan
        plan = super().generate_plan(command)
        
        # Cache the plan if successful
        if plan:
            # Limit cache size
            if len(self.command_cache) >= self.max_cache_size:
                # Remove oldest entry (in a real implementation, you might use LRU)
                oldest_key = next(iter(self.command_cache))
                del self.command_cache[oldest_key]
            
            self.command_cache[command_hash] = plan
        
        return plan
```

### Asynchronous Processing

To improve responsiveness, process audio and generate plans asynchronously:

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class AsyncVoiceToActionNode(VoiceToActionNode):
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.plan_queue = asyncio.Queue()
        
    async def process_voice_command_async(self, command):
        """Process voice command asynchronously"""
        # Generate plan in a separate thread
        loop = asyncio.get_event_loop()
        plan = await loop.run_in_executor(self.executor, self.generate_plan, command)
        
        if plan:
            # Execute plan in a separate thread
            success = await loop.run_in_executor(self.executor, self.execute_plan_safely, plan)
            return success
        else:
            return False
```

## Testing and Validation

Create a testing framework to validate the voice-to-action pipeline:

```python
import unittest

class TestVoiceToActionPipeline(unittest.TestCase):
    def setUp(self):
        """Set up test environment"""
        # In a real test, you might mock the LLM and audio components
        pass
    
    def test_simple_command(self):
        """Test a simple navigation command"""
        command = "move forward 2 meters"
        expected_actions = [
            {"action": "move_forward", "parameters": {"distance": 2.0}}
        ]
        
        # In a real test, you would validate the generated plan
        # against expected actions
        # self.assertEqual(generated_plan, expected_actions)
    
    def test_complex_command(self):
        """Test a complex multi-step command"""
        command = "go to the kitchen and pick up the red cup"
        # Validate that this generates appropriate navigation and manipulation steps
    
    def test_error_handling(self):
        """Test error handling for invalid commands"""
        command = "invalid command that should fail"
        # Validate that the system handles this gracefully
```

## Summary

The voice-to-action pipeline creates an intuitive interface for human-robot interaction by combining speech recognition with cognitive planning. When implementing such a system, consider:

1. **Architecture**: Design a robust pipeline with clear separation of concerns
2. **Safety**: Implement comprehensive validation and safety checks
3. **Performance**: Optimize for latency, especially in real-time applications
4. **Reliability**: Include error handling and recovery mechanisms
5. **Context**: Consider conversation history for better understanding
6. **Privacy**: Choose between cloud and local processing based on requirements

The next section will explore practical examples of implementing these concepts with real-world robotics applications.