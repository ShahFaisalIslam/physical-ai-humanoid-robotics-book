---
title: OpenAI Whisper Integration Code Examples
description: Practical code examples for integrating OpenAI Whisper in robotics applications
sidebar_position: 4
---

# OpenAI Whisper Integration Code Examples

## Overview

This section provides practical code examples for integrating OpenAI Whisper into robotics applications. These examples demonstrate how to capture audio, process it through Whisper, and use the results for robotic control.

## Basic Audio Capture and Processing

### Simple Audio Capture Node

```python
import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
import whisper
import threading
from std_msgs.msg import String

class SimpleWhisperNode(Node):
    def __init__(self):
        super().__init__('simple_whisper_node')
        
        # Publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'recognized_speech', 10)
        
        # Load Whisper model
        self.model = whisper.load_model("base")
        
        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Start audio recording
        self.start_recording()
        
        self.get_logger().info('Simple Whisper Node initialized')

    def start_recording(self):
        """Start audio recording in a separate thread"""
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def record_audio(self):
        """Record audio and process with Whisper"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # Buffer to hold audio chunks
        audio_buffer = []
        buffer_duration = 3  # Process every 3 seconds
        max_chunks = int(buffer_duration * self.rate / self.chunk)
        
        while rclpy.ok():
            # Read audio chunk
            data = stream.read(self.chunk)
            audio_buffer.append(data)
            
            # If buffer is full, process the audio
            if len(audio_buffer) >= max_chunks:
                # Convert buffer to numpy array
                audio_data = b''.join(audio_buffer)
                audio_np = np.frombuffer(audio_data, dtype=np.int16)
                
                # Normalize to [-1, 1]
                audio_float = audio_np.astype(np.float32) / 32768.0
                
                # Process with Whisper
                result = self.model.transcribe(audio_float)
                recognized_text = result['text'].strip()
                
                if recognized_text:  # Only publish if there's text
                    msg = String()
                    msg.data = recognized_text
                    self.text_publisher.publish(msg)
                    self.get_logger().info(f'Recognized: "{recognized_text}"')
                
                # Clear buffer
                audio_buffer = []

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()
```

## Advanced Audio Processing

### Audio Preprocessing for Better Recognition

```python
import librosa
import soundfile as sf
import io
import numpy as np

class AdvancedWhisperNode(SimpleWhisperNode):
    def __init__(self):
        super().__init__()
        
        # Add noise reduction parameters
        self.noise_floor = 0.01  # Minimum signal level to consider as speech
        self.silence_threshold = 0.02  # Threshold for silence detection
        
        self.get_logger().info('Advanced Whisper Node initialized')

    def preprocess_audio(self, raw_audio):
        """
        Preprocess raw audio for optimal Whisper performance
        """
        # Resample if needed
        if len(raw_audio) > 0 and isinstance(raw_audio, np.ndarray):
            # Normalize audio
            raw_audio = raw_audio / max(np.max(np.abs(raw_audio)), 1e-9)  # Avoid division by zero
            
            # Apply pre-emphasis filter to boost high frequencies
            pre_emphasis = 0.97
            audio_pre = np.append(raw_audio[0], raw_audio[1:] - pre_emphasis * raw_audio[:-1])
            
            # Apply windowing to reduce spectral leakage
            window = np.hanning(len(audio_pre))
            audio_windowed = audio_pre * window
            
            return audio_windowed
        else:
            return raw_audio

    def is_silence(self, audio_chunk, threshold=None):
        """Check if an audio chunk contains silence"""
        if threshold is None:
            threshold = self.silence_threshold
            
        # Calculate RMS energy of the audio chunk
        rms = np.sqrt(np.mean(np.square(audio_chunk)))
        return rms < threshold

    def record_audio(self):
        """Record audio with silence detection and preprocessing"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # Buffer to hold audio chunks
        audio_buffer = []
        buffer_duration = 3  # Process every 3 seconds
        max_chunks = int(buffer_duration * self.rate / self.chunk)
        
        while rclpy.ok():
            # Read audio chunk
            data = stream.read(self.chunk)
            audio_np = np.frombuffer(data, dtype=np.int16)
            audio_float = audio_np.astype(np.float32) / 32768.0
            
            # Check if this chunk is silence
            if not self.is_silence(audio_float):
                audio_buffer.append(data)
            
            # If buffer is full, process the audio
            if len(audio_buffer) >= max_chunks and len(audio_buffer) > 0:
                # Convert buffer to numpy array
                audio_data = b''.join(audio_buffer)
                audio_np = np.frombuffer(audio_data, dtype=np.int16)
                
                # Preprocess the audio
                audio_processed = self.preprocess_audio(audio_np.astype(np.float32) / 32768.0)
                
                # Process with Whisper
                result = self.model.transcribe(audio_processed)
                recognized_text = result['text'].strip()
                
                if recognized_text:  # Only publish if there's text
                    msg = String()
                    msg.data = recognized_text
                    self.text_publisher.publish(msg)
                    self.get_logger().info(f'Recognized: "{recognized_text}"')
                
                # Clear buffer
                audio_buffer = []

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()
```

## Streaming Audio Processing

### Continuous Audio Processing with VAD (Voice Activity Detection)

```python
import collections
import webrtcvad

class StreamingWhisperNode(AdvancedWhisperNode):
    def __init__(self):
        super().__init__()
        
        # Initialize WebRTC VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad()
        # Set VAD aggressiveness (0-3, 3 is most aggressive)
        self.vad.set_mode(1)
        
        # Audio parameters for VAD (must be 8000, 16000, 32000, or 48000 Hz)
        self.vad_sample_rate = 16000
        # Frame size for VAD (10, 20, or 30 ms)
        self.frame_duration = 20  # ms
        self.frame_size = int(self.vad_sample_rate * self.frame_duration / 1000) * 2  # 2 bytes per sample
        
        # Ring buffer for audio chunks
        self.ring_buffer = collections.deque(maxlen=int(30 * self.vad_sample_rate / self.frame_size))  # 30 frames = 600ms
        self.triggered = False
        self.vad_frames = []
        self.ring_buffer_count = 0
        self.speech_start = 0
        self.speech_end = 0
        self.silence_duration = 0
        
        self.get_logger().info('Streaming Whisper Node with VAD initialized')

    def is_speech(self, frame):
        """Check if a frame contains speech using WebRTC VAD"""
        try:
            return self.vad.is_speech(frame, self.vad_sample_rate)
        except:
            return False

    def record_audio(self):
        """Record audio with VAD and streaming processing"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.frame_size//2  # Divide by 2 because our frame size is in bytes but chunk is in samples
        )
        
        while rclpy.ok():
            # Read audio chunk (10ms worth)
            data = stream.read(self.frame_size//2, exception_on_overflow=False)
            
            # Add to ring buffer
            self.ring_buffer.append(data)
            
            # Check if this frame contains speech
            is_speech = self.is_speech(data)
            
            if not self.triggered:
                # No speech detected yet
                self.ring_buffer_count += 1
                if is_speech:
                    # Start of speech detected
                    self.triggered = True
                    self.speech_start = self.ring_buffer_count - self.ring_buffer.maxlen
                    self.vad_frames = self.ring_buffer.copy()
            else:
                # Speech already detected
                if is_speech:
                    # Continue collecting speech frames
                    self.vad_frames.append(data)
                    self.silence_duration = 0
                else:
                    # Silence detected
                    self.silence_duration += 1
                    self.vad_frames.append(data)
                    
                    # If silence continues for 20 frames (200ms), consider speech ended
                    if self.silence_duration > 20:
                        # Process collected speech
                        self.process_speech_segment()
                        
                        # Reset for next speech segment
                        self.triggered = False
                        self.silence_duration = 0
                        self.vad_frames = []

    def process_speech_segment(self):
        """Process a complete speech segment"""
        if len(self.vad_frames) < 10:  # Less than 100ms of speech, probably noise
            return
            
        # Convert collected frames to numpy array
        audio_data = b''.join(self.vad_frames)
        audio_np = np.frombuffer(audio_data, dtype=np.int16)
        
        # Preprocess the audio
        audio_processed = self.preprocess_audio(audio_np.astype(np.float32) / 32768.0)
        
        # Process with Whisper
        result = self.model.transcribe(audio_processed)
        recognized_text = result['text'].strip()
        
        if recognized_text:  # Only publish if there's text
            msg = String()
            msg.data = recognized_text
            self.text_publisher.publish(msg)
            self.get_logger().info(f'Recognized: "{recognized_text}"')

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()
```

## Whisper with ROS 2 Actions

### Using ROS 2 Actions for Long-Running Transcription

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

# Define a custom action interface (you would create this in your package)
# For this example, we'll define a simple structure
class TranscriptionAction:
    class Goal:
        def __init__(self):
            self.duration = 0  # Duration in seconds to record
    
    class Result:
        def __init__(self):
            self.transcript = ""
    
    class Feedback:
        def __init__(self):
            self.interim_transcript = ""

class ActionWhisperNode(Node):
    def __init__(self):
        super().__init__('action_whisper_node')
        
        # Publisher for real-time updates
        self.interim_publisher = self.create_publisher(String, 'interim_transcript', 10)
        
        # Initialize Whisper model
        self.model = whisper.load_model("base")
        
        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            TranscriptionAction,
            'transcribe_audio',
            self.execute_callback
        )
        
        self.get_logger().info('Action Whisper Node initialized')

    def execute_callback(self, goal_handle):
        """Execute the transcription action"""
        self.get_logger().info(f'Executing transcription for {goal_handle.request.duration} seconds')
        
        # Record audio for the specified duration
        audio_data = self.record_for_duration(goal_handle.request.duration)
        
        # Process with Whisper
        result = self.model.transcribe(audio_data)
        transcript = result['text'].strip()
        
        # Create result message
        result_msg = TranscriptionAction.Result()
        result_msg.transcript = transcript
        
        # Publish final result
        final_msg = String()
        final_msg.data = transcript
        self.interim_publisher.publish(final_msg)
        
        self.get_logger().info(f'Transcription completed: "{transcript}"')
        
        goal_handle.succeed()
        return result_msg

    def record_for_duration(self, duration_sec):
        """Record audio for a specific duration"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        frames = []
        total_chunks = int(duration_sec * self.rate / self.chunk)
        
        for _ in range(total_chunks):
            data = stream.read(self.chunk)
            frames.append(data)
        
        # Convert to numpy array
        audio_data = b''.join(frames)
        audio_np = np.frombuffer(audio_data, dtype=np.int16)
        audio_float = audio_np.astype(np.float32) / 32768.0
        
        stream.stop_stream()
        stream.close()
        
        return audio_float

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()
```

## Error Handling and Robustness

### Whisper Node with Comprehensive Error Handling

```python
import time
from collections import deque

class RobustWhisperNode(Node):
    def __init__(self):
        super().__init__('robust_whisper_node')
        
        # Publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'recognized_speech', 10)
        
        # Parameters
        self.declare_parameter('whisper_model_size', 'base')
        self.declare_parameter('audio_rate', 16000)
        self.declare_parameter('audio_chunk', 1024)
        
        model_size = self.get_parameter('whisper_model_size').get_parameter_value().string_value
        self.rate = self.get_parameter('audio_rate').get_parameter_value().integer_value
        self.chunk = self.get_parameter('audio_chunk').get_parameter_value().integer_value
        
        # Initialize Whisper model with error handling
        try:
            self.model = whisper.load_model(model_size)
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None
            return
        
        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        
        # Initialize PyAudio
        try:
            self.audio = pyaudio.PyAudio()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PyAudio: {e}')
            return
        
        # Audio buffer with timeout
        self.audio_queue = queue.Queue()
        self.is_recording = True
        
        # Statistics for monitoring
        self.processed_chunks = 0
        self.failed_transcriptions = 0
        self.start_time = time.time()
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Timer for periodic statistics
        self.stats_timer = self.create_timer(30.0, self.log_statistics)
        
        self.get_logger().info('Robust Whisper Node initialized')

    def record_audio(self):
        """Record audio with error handling"""
        try:
            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            return
        
        while self.is_recording:
            try:
                data = stream.read(self.chunk, exception_on_overflow=False)
                self.audio_queue.put(data)
            except Exception as e:
                self.get_logger().warn(f'Error reading audio: {e}')
                # Continue despite error
        
        stream.stop_stream()
        stream.close()

    def process_audio(self):
        """Process audio from queue with error handling"""
        audio_buffer = []
        buffer_duration = 3  # Process every 3 seconds
        max_chunks = int(buffer_duration * self.rate / self.chunk)
        
        while self.is_recording:
            try:
                # Get audio chunk from queue with timeout
                data = self.audio_queue.get(timeout=1.0)
                audio_buffer.append(data)
                
                # If buffer is full, process the audio
                if len(audio_buffer) >= max_chunks:
                    self.process_audio_buffer(audio_buffer)
                    audio_buffer = []  # Clear buffer
                    
            except queue.Empty:
                continue  # Continue if no audio in queue
            except Exception as e:
                self.get_logger().error(f'Error processing audio: {e}')
                self.failed_transcriptions += 1
                audio_buffer = []  # Clear buffer on error

    def process_audio_buffer(self, audio_buffer):
        """Process a buffer of audio chunks"""
        try:
            # Convert buffer to numpy array
            audio_data = b''.join(audio_buffer)
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            # Normalize to [-1, 1]
            audio_float = audio_np.astype(np.float32) / 32768.0
            
            # Process with Whisper
            result = self.model.transcribe(audio_float)
            recognized_text = result['text'].strip()
            
            if recognized_text:  # Only publish if there's text
                msg = String()
                msg.data = recognized_text
                self.text_publisher.publish(msg)
                self.get_logger().info(f'Recognized: "{recognized_text}"')
            
            self.processed_chunks += 1
            
        except Exception as e:
            self.get_logger().error(f'Error in Whisper transcription: {e}')
            self.failed_transcriptions += 1

    def log_statistics(self):
        """Log processing statistics"""
        elapsed_time = time.time() - self.start_time
        success_rate = 0
        if self.processed_chunks > 0:
            success_rate = 100 * (self.processed_chunks - self.failed_transcriptions) / self.processed_chunks
            
        self.get_logger().info(
            f'Audio processing stats: {self.processed_chunks} processed, '
            f'{self.failed_transcriptions} failed, '
            f'{success_rate:.1f}% success rate over {elapsed_time/60:.1f} minutes'
        )

    def destroy_node(self):
        """Clean up resources"""
        self.is_recording = False
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()
```

## Summary

These code examples demonstrate various approaches to integrating OpenAI Whisper in robotics applications:

1. **Basic Integration**: Simple audio capture and transcription
2. **Advanced Processing**: Audio preprocessing for better recognition
3. **Streaming**: Continuous processing with voice activity detection
4. **ROS 2 Actions**: Using actions for long-running transcription tasks
5. **Robust Implementation**: Error handling and monitoring

Each approach has its trade-offs in terms of complexity, resource usage, and real-time performance. Choose the approach that best fits your specific robotics application requirements.