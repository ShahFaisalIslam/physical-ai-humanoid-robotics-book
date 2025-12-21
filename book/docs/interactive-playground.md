---
title: Interactive Code Playground
description: Interactive components for experimenting with robotics code examples
sidebar_position: 1
---

# Interactive Code Playground

## Overview

This section provides interactive code playground components that allow readers to experiment with robotics code examples directly in their browser. These components simulate the behavior of real robotic systems without requiring actual hardware or ROS 2 installation.

## Interactive ROS 2 Publisher/Subscriber Example

```jsx
import React, { useState, useEffect } from 'react';

const ROS2Playground = () => {
  const [messages, setMessages] = useState([]);
  const [newMessage, setNewMessage] = useState('');
  const [topic, setTopic] = useState('/chatter');
  const [publisherStatus, setPublisherStatus] = useState('idle');
  const [subscriberStatus, setSubscriberStatus] = useState('listening');

  const publishMessage = () => {
    if (!newMessage.trim()) return;

    const message = {
      id: Date.now(),
      topic: topic,
      data: newMessage,
      timestamp: new Date().toISOString(),
      type: 'String'
    };

    setMessages(prev => [...prev, message]);
    setNewMessage('');
    setPublisherStatus('published');
    
    setTimeout(() => setPublisherStatus('idle'), 1000);
  };

  const clearMessages = () => {
    setMessages([]);
  };

  return (
    <div className="playground-container">
      <h3>ROS 2 Publisher/Subscriber Playground</h3>
      
      <div className="controls">
        <div className="input-group">
          <label htmlFor="topic">Topic:</label>
          <input
            id="topic"
            type="text"
            value={topic}
            onChange={(e) => setTopic(e.target.value)}
          />
        </div>
        
        <div className="input-group">
          <label htmlFor="message">Message:</label>
          <input
            id="message"
            type="text"
            value={newMessage}
            onChange={(e) => setNewMessage(e.target.value)}
            placeholder="Enter message to publish..."
          />
          <button onClick={publishMessage} disabled={publisherStatus === 'published'}>
            {publisherStatus === 'published' ? 'Published!' : 'Publish'}
          </button>
        </div>
        
        <button onClick={clearMessages} className="clear-btn">Clear Messages</button>
      </div>

      <div className="status-indicators">
        <div className={`status-indicator ${publisherStatus}`}>
          Publisher: {publisherStatus}
        </div>
        <div className={`status-indicator ${subscriberStatus}`}>
          Subscriber: {subscriberStatus}
        </div>
      </div>

      <div className="message-log">
        <h4>Message Log</h4>
        {messages.length === 0 ? (
          <p className="no-messages">No messages yet. Publish a message to see it here.</p>
        ) : (
          <ul>
            {messages.map((msg) => (
              <li key={msg.id} className="message-item">
                <div className="message-header">
                  <span className="topic">{msg.topic}</span>
                  <span className="timestamp">{msg.timestamp}</span>
                </div>
                <div className="message-content">
                  {msg.data}
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>

      <div className="explanation">
        <h4>How It Works</h4>
        <p>
          This interactive playground simulates the basic functionality of ROS 2 publishers and subscribers:
        </p>
        <ul>
          <li><strong>Publisher</strong>: Sends messages to a specified topic</li>
          <li><strong>Subscriber</strong>: Receives messages from topics it's listening to</li>
          <li><strong>Topic</strong>: Named channel for message exchange</li>
          <li><strong>Messages</strong>: Data sent between nodes in a standardized format</li>
        </ul>
        <p>
          In a real ROS 2 system, publishers and subscribers would be in separate nodes running on the same or different machines.
          The ROS 2 middleware (DDS) handles the communication between them based on topic names.
        </p>
      </div>
    </div>
  );
};

export default ROS2Playground;
```

## Interactive Robot Navigation Simulator

```jsx
import React, { useState, useEffect, useRef } from 'react';

const NavigationSimulator = () => {
  const canvasRef = useRef(null);
  const [robotPos, setRobotPos] = useState({ x: 50, y: 50 });
  const [goalPos, setGoalPos] = useState({ x: 400, y: 300 });
  const [obstacles, setObstacles] = useState([
    { x: 150, y: 150, width: 50, height: 50 },
    { x: 250, y: 200, width: 30, height: 80 },
    { x: 350, y: 100, width: 70, height: 30 }
  ]);
  const [isNavigating, setIsNavigating] = useState(false);
  const [path, setPath] = useState([]);

  // Draw the environment
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw obstacles
    ctx.fillStyle = '#8B4513';
    obstacles.forEach(obstacle => {
      ctx.fillRect(obstacle.x, obstacle.y, obstacle.width, obstacle.height);
    });
    
    // Draw goal
    ctx.fillStyle = '#00FF00';
    ctx.beginPath();
    ctx.arc(goalPos.x, goalPos.y, 10, 0, Math.PI * 2);
    ctx.fill();
    
    // Draw path if exists
    if (path.length > 1) {
      ctx.strokeStyle = '#0000FF';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(path[0].x, path[0].y);
      for (let i = 1; i < path.length; i++) {
        ctx.lineTo(path[i].x, path[i].y);
      }
      ctx.stroke();
    }
    
    // Draw robot
    ctx.fillStyle = '#FF0000';
    ctx.beginPath();
    ctx.arc(robotPos.x, robotPos.y, 8, 0, Math.PI * 2);
    ctx.fill();
    
    // Draw robot direction indicator
    ctx.strokeStyle = '#FFFFFF';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(robotPos.x, robotPos.y);
    ctx.lineTo(robotPos.x + 15, robotPos.y);
    ctx.stroke();
  }, [robotPos, goalPos, obstacles, path]);

  // Simple path planning (straight line with obstacle avoidance)
  const planPath = () => {
    // In a real system, this would use A* or D* algorithm
    // For this simulation, we'll create a simple path
    const newPath = [];
    let currentX = robotPos.x;
    let currentY = robotPos.y;
    
    // Move in steps toward the goal
    const steps = 50;
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const x = robotPos.x + (goalPos.x - robotPos.x) * t;
      const y = robotPos.y + (goalPos.y - robotPos.y) * t;
      newPath.push({ x, y });
    }
    
    // Add simple obstacle avoidance
    const adjustedPath = newPath.map(point => {
      // Check if point is too close to any obstacle
      for (const obs of obstacles) {
        const distX = Math.abs(point.x - (obs.x + obs.width/2));
        const distY = Math.abs(point.y - (obs.y + obs.height/2));
        const halfWidth = obs.width/2 + 15;
        const halfHeight = obs.height/2 + 15;
        
        if (distX <= halfWidth && distY <= halfHeight) {
          // Move point away from obstacle
          return {
            x: point.x + (point.x < obs.x ? -20 : 20),
            y: point.y + (point.y < obs.y ? -20 : 20)
          };
        }
      }
      return point;
    });
    
    setPath(adjustedPath);
    return adjustedPath;
  };

  const startNavigation = () => {
    if (isNavigating) return;
    
    const plannedPath = planPath();
    setIsNavigating(true);
    
    // Animate the robot along the path
    let step = 0;
    const interval = setInterval(() => {
      if (step < plannedPath.length) {
        setRobotPos(plannedPath[step]);
        step++;
      } else {
        clearInterval(interval);
        setIsNavigating(false);
      }
    }, 100); // Move every 100ms
  };

  const resetSimulation = () => {
    setRobotPos({ x: 50, y: 50 });
    setPath([]);
    setIsNavigating(false);
  };

  const handleCanvasClick = (e) => {
    if (isNavigating) return;
    
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    // Check if click is on an obstacle
    let onObstacle = false;
    for (const obs of obstacles) {
      if (x >= obs.x && x <= obs.x + obs.width && 
          y >= obs.y && y <= obs.y + obs.height) {
        onObstacle = true;
        break;
      }
    }
    
    if (!onObstacle) {
      setGoalPos({ x, y });
    }
  };

  return (
    <div className="navigation-simulator">
      <h3>Robot Navigation Simulator</h3>
      
      <div className="simulator-controls">
        <button onClick={startNavigation} disabled={isNavigating}>
          {isNavigating ? 'Navigating...' : 'Start Navigation'}
        </button>
        <button onClick={resetSimulation}>Reset</button>
        <p>Click on the map to set a new goal (avoid obstacles)</p>
      </div>
      
      <div className="canvas-container">
        <canvas
          ref={canvasRef}
          width={500}
          height={400}
          onClick={handleCanvasClick}
          style={{ border: '1px solid #ccc', background: '#f0f0f0' }}
        />
      </div>
      
      <div className="simulator-info">
        <h4>Navigation Concepts Demonstrated</h4>
        <ul>
          <li><strong>Path Planning</strong>: Finding a route from start to goal</li>
          <li><strong>Obstacle Avoidance</strong>: Navigating around obstacles</li>
          <li><strong>Localization</strong>: Knowing the robot's position</li>
          <li><strong>Motion Control</strong>: Moving the robot along the path</li>
        </ul>
        
        <p>
          In a real robotic system, navigation involves more sophisticated algorithms like A*, D*, or RRT for path planning,
          and Dynamic Window Approach (DWA) or Trajectory Rollout for local obstacle avoidance.
          This simulation demonstrates the basic concepts without the complexity of real-world implementation.
        </p>
      </div>
    </div>
  );
};

export default NavigationSimulator;
```

## Interactive Perception Playground

```jsx
import React, { useState, useRef, useEffect } from 'react';

const PerceptionPlayground = () => {
  const canvasRef = useRef(null);
  const [detectedObjects, setDetectedObjects] = useState([]);
  const [selectedTool, setSelectedTool] = useState('add');
  const [detectionMode, setDetectionMode] = useState('color');
  const [isDetecting, setIsDetecting] = useState(false);

  // Draw the scene
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw background
    ctx.fillStyle = '#e0e0e0';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Draw objects
    detectedObjects.forEach(obj => {
      ctx.fillStyle = obj.color;
      ctx.beginPath();
      if (obj.shape === 'rectangle') {
        ctx.rect(obj.x, obj.y, obj.width, obj.height);
      } else if (obj.shape === 'circle') {
        ctx.arc(obj.x, obj.y, obj.radius, 0, Math.PI * 2);
      } else if (obj.shape === 'triangle') {
        ctx.moveTo(obj.x, obj.y);
        ctx.lineTo(obj.x + obj.width, obj.y);
        ctx.lineTo(obj.x + obj.width/2, obj.y - obj.height);
        ctx.closePath();
      }
      ctx.fill();
      
      // Draw object label
      ctx.fillStyle = '#000';
      ctx.font = '12px Arial';
      ctx.fillText(obj.label, obj.x, obj.y - 10);
    });
  }, [detectedObjects]);

  const handleCanvasClick = (e) => {
    if (selectedTool !== 'add' || isDetecting) return;
    
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    // Add a new object
    const newObject = {
      id: Date.now(),
      x,
      y,
      width: 40 + Math.random() * 40,
      height: 40 + Math.random() * 40,
      radius: 20 + Math.random() * 30,
      shape: ['rectangle', 'circle', 'triangle'][Math.floor(Math.random() * 3)],
      color: ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF'][Math.floor(Math.random() * 5)],
      label: `Object ${detectedObjects.length + 1}`
    };
    
    setDetectedObjects(prev => [...prev, newObject]);
  };

  const detectObjects = () => {
    if (isDetecting) return;
    
    setIsDetecting(true);
    
    // Simulate detection process
    setTimeout(() => {
      // In a real system, this would use computer vision algorithms
      // For this simulation, we'll just return the existing objects
      setIsDetecting(false);
    }, 1500);
  };

  const clearScene = () => {
    setDetectedObjects([]);
  };

  return (
    <div className="perception-playground">
      <h3>Perception Playground</h3>
      
      <div className="playground-controls">
        <div className="tool-selector">
          <label>
            <input
              type="radio"
              name="tool"
              value="add"
              checked={selectedTool === 'add'}
              onChange={() => setSelectedTool('add')}
            />
            Add Objects
          </label>
          <label>
            <input
              type="radio"
              name="tool"
              value="detect"
              checked={selectedTool === 'detect'}
              onChange={() => setSelectedTool('detect')}
            />
            Detect Objects
          </label>
        </div>
        
        <div className="detection-mode">
          <label>
            Detection Mode:
            <select 
              value={detectionMode} 
              onChange={(e) => setDetectionMode(e.target.value)}
            >
              <option value="color">Color-based Detection</option>
              <option value="shape">Shape-based Detection</option>
              <option value="size">Size-based Detection</option>
            </select>
          </label>
        </div>
        
        <div className="action-buttons">
          <button onClick={detectObjects} disabled={isDetecting || selectedTool !== 'detect'}>
            {isDetecting ? 'Detecting...' : 'Run Detection'}
          </button>
          <button onClick={clearScene}>Clear Scene</button>
        </div>
      </div>
      
      <div className="canvas-container">
        <canvas
          ref={canvasRef}
          width={600}
          height={400}
          onClick={handleCanvasClick}
          style={{ border: '1px solid #ccc', background: '#f0f0f0' }}
        />
      </div>
      
      <div className="detection-results">
        <h4>Detection Results</h4>
        {detectedObjects.length === 0 ? (
          <p>No objects detected. Add objects or run detection.</p>
        ) : (
          <div className="object-grid">
            {detectedObjects.map(obj => (
              <div key={obj.id} className="object-card">
                <div 
                  className="object-preview" 
                  style={{ 
                    backgroundColor: obj.color,
                    width: '50px',
                    height: '50px',
                    display: 'inline-block',
                    marginRight: '10px'
                  }}
                />
                <div>
                  <strong>{obj.label}</strong><br/>
                  Shape: {obj.shape}<br/>
                  Position: ({Math.round(obj.x)}, {Math.round(obj.y)})
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
      
      <div className="explanation">
        <h4>Perception Concepts Demonstrated</h4>
        <ul>
          <li><strong>Object Detection</strong>: Identifying objects in the environment</li>
          <li><strong>Feature Extraction</strong>: Identifying object characteristics (color, shape, size)</li>
          <li><strong>Scene Understanding</strong>: Interpreting the spatial relationships between objects</li>
          <li><strong>Computer Vision</strong>: Algorithms for image processing and analysis</li>
        </ul>
        
        <p>
          In real robotic systems, perception involves sophisticated algorithms like convolutional neural networks (CNNs) 
          for object detection, SLAM for mapping and localization, and sensor fusion to combine data from multiple sources 
          (cameras, LIDAR, IMU, etc.).
        </p>
      </div>
    </div>
  );
};

export default PerceptionPlayground;
```

## Interactive Manipulation Simulator

```jsx
import React, { useState, useRef, useEffect } from 'react';

const ManipulationSimulator = () => {
  const canvasRef = useRef(null);
  const [armJoints, setArmJoints] = useState([
    { angle: 0, length: 80 },  // Shoulder
    { angle: 0, length: 70 },  // Elbow
    { angle: 0, length: 60 }   // Wrist
  ]);
  const [gripperOpen, setGripperOpen] = useState(true);
  const [heldObject, setHeldObject] = useState(null);
  const [objects, setObjects] = useState([
    { id: 1, x: 300, y: 300, width: 30, height: 30, color: '#FF0000', label: 'Red Block' },
    { id: 2, x: 350, y: 250, width: 25, height: 40, color: '#00FF00', label: 'Green Cylinder' },
    { id: 3, x: 250, y: 320, width: 35, height: 25, color: '#0000FF', label: 'Blue Box' }
  ]);
  const [targetPosition, setTargetPosition] = useState({ x: 400, y: 100 });

  // Draw the robotic arm and environment
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw base
    ctx.fillStyle = '#808080';
    ctx.fillRect(50 - 20, canvas.height - 20, 40, 20);
    
    // Calculate arm positions
    let currentX = 50;
    let currentY = canvas.height - 20;
    let currentAngle = 0;
    
    // Draw arm links
    armJoints.forEach((joint, index) => {
      const newX = currentX + Math.cos(currentAngle + joint.angle) * joint.length;
      const newY = currentY - Math.sin(currentAngle + joint.angle) * joint.length;
      
      // Draw link
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 8;
      ctx.beginPath();
      ctx.moveTo(currentX, currentY);
      ctx.lineTo(newX, newY);
      ctx.stroke();
      
      // Draw joint
      ctx.fillStyle = '#666';
      ctx.beginPath();
      ctx.arc(currentX, currentY, 6, 0, Math.PI * 2);
      ctx.fill();
      
      currentX = newX;
      currentY = newY;
      currentAngle += joint.angle;
    });
    
    // Draw gripper
    const gripperSize = gripperOpen ? 15 : 5;
    ctx.fillStyle = '#333';
    ctx.fillRect(currentX - 10, currentY - gripperSize/2, 10, gripperSize);
    
    // Draw gripper fingers
    ctx.fillStyle = '#555';
    ctx.fillRect(currentX, currentY - gripperSize/2 - 5, 5, gripperSize + 10);
    ctx.fillRect(currentX, currentY - gripperSize/2 - 5 + (gripperOpen ? 15 : 5), 5, gripperSize + 10);
    
    // Draw held object if any
    if (heldObject) {
      const obj = objects.find(o => o.id === heldObject.id);
      if (obj) {
        ctx.fillStyle = obj.color;
        ctx.fillRect(
          currentX - obj.width/2, 
          currentY - obj.height/2, 
          obj.width, 
          obj.height
        );
      }
    }
    
    // Draw objects in the environment
    objects.forEach(obj => {
      if (obj.id !== heldObject?.id) { // Don't draw if held
        ctx.fillStyle = obj.color;
        ctx.fillRect(obj.x, obj.y, obj.width, obj.height);
        
        // Draw object label
        ctx.fillStyle = '#000';
        ctx.font = '12px Arial';
        ctx.fillText(obj.label, obj.x, obj.y - 5);
      }
    });
    
    // Draw target position
    ctx.fillStyle = 'rgba(0, 255, 0, 0.3)';
    ctx.beginPath();
    ctx.arc(targetPosition.x, targetPosition.y, 15, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#00FF00';
    ctx.lineWidth = 2;
    ctx.stroke();
  }, [armJoints, gripperOpen, objects, heldObject, targetPosition]);

  const moveJoint = (index, delta) => {
    setArmJoints(prev => {
      const newJoints = [...prev];
      newJoints[index].angle += delta;
      return newJoints;
    });
  };

  const toggleGripper = () => {
    if (heldObject) {
      // Release object
      setHeldObject(null);
    } else {
      // Check if we're close to an object to grasp
      const canvas = canvasRef.current;
      let currentX = 50;
      let currentY = canvas.height - 20;
      let currentAngle = 0;
      
      // Calculate end effector position
      armJoints.forEach(joint => {
        currentX = currentX + Math.cos(currentAngle + joint.angle) * joint.length;
        currentY = currentY - Math.sin(currentAngle + joint.angle) * joint.length;
        currentAngle += joint.angle;
      });
      
      // Check distance to objects
      for (const obj of objects) {
        const dx = currentX - (obj.x + obj.width/2);
        const dy = currentY - (obj.y + obj.height/2);
        const distance = Math.sqrt(dx*dx + dy*dy);
        
        if (distance < 30) { // Within grasp range
          setHeldObject(obj);
          break;
        }
      }
    }
    
    setGripperOpen(!gripperOpen);
  };

  const goToTarget = () => {
    // This would use inverse kinematics in a real system
    // For this simulation, we'll just move joints to approximate the target
    const targetX = targetPosition.x;
    const targetY = targetPosition.y;
    
    // Calculate angles to reach target (simplified)
    const dx = targetX - 50;
    const dy = (canvasRef.current.height - 20) - targetY;
    const distance = Math.sqrt(dx*dx + dy*dy);
    
    // Simple approximation - in reality this would use inverse kinematics
    const angle1 = Math.atan2(dy, dx);
    const angle2 = 0; // Simplified
    const angle3 = 0; // Simplified
    
    setArmJoints([
      { angle: angle1, length: 80 },
      { angle: angle2, length: 70 },
      { angle: angle3, length: 60 }
    ]);
  };

  const updateTarget = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    setTargetPosition({ x, y });
  };

  return (
    <div className="manipulation-simulator">
      <h3>Robotic Manipulation Simulator</h3>
      
      <div className="simulator-controls">
        <div className="joint-controls">
          <h4>Joint Controls</h4>
          {armJoints.map((joint, index) => (
            <div key={index} className="joint-control">
              <label>Joint {index + 1}:</label>
              <button onClick={() => moveJoint(index, -0.1)}>←</button>
              <button onClick={() => moveJoint(index, 0.1)}>→</button>
              <span>Angle: {(joint.angle * 180 / Math.PI).toFixed(1)}°</span>
            </div>
          ))}
        </div>
        
        <div className="gripper-control">
          <h4>Gripper Control</h4>
          <button onClick={toggleGripper}>
            {gripperOpen ? 'Close Gripper' : 'Open Gripper'}
          </button>
          <p>Status: {gripperOpen ? 'Open' : 'Closed'}</p>
          {heldObject && <p>Holding: {heldObject.label}</p>}
        </div>
        
        <div className="action-controls">
          <h4>Actions</h4>
          <button onClick={goToTarget}>Go to Target</button>
          <button onClick={() => setTargetPosition({ x: 400, y: 100 })}>Reset Target</button>
        </div>
      </div>
      
      <div className="canvas-container">
        <canvas
          ref={canvasRef}
          width={600}
          height={400}
          onClick={updateTarget}
          style={{ border: '1px solid #ccc', background: '#f9f9f9' }}
        />
        <p>Click on the canvas to set a new target position</p>
      </div>
      
      <div className="explanation">
        <h4>Manipulation Concepts Demonstrated</h4>
        <ul>
          <li><strong>Forward Kinematics</strong>: Calculating end-effector position from joint angles</li>
          <li><strong>Inverse Kinematics</strong>: Calculating joint angles to reach a target position</li>
          <li><strong>Grasping</strong>: Controlling the gripper to pick up objects</li>
          <li><strong>Manipulation Planning</strong>: Planning trajectories to avoid collisions</li>
        </ul>
        
        <p>
          In real robotic manipulation systems, precise control algorithms, force feedback, 
          and advanced grasp planning are used to reliably manipulate objects. 
          This simulation demonstrates the basic concepts of robotic arm control and manipulation.
        </p>
      </div>
    </div>
  );
};

export default ManipulationSimulator;
```

## How to Use These Components

To use these interactive components in your Docusaurus site, you would:

1. Create a new React component file in your `src/components` directory
2. Add the code for the component you want to use
3. Import and use the component in your markdown files:

```markdown
import ROS2Playground from '@site/src/components/ROS2Playground';
import NavigationSimulator from '@site/src/components/NavigationSimulator';
import PerceptionPlayground from '@site/src/components/PerceptionPlayground';
import ManipulationSimulator from '@site/src/components/ManipulationSimulator';

# Interactive Robotics Playground

<ROS2Playground />

<NavigationSimulator />

<PerceptionPlayground />

<ManipulationSimulator />
```

These interactive components provide hands-on experience with robotics concepts without requiring actual hardware or ROS 2 installation, making them ideal for educational purposes in the context of this book.