---
sidebar_position: 6
---

# Assessment Questions: ROS 2 Fundamentals

This section provides assessment questions to test your understanding of ROS 2 fundamentals.

## Multiple Choice Questions

1. What does ROS stand for?
   a) Robot Operating System
   b) Robotic Operational Software
   c) Remote Operating System
   d) Robot Operation Service

2. Which communication pattern is asynchronous in ROS 2?
   a) Services
   b) Topics/Publish-Subscribe
   c) Actions
   d) Parameters

3. What is the primary Python client library for ROS 2?
   a) rospy
   b) roslibpy
   c) rclpy
   d) pyros

4. In URDF, what element defines the physical properties of a link?
   a) `<visual>`
   b) `<collision>`
   c) `<inertial>`
   d) `<geometry>`

5. Which joint type allows continuous rotation around an axis?
   a) revolute
   b) prismatic
   c) fixed
   d) continuous

## Short Answer Questions

1. Explain the difference between a ROS 2 node and a process in the traditional computing sense.

2. What is the purpose of Quality of Service (QoS) settings in ROS 2?

3. Describe the main components of a URDF file and their purposes.

4. What is the role of the DDS (Data Distribution Service) in ROS 2?

5. Explain how parameters work in ROS 2 and why they are useful.

## Practical Application Questions

1. Design a simple ROS 2 system for a mobile robot with a laser scanner. Describe the nodes, topics, and their purposes.

2. Create a URDF snippet for a simple robotic arm with a base, upper arm, and lower arm connected by revolute joints.

3. Write a simple rclpy publisher that publishes the current timestamp to a topic called "current_time".

4. Explain how you would structure a ROS 2 package for a humanoid robot controller.

5. Describe how you would use launch files to start a complete robotic system with multiple nodes.

## Answers

### Multiple Choice Answers
1. a) Robot Operating System
2. b) Topics/Publish-Subscribe
3. c) rclpy
4. c) `<inertial>`
5. d) continuous

### Short Answer Answers

1. A ROS 2 node is a process that performs computation and communicates with other nodes through topics, services, or actions. While a traditional process is an independent unit of execution, a ROS 2 node is specifically designed to work within the ROS 2 ecosystem, using ROS 2's communication mechanisms.

2. Quality of Service (QoS) settings define the delivery guarantees for messages in ROS 2. They allow you to specify requirements like reliability (best effort vs reliable), durability (volatile vs transient local), and history (keep last N messages vs keep all messages).

3. URDF files have three main types of elements for each link:
   - `<visual>`: Defines how the link appears in visualizations
   - `<collision>`: Defines collision properties for physics simulation
   - `<inertial>`: Defines mass, center of mass, and inertia properties

4. DDS (Data Distribution Service) is the underlying middleware that ROS 2 uses for communication. It provides a standardized approach for real-time, scalable, and robust data exchange between nodes, handling the actual transmission of messages.

5. Parameters in ROS 2 allow nodes to be configured at runtime. They provide a way to modify node behavior without changing code. Parameters can be set at launch time or changed dynamically during execution, making systems more flexible and configurable.

## Scoring Guide

- Multiple Choice (5 questions): 1 point each
- Short Answer (5 questions): 3 points each
- Practical Application (5 questions): 5 points each

Total: 50 points

Scoring:
- 45-50 points: Excellent understanding
- 35-44 points: Good understanding
- 25-34 points: Satisfactory understanding
- Below 25 points: Needs further study