---
sidebar_position: 5
---

# Exercises: ROS 2 Fundamentals

This section provides exercises to reinforce your understanding of ROS 2 fundamentals. These exercises are designed to help you apply the concepts learned in the previous chapters.

## Exercise 1: Basic Publisher-Subscriber

**Objective**: Create a publisher that publishes a counter value every second and a subscriber that prints the received value.

**Instructions**:
1. Create a publisher node that publishes integer values to the topic `counter_topic`
2. The publisher should increment a counter and publish its value every second
3. Create a subscriber node that subscribes to `counter_topic`
4. The subscriber should print the received counter value to the console
5. Test that both nodes work together

**Sample Solution Approach**:
- Use `std_msgs.msg.Int32` for the message type
- In the publisher, create a timer that increments a counter and publishes the value
- In the subscriber, create a callback that prints the received value

## Exercise 2: Custom Message

**Objective**: Create a custom message and use it in a publisher-subscriber system.

**Instructions**:
1. Define a custom message type with at least two fields (e.g., name and age)
2. Create a publisher that publishes instances of your custom message
3. Create a subscriber that receives and displays the custom message content
4. Build and run your nodes to verify they work correctly

**Hint**: Custom messages are defined in `.msg` files in a `msg/` directory of your package.

## Exercise 3: Service Implementation

**Objective**: Create a service that performs a calculation and a client that uses it.

**Instructions**:
1. Create a service that accepts two numbers and returns their product
2. Implement the service server that performs the multiplication
3. Create a client that calls the service with two numbers
4. Print the result from the service call

**Sample Service Definition** (in a `.srv` file):
```
float64 a
float64 b
---
float64 product
```

## Exercise 4: Parameter-Based Behavior

**Objective**: Create a node that changes its behavior based on parameters.

**Instructions**:
1. Create a node that publishes messages to a topic
2. Add a parameter that controls the publishing frequency
3. Add another parameter that controls the content of the messages
4. Test changing parameters at runtime using `ros2 param` commands

## Exercise 5: TF Tree for Humanoid

**Objective**: Design a URDF for a simple humanoid robot with at least 5 joints.

**Instructions**:
1. Create a URDF file for a humanoid with torso, head, two arms, and two legs
2. Define appropriate joints connecting the links
3. Set reasonable joint limits based on human anatomy
4. Add visual and collision elements for each link

## Exercise 6: Launch File

**Objective**: Create a launch file that starts multiple nodes together.

**Instructions**:
1. Create two simple nodes (e.g., a publisher and a service server)
2. Create a launch file that starts both nodes simultaneously
3. Add parameters to the launch file that get passed to the nodes
4. Test the launch file to ensure both nodes start correctly

## Self-Assessment Questions

1. What is the difference between a publisher-subscriber pattern and a service-client pattern?
2. When would you use a service versus a topic for communication?
3. What are the different types of joints available in URDF?
4. How do you handle parameters in rclpy nodes?
5. What is the purpose of the `rclpy.init()` function?
6. Explain the difference between visual and collision elements in URDF.
7. What are the main components of a ROS 2 node?
8. How do you create a timer in rclpy?

## Solutions

After attempting these exercises, compare your solutions with best practices:

- Proper error handling in all nodes
- Meaningful node and topic names
- Appropriate QoS settings for your use case
- Clean code organization following ROS 2 conventions
- Proper resource cleanup in node destruction