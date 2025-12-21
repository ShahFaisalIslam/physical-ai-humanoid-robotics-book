---
sidebar_position: 3
---

# rclpy Basics

rclpy is the Python client library for ROS 2. It provides the standard interface for Python programs to interact with ROS 2, allowing you to create nodes, publish and subscribe to topics, provide and use services, and more.

## Introduction to rclpy

rclpy is built on top of the ROS 2 client library (rcl) and provides a Pythonic interface to ROS 2 functionality. It handles the underlying complexities of ROS 2 communication, allowing you to focus on implementing your robot's behavior.

## Core Components

### Node

The Node is the fundamental building block in ROS 2. All communication in ROS 2 happens through nodes.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code goes here

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 communications
    node = MyNode()        # Create node
    rclpy.spin(node)       # Keep node running until shutdown
    node.destroy_node()    # Clean up resources
    rclpy.shutdown()       # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

### Creating Publishers

A publisher allows a node to send messages to a topic:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        # 10 is the queue size (how many messages to buffer if subscriber is slow)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {message}')
```

### Creating Subscribers

A subscriber allows a node to receive messages from a topic:

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)  # Queue size
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

### Creating Services

Services allow for request-response communication:

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Creating Clients

Clients call services:

```python
from example_interfaces.srv import AddTwoInts

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.send_request(2, 3)  # Example request

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Result: {response.sum}')
```

## Working with Parameters

Nodes can have configurable parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('threshold', 0.5)
        
        # Get parameter value
        param_value = self.get_parameter('param_name').get_parameter_value().string_value
        threshold = self.get_parameter('threshold').get_parameter_value().double_value
        
        self.get_logger().info(f'Parameter value: {param_value}, Threshold: {threshold}')
```

## Timers

Timers allow you to execute code at regular intervals:

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback {self.i}')
        self.i += 1
```

## Quality of Service (QoS)

QoS settings allow you to specify the delivery guarantees for your messages:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE    # or TRANSIENT_LOCAL
)

publisher = self.create_publisher(String, 'topic_name', qos_profile)
```

## Lifecycle Nodes

For more complex systems, you might want to use lifecycle nodes:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleExampleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_example_node')
        
    def on_configure(self, state):
        self.get_logger().info(f'Configuring from state: {state}')
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):
        self.get_logger().info(f'Activating from state: {state}')
        return TransitionCallbackReturn.SUCCESS
```

## Error Handling and Best Practices

1. **Always handle exceptions** when calling services or processing messages
2. **Use appropriate QoS settings** for your use case
3. **Initialize and shutdown properly** to prevent resource leaks
4. **Use logging** to track node behavior
5. **Follow naming conventions** for consistency
6. **Keep nodes focused** on a single responsibility

rclpy provides a powerful and flexible interface to ROS 2 functionality, enabling you to build sophisticated robotic applications in Python.