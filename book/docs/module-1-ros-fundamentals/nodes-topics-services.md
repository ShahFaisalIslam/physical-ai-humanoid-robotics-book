---
sidebar_position: 2
---

# Nodes, Topics, and Services

In ROS 2, the fundamental concepts of Nodes, Topics, and Services form the backbone of robot communication. Understanding these concepts is crucial for developing any robotic system.

## Nodes

A node is a process that performs computation. In ROS 2, nodes are designed to be modular, allowing you to break down complex robot behaviors into smaller, manageable pieces. Each node typically performs a specific task and communicates with other nodes through topics, services, or actions.

### Creating a Node

In Python, using the `rclpy` library, you create a node by subclassing `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Topics and Publishers/Subscribers

Topics enable a publish-subscribe communication model in ROS 2. This is an asynchronous communication method where publishers send messages to a topic without knowing who (if anyone) is subscribed to that topic.

### Publishers

A publisher sends messages to a specific topic. Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic.

```python
# Create a publisher
self.publisher = self.create_publisher(String, 'topic', 10)
```

### Subscribers

A subscriber receives messages from a specific topic. The callback function is executed whenever a message is received on that topic.

```python
def subscription_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)

# Create a subscriber
self.subscription = self.create_subscription(
    String,
    'topic',
    subscription_callback,
    10)
```

## Services

Services provide a request-response communication pattern. A client sends a request to a service and waits for a response. This is a synchronous communication method.

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Creating a Service Client

```python
class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Communication Patterns Comparison

| Pattern | Type | Use Case | Example |
|---------|------|----------|---------|
| Publisher/Subscriber | Asynchronous | Continuous data flow | Sensor data, robot state |
| Service/Client | Synchronous | Request/Response | Calculations, configuration |
| Action | Asynchronous with feedback | Long-running tasks | Navigation, manipulation |

## Best Practices

1. **Use meaningful names**: Choose descriptive names for your nodes, topics, and services
2. **Follow naming conventions**: Use lowercase with underscores (e.g., `robot_state`, `move_arm`)
3. **Handle failures gracefully**: Always check if services are available before calling them
4. **Use appropriate QoS settings**: Quality of Service settings affect message delivery guarantees
5. **Keep nodes focused**: Each node should have a single responsibility

## Example: Simple Publisher-Subscriber System

Here's a complete example of a publisher-subscriber system:

**Publisher**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from talker: %d' % self.get_clock().now().nanoseconds
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Service/Client Implementation

Here's a complete example of a service/client implementation:

**Service Server**:
```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client**:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    future = minimal_client.send_request(2, 3)

    try:
        rclpy.spin_until_future_complete(minimal_client, future)
        if future.result() is not None:
            response = future.result()
            minimal_client.get_logger().info(
                'Result of add_two_ints: %d' % response.sum)
        else:
            minimal_client.get_logger().info('Service call failed')
    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This communication model allows for flexible and decoupled robot architectures, where different parts of the system can be developed and tested independently.