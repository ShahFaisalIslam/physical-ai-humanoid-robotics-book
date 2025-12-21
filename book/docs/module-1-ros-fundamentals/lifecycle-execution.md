---
sidebar_position: 7
---

# ROS 2 Lifecycle and Execution Model

Understanding the ROS 2 lifecycle and execution model is crucial for developing robust robotic applications. This section explains how ROS 2 nodes are managed, executed, and how they interact with the system.

## ROS 2 Execution Model

ROS 2 uses a distributed execution model where nodes can run on different processes, devices, or even networks. Unlike the original ROS, ROS 2 does not require a central master node, making it more robust and suitable for production environments.

### Node Initialization and Execution

The typical lifecycle of a ROS 2 node involves several phases:

1. **Initialization**: The node is created and configured
2. **Activation**: The node becomes active and can participate in communication
3. **Execution**: The node runs and processes messages, services, etc.
4. **Shutdown**: The node is properly cleaned up

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # 1. Initialize ROS communications
    rclpy.init(args=args)
    
    # 2. Create node instance
    node = MyNode()
    
    # 3. Execute node (blocking call)
    rclpy.spin(node)
    
    # 4. Clean up
    node.destroy_node()
    rclpy.shutdown()
```

## Execution Contexts

ROS 2 provides different execution contexts to handle callbacks:

### Single-threaded Executor
The default executor that processes callbacks sequentially in a single thread:

```python
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
executor.shutdown()
```

### Multi-threaded Executor
Processes callbacks in multiple threads, allowing for concurrent execution:

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
executor.shutdown()
```

## Lifecycle Nodes

For more complex systems requiring explicit state management, ROS 2 provides LifecycleNodes:

### Lifecycle States
- **Unconfigured**: Node is created but not configured
- **Inactive**: Node is configured but not active
- **Active**: Node is running and participating in communication
- **Finalized**: Node is shut down

### State Transitions
- Unconfigured → Inactive (configure transition)
- Inactive → Active (activate transition)
- Active → Inactive (deactivate transition)
- Active → Finalized (cleanup transition)
- Inactive → Finalized (cleanup transition)

### Lifecycle Node Example

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
        
    def on_configure(self, state):
        self.get_logger().info(f'Configuring from state: {state}')
        # Initialize resources here
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):
        self.get_logger().info(f'Activating from state: {state}')
        # Activate components
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(self, state):
        self.get_logger().info(f'Deactivating from state: {state}')
        # Deactivate components
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state):
        self.get_logger().info(f'Cleaning up from state: {state}')
        # Clean up resources
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = MyLifecycleNode()
    
    # Initially in unconfigured state
    node.trigger_configure()  # Transition to inactive
    node.trigger_activate()   # Transition to active
    # ... do work ...
    node.trigger_deactivate() # Transition back to inactive
    node.trigger_cleanup()    # Transition back to unconfigured
    node.trigger_shutdown()   # Finalize the node
    
    rclpy.shutdown()
```

## Callback Groups

Callback groups allow you to control which callbacks are executed together:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup

# Mutually exclusive: only one callback in the group runs at a time
cb_group = MutuallyExclusiveCallbackGroup()

# Reentrant: multiple callbacks in the group can run simultaneously
reentrant_group = ReentrantCallbackGroup()

# Use in creating subscriptions/services
sub = self.create_subscription(
    String,
    'topic',
    callback,
    10,
    callback_group=cb_group
)
```

## Spin Variants

ROS 2 provides different spin methods for different use cases:

### Standard Spin
```python
rclpy.spin(node)  # Blocks until node is shut down
```

### Spin Once
```python
rclpy.spin_once(node, timeout_sec=1.0)  # Process available callbacks once
```

### Spin Until Future Complete
```python
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)  # Wait for service call
```

## Threading Model

ROS 2 uses a flexible threading model:

- **Node-local threading**: Each node can have its own executor
- **Callback-based**: Callbacks are executed by executors
- **Waitable objects**: Timers, subscriptions, services, etc. are all waitable
- **Concurrent processing**: Multiple executors can run concurrently

## Quality of Service (QoS) and Execution

QoS settings affect how messages are handled during execution:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable communication with last 10 messages
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
```

## Best Practices for Execution

1. **Choose the right executor**: Use single-threaded for simple nodes, multi-threaded for complex ones
2. **Use lifecycle nodes**: For complex systems requiring explicit state management
3. **Handle shutdown gracefully**: Always clean up resources in node destruction
4. **Use appropriate QoS settings**: Match communication requirements to application needs
5. **Avoid blocking in callbacks**: Keep callbacks lightweight to maintain responsiveness
6. **Use callback groups**: To control execution order and concurrency

Understanding the ROS 2 lifecycle and execution model is essential for creating robust and efficient robotic applications that can handle complex state transitions and concurrent operations.