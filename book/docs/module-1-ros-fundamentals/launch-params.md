---
sidebar_position: 8
---

# ROS 2 Launch Files and Parameter Management

Launch files and parameter management are essential for configuring and starting complex robotic systems in ROS 2. This section covers how to create launch files and manage parameters effectively.

## Launch Files

Launch files allow you to start multiple nodes with specific configurations simultaneously. They are written in Python using the `launch` package.

### Basic Launch File Structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    param_file = LaunchConfiguration('param_file')
    
    # Declare launch arguments
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='path/to/params.yaml',
        description='Path to parameter file'
    )
    
    # Define nodes
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker_node',
        parameters=[param_file],
        remappings=[
            ('chatter', 'custom_chatter')
        ]
    )
    
    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener_node',
        parameters=[param_file]
    )
    
    # Return launch description
    return LaunchDescription([
        param_file_arg,
        talker_node,
        listener_node
    ])
```

### Launch File Actions

Launch files support various actions:

#### Node Action
The primary action for starting ROS 2 nodes:

```python
from launch_ros.actions import Node

my_node = Node(
    package='my_package',
    executable='my_node_executable',  # or name='my_node' for default executable
    name='custom_node_name',
    parameters=[
        {'param1': 'value1'},
        '/path/to/params.yaml'
    ],
    remappings=[
        ('original_topic', 'new_topic'),
        ('original_service', 'new_service')
    ],
    arguments=['arg1', 'arg2'],
    output='screen'  # Output to screen instead of log file
)
```

#### Conditional Launch
Execute actions based on conditions:

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

use_sim_time = LaunchConfiguration('use_sim_time')

sim_node = Node(
    condition=IfCondition(use_sim_time),
    package='gazebo_ros',
    executable='gazebo'
)
```

#### Execute Process
Run non-ROS executables:

```python
from launch.actions import ExecuteProcess

rviz = ExecuteProcess(
    cmd=['rviz2', '-d', '/path/to/config.rviz'],
    output='screen'
)
```

### Launch Arguments

Launch arguments allow passing values to launch files:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Declare argument
sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
)

# Use argument
use_sim_time = LaunchConfiguration('use_sim_time')

# In node definition
node = Node(
    package='my_package',
    executable='my_node',
    parameters=[{'use_sim_time': use_sim_time}]
)
```

## Parameter Management

Parameters allow runtime configuration of nodes without recompilation.

### Parameter Declaration

In your node, declare parameters with default values:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('debug_mode', False)
        
        # Get parameter values
        self.param_value = self.get_parameter('param_name').value
        self.threshold = self.get_parameter('threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value
```

### Parameter Files (YAML)

Parameters can be defined in YAML files:

```yaml
# params.yaml
/**:  # Applies to all nodes
  ros__parameters:
    use_sim_time: false
    log_level: 'info'

my_node:  # Applies to specific node
  ros__parameters:
    frequency: 10.0
    topic_name: 'chatter'
    thresholds:
      min: 0.1
      max: 10.0
```

### Setting Parameters at Runtime

Parameters can be set in various ways:

#### In Launch Files
```python
my_node = Node(
    package='my_package',
    executable='my_node',
    parameters=[
        {'param1': 'value1'},
        {'param2': 42.0},
        '/path/to/params.yaml'
    ]
)
```

#### Command Line
```bash
# Set parameter when running node
ros2 run my_package my_node --ros-args --param param_name:=value

# Load parameters from file
ros2 run my_package my_node --ros-args --params-file params.yaml
```

#### Using ros2 param Command
```bash
# List parameters of a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value
ros2 param set /node_name param_name new_value

# Load parameters from file
ros2 param load /node_name params.yaml

# Dump parameters to file
ros2 param dump /node_name
```

## Advanced Launch Features

### Substitutions

Launch files support various substitutions for dynamic values:

```python
from launch.substitutions import TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Combine paths
config_path = PathJoinSubstitution([
    FindPackageShare('my_package'),
    'config',
    'my_config.yaml'
])

# Direct text
direct_text = TextSubstitution(text='my_value')

# In node definition
node = Node(
    package='my_package',
    executable='my_node',
    parameters=[config_path]
)
```

### Event Handlers

Handle events during launch execution:

```python
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import RegisterEventHandler

# Register an event handler
launch_description = LaunchDescription([
    my_node,
    RegisterEventHandler(
        OnProcessStart(
            target_action=my_node,
            on_start=[
                # Actions to perform when node starts
            ]
        )
    )
])
```

## Launch File Best Practices

1. **Modular design**: Break complex systems into multiple launch files
2. **Use arguments**: Make launch files configurable with launch arguments
3. **Parameter files**: Use YAML files for complex parameter configurations
4. **Descriptive names**: Use clear names for nodes and parameters
5. **Default values**: Provide sensible defaults for all arguments
6. **Documentation**: Comment your launch files to explain the purpose of each component

## Example: Complete Launch File for Humanoid Robot

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Parameters
    config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_robot_bringup'),
        'config',
        'humanoid_params.yaml'
    ])
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Perception nodes
    camera_driver = Node(
        package='camera_driver',
        executable='camera_node',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        controller_manager,
        camera_driver
    ])
```

Launch files and parameter management provide powerful tools for configuring and starting complex robotic systems. Understanding these concepts is essential for developing maintainable and configurable ROS 2 applications.