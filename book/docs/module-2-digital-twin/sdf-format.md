---
sidebar_position: 9
---

# Simulation Description Format (SDF)

The Simulation Description Format (SDF) is an XML-based format used by Gazebo to describe simulation environments, robot models, and objects. SDF provides a complete description of the physics, visual properties, and plugins needed for simulation.

## SDF Overview

SDF is designed to be:
- **Flexible**: Accommodates various simulation scenarios
- **Extensible**: Allows custom elements and plugins
- **Hierarchical**: Organizes elements in a logical structure
- **Human-readable**: Clear XML format for easy understanding

## SDF File Structure

A basic SDF file structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- World elements go here -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Custom models -->
    <model name="my_robot" placement_frame="world">
      <!-- Model elements go here -->
    </model>
  </world>
</sdf>
```

## World Elements

### Physics Engine Configuration
Configure the physics simulation parameters:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Models in Worlds
Define models within the world:

```xml
<model name="simple_box">
  <pose>0 0 0.5 0 0 0</pose>  <!-- x, y, z, roll, pitch, yaw -->
  <static>false</static>      <!-- Whether the model is static -->
  
  <link name="box_link">
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.083</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.083</iyy>
        <iyz>0</iyz>
        <izz>0.083</izz>
      </inertia>
    </inertial>
    
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
    
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

## Model Elements

### Links
Links represent rigid bodies in the model:

```xml
<link name="link_name">
  <!-- Inertial properties for physics simulation -->
  <inertial>
    <mass>1.0</mass>
    <pose>0 0 0 0 0 0</pose>
    <inertia>
      <ixx>0.1</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.1</iyy>
      <iyz>0.0</iyz>
      <izz>0.1</izz>
    </inertia>
  </inertial>
  
  <!-- Visual properties for rendering -->
  <visual name="visual_name">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>0.5 0.5 0.5</size>
      </box>
    </geometry>
    <material>
      <script>
        <uri>file://media/materials/scripts/gazebo.material</uri>
        <name>Gazebo/Blue</name>
      </script>
    </material>
  </visual>
  
  <!-- Collision properties for physics -->
  <collision name="collision_name">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>0.5 0.5 0.5</size>
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
        <threshold>100000.0</threshold>
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1000000000000.0</kp>
          <kd>1.0</kd>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

### Joints
Joints connect links and define their relative motion:

```xml
<joint name="joint_name" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>
  <pose>0 0 0 0 0 0</pose>
  
  <axis>
    <xyz>0 0 1</xyz>  <!-- Rotation axis -->
    <limit>
      <lower>-1.57</lower>      <!-- Lower limit (radians) -->
      <upper>1.57</upper>       <!-- Upper limit (radians) -->
      <effort>100.0</effort>    <!-- Maximum effort (N-m) -->
      <velocity>1.0</velocity>  <!-- Maximum velocity (rad/s) -->
    </limit>
    <dynamics>
      <damping>0.1</damping>    <!-- Damping coefficient -->
      <friction>0.0</friction>  <!-- Joint friction -->
    </dynamics>
  </axis>
</joint>
```

## Common Joint Types

### Revolute Joint
Rotates around a single axis:

```xml
<joint name="hinge" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
  </axis>
</joint>
```

### Prismatic Joint
Linear motion along an axis:

```xml
<joint name="slider" type="prismatic">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>1 0 0</xyz>
    <limit>
      <lower>-0.5</lower>
      <upper>0.5</upper>
      <effort>200</effort>
      <velocity>0.5</velocity>
    </limit>
  </axis>
</joint>
```

### Fixed Joint
No relative motion (rigid connection):

```xml
<joint name="fixed_joint" type="fixed">
  <parent>link1</parent>
  <child>link2</child>
</joint>
```

## Sensors in SDF

### Camera Sensor
```xml
<sensor name="camera" type="camera">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <camera name="cam">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>image_raw</topic_name>
  </plugin>
</sensor>
```

### LiDAR Sensor
```xml
<sensor name="laser" type="ray">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
        <max_angle>1.570796</max_angle>    <!-- 90 degrees -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
    <topic_name>scan</topic_name>
    <frame_name>laser_frame</frame_name>
  </plugin>
</sensor>
```

## Including Models

### Standard Models
Include pre-built models from Gazebo's model database:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
<include>
  <uri>model://sun</uri>
</include>
<include>
  <uri>model://cylinder</uri>
  <pose>1 0 0 0 0 0</pose>
</include>
```

### Custom Models
Include models from custom paths:

```xml
<include>
  <uri>model://my_custom_robot</uri>
  <pose>0 0 0.5 0 0 0</pose>
  <name>custom_robot_1</name>
</include>
```

## Plugins

### ROS Control Plugin
Integrate with ROS control system:

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/my_robot</robotNamespace>
  <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
</plugin>
```

### IMU Plugin
Simulate IMU sensor:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <topicName>imu/data</topicName>
    <bodyName>imu_link</bodyName>
    <frameName>imu_frame</frameName>
    <gaussianNoise>0.01</gaussianNoise>
  </plugin>
</sensor>
```

## Best Practices for SDF

### 1. Organize Hierarchically
Structure your SDF files logically with clear grouping of related elements.

### 2. Use Meaningful Names
Name elements descriptively to make SDF files easier to understand and debug.

### 3. Validate Your SDF
Use the `gz sdf` command to validate your SDF files:
```bash
gz sdf -k model.sdf  # Check model file
gz sdf -k world.world  # Check world file
```

### 4. Separate Concerns
Use separate SDF files for different components when possible to improve modularity.

### 5. Document Complex Models
Add comments to explain complex SDF structures and configurations.

SDF is a powerful format that enables detailed specification of simulation environments and robot models. Understanding its structure and elements is essential for creating effective robotic simulations in Gazebo.