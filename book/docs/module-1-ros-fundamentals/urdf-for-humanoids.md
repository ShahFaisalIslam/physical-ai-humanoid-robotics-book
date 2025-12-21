---
sidebar_position: 4
---

# URDF for Humanoids

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. For humanoid robots, URDF is essential for defining the physical structure, kinematic properties, and visual representation of the robot.

## What is URDF?

URDF is an XML format that describes robot models, including:
- Physical structure (links and joints)
- Kinematic properties (mass, inertia, limits)
- Visual representation (meshes, colors)
- Collision properties

## Basic URDF Structure

A URDF file has a root `<robot>` element containing `<link>` and `<joint>` elements:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Joints connect links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
  
  <link name="child_link">
    <!-- Child link definition -->
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:

### Visual Elements
Define how the link appears in visualizations:
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
    <!-- or <cylinder radius="0.1" length="0.2"/> -->
    <!-- or <sphere radius="0.1"/> -->
    <!-- or <mesh filename="package://path/to/mesh.stl"/> -->
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Elements
Define collision properties:
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Elements
Define physical properties:
```xml
<inertial>
  <mass value="0.1"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

## Joints

Joints connect links and define their motion. Common joint types:

### Revolute Joints
Rotate around a single axis (like a hinge):
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="100" velocity="1"/>
</joint>
```

### Fixed Joints
Rigid connection with no movement:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base"/>
  <child link="sensor_mount"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

### Continuous Joints
Rotate continuously around an axis:
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Humanoid Robot Structure

A humanoid robot typically has a hierarchical structure:

```
root (base_link)
├── pelvis
    ├── left_leg
    │   ├── left_lower_leg
    │   └── left_foot
    ├── right_leg
    │   ├── right_lower_leg
    │   └── right_foot
    ├── torso
        ├── neck
        │   └── head
        ├── left_arm
        │   ├── left_lower_arm
        │   └── left_hand
        └── right_arm
            ├── right_lower_arm
            └── right_hand
```

## URDF for a Simple Humanoid

Here's an example URDF for a simplified humanoid:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Pelvis (root link) -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="revolute">
    <parent link="pelvis"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="-0.15 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.2 0.6 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Additional joints and links would continue in this pattern -->
</robot>
```

## Xacro for Complex Models

For complex humanoid robots, Xacro (XML Macros) is often used to make URDF files more manageable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <xacro:property name="M_PI" value="3.1415926535897931" />
  
  <!-- Define a macro for a simple arm -->
  <xacro:macro name="simple_arm" params="prefix parent_link x y z roll pitch yaw">
    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
    </joint>

    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:simple_arm prefix="left" parent_link="torso" x="-0.15" y="0" z="0.2" roll="0" pitch="0" yaw="0"/>
  <xacro:simple_arm prefix="right" parent_link="torso" x="0.15" y="0" z="0.2" roll="0" pitch="0" yaw="0"/>
</robot>
```

## Best Practices for Humanoid URDFs

1. **Use appropriate units**: All distances in meters, angles in radians
2. **Set realistic inertial properties**: Mass and inertia affect simulation accuracy
3. **Consider joint limits**: Set realistic limits based on human anatomy
4. **Use visual and collision elements**: Separate for better performance
5. **Validate your URDF**: Use tools like `check_urdf` to verify syntax
6. **Organize complex models**: Use Xacro to avoid repetition

URDF is fundamental to humanoid robotics in ROS 2, providing the necessary description for simulation, visualization, and control algorithms.