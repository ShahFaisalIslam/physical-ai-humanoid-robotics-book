---
sidebar_position: 11
---

# Gazebo Plugin Development

Gazebo plugins extend the simulator's functionality, enabling custom behaviors, sensors, and integration with external systems like ROS. This section covers the fundamentals of Gazebo plugin development with practical examples.

## Plugin Architecture

Gazebo supports several types of plugins:
- **World plugins**: Affect the entire simulation world
- **Model plugins**: Attach to specific models
- **Sensor plugins**: Extend sensor capabilities
- **System plugins**: Core system extensions

## Basic Plugin Structure

### World Plugin Example

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class CustomWorldPlugin : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Store the world pointer for later use
      this->world = _world;
      
      // Read custom parameters from SDF
      if (_sdf->HasElement("custom_param"))
        this->customParam = _sdf->Get<double>("custom_param");
      else
        this->customParam = 1.0;  // Default value
      
      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomWorldPlugin::OnUpdate, this));
      
      gzmsg << "Custom world plugin loaded with param: " << this->customParam << std::endl;
    }

    public: void OnUpdate()
    {
      // Custom update logic here
      // This runs every simulation iteration
      
      // Example: Apply a force to all models
      for (auto model : this->world->Models())
      {
        // Apply a small upward force to all models
        math::Vector3 force(0, 0, 0.1);
        model->GetLink()->AddForce(force);
      }
    }

    private: physics::WorldPtr world;
    private: double customParam;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(CustomWorldPlugin)
}
```

### Model Plugin Example

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class CustomModelPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer
      this->model = _model;
      
      // Get a pointer to the first joint (for demonstration)
      this->joint = _model->GetJoint("joint_name");
      
      if (!this->joint)
      {
        gzerr << "No joint named 'joint_name' found in model '" 
              << _model->GetName() << "'. Plugin not loaded.\n";
        return;
      }
      
      // Listen to the update event to control the joint
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomModelPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Control the joint position
      if (this->joint)
      {
        // Example: Set joint to oscillate
        double time = this->model->GetWorld()->GetSimTime().Double();
        double targetPos = 0.5 * sin(time * 2.0); // Oscillate at 2 rad/s
        this->joint->SetPosition(0, targetPos);
      }
    }

    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomModelPlugin)
}
```

## ROS Integration Plugins

### ROS Publisher Plugin

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class ROSPublisherPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_ros_publisher_plugin",
                  ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("gazebo_ros_publisher_plugin"));
      
      // Create publisher
      this->pub = this->rosNode->advertise<std_msgs::Float64>("/model_position", 1);
      
      // Store model pointer
      this->model = _model;
      
      // Connect to update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ROSPublisherPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Get model position
      math::Pose pose = this->model->GetWorldPose();
      double zPos = pose.pos.z;
      
      // Publish to ROS
      std_msgs::Float64 msg;
      msg.data = zPos;
      
      this->pub.publish(msg);
    }

    private: physics::ModelPtr model;
    private: ros::Publisher pub;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(ROSPublisherPlugin)
}
```

### ROS Subscriber Plugin

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class ROSSubscriberPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Initialize ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_ros_subscriber_plugin",
                  ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("gazebo_ros_subscriber_plugin"));
      
      // Create subscriber
      this->sub = this->rosNode->subscribe("/model_force", 1,
          &ROSSubscriberPlugin::OnRosMsg, this);
      
      // Store model pointer
      this->model = _model;
    }

    private: void OnRosMsg(const std_msgs::Float64ConstPtr &_msg)
    {
      // Apply force to model based on ROS message
      math::Vector3 force(0, 0, _msg->data);
      this->model->GetLink()->AddForce(force);
    }

    private: physics::ModelPtr model;
    private: ros::Subscriber sub;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
  };

  GZ_REGISTER_MODEL_PLUGIN(ROSSubscriberPlugin)
}
```

## Sensor Plugin Example

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

namespace gazebo
{
  class CustomMagnetometerPlugin : public SensorPlugin
  {
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the magnetometer sensor
      this->parentSensor = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(_sensor);
      
      if (!this->parentSensor)
      {
        gzerr << "CustomMagnetometerPlugin requires a MagnetometerSensor.\n";
        return;
      }
      
      // Initialize ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_magnetometer_plugin",
                  ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node and publisher
      this->rosNode.reset(new ros::NodeHandle("gazebo_magnetometer_plugin"));
      this->pub = this->rosNode->advertise<sensor_msgs::MagneticField>("/magnetic_field", 1);
      
      // Connect to sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&CustomMagnetometerPlugin::OnUpdate, this));
      
      // Make sure the parent sensor is active
      this->parentSensor->SetActive(true);
    }

    public: void OnUpdate()
    {
      // Get magnetic field data from sensor
      ignition::math::Vector3d field = this->parentSensor->MagneticField();

      // Create and publish ROS message
      sensor_msgs::MagneticField msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = this->parentSensor->FrameId();
      
      msg.magnetic_field.x = field.X();
      msg.magnetic_field.y = field.Y();
      msg.magnetic_field.z = field.Z();
      
      // Add some noise to simulate real sensor
      static double noise_level = 1e-6;  // 1 microTesla
      msg.magnetic_field.x += (static_cast<double>(rand()) / RAND_MAX - 0.5) * noise_level;
      msg.magnetic_field.y += (static_cast<double>(rand()) / RAND_MAX - 0.5) * noise_level;
      msg.magnetic_field.z += (static_cast<double>(rand()) / RAND_MAX - 0.5) * noise_level;
      
      this->pub.publish(msg);
    }

    private: sensors::MagnetometerSensorPtr parentSensor;
    private: ros::Publisher pub;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_SENSOR_PLUGIN(CustomMagnetometerPlugin)
}
```

## Plugin Configuration in SDF

### Model with Plugin

```xml
<model name="custom_robot">
  <!-- Links and joints -->
  <link name="base_link">
    <inertial>
      <mass>1.0</mass>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual name="visual">
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Attach plugin to model -->
  <plugin name="custom_model_plugin" filename="libCustomModelPlugin.so">
    <param_name>param_value</param_name>
  </plugin>
</model>
```

### World with Plugin

```xml
<sdf version="1.7">
  <world name="custom_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Attach world plugin -->
    <plugin name="custom_world_plugin" filename="libCustomWorldPlugin.so">
      <custom_param>2.5</custom_param>
    </plugin>
    
    <!-- Models go here -->
  </world>
</sdf>
```

## Advanced Plugin Concepts

### Using Custom Messages

```cpp
// In your plugin
#include "custom_msgs/CustomMessage.h"  // Your custom message

// Publisher for custom message
this->customPub = this->rosNode->advertise<custom_msgs::CustomMessage>("/custom_topic", 1);

// In your update function
custom_msgs::CustomMessage msg;
msg.field1 = value1;
msg.field2 = value2;
this->customPub.publish(msg);
```

### Parameter Handling

```cpp
public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Required parameter with default
  this->topicName = _sdf->Get<std::string>("topic_name", "default_topic").first;
  
  // Optional parameter with validation
  if (_sdf->HasElement("update_rate"))
  {
    double rate = _sdf->Get<double>("update_rate");
    if (rate > 0 && rate <= 1000)  // Validate range
      this->updateRate = rate;
    else
      gzerr << "Invalid update rate: " << rate << std::endl;
  }
  else
  {
    this->updateRate = 100;  // Default 100 Hz
  }
}
```

### Error Handling and Logging

```cpp
public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Check for required elements
  if (!_sdf->HasElement("required_param"))
  {
    gzerr << "Missing required parameter 'required_param'\n";
    return;
  }
  
  // Validate model structure
  if (_model->GetJointCount() == 0)
  {
    gzerr << "Model has no joints, plugin requires joints\n";
    return;
  }
  
  // Log successful initialization
  gzmsg << "Plugin successfully loaded for model: " << _model->GetName() << std::endl;
}
```

## Build Configuration

### CMakeLists.txt for Plugin

```cmake
cmake_minimum_required(VERSION 3.5)
project(custom_gazebo_plugins)

# Find packages
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Add plugin library
add_library(CustomModelPlugin SHARED
  src/custom_model_plugin.cpp
)

# Link libraries
target_link_libraries(CustomModelPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Install plugin
install(TARGETS CustomModelPlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
```

## Best Practices

### 1. Resource Management
- Always clean up connections and resources
- Use smart pointers when possible
- Avoid memory leaks in long-running simulations

### 2. Thread Safety
- Gazebo plugins typically run in the main simulation thread
- Be careful with ROS callbacks that might run in different threads
- Use mutexes when sharing data between threads

### 3. Performance
- Keep update functions efficient
- Avoid heavy computations in the main simulation loop
- Use appropriate update rates

### 4. Error Handling
- Validate inputs and parameters
- Provide meaningful error messages
- Fail gracefully when possible

### 5. Testing
- Test plugins with various simulation scenarios
- Validate parameter handling
- Check behavior at simulation boundaries (start, stop, reset)

Gazebo plugins provide powerful extensibility for simulation environments. By following these examples and best practices, you can create custom behaviors, sensors, and integrations that enhance your robotic simulation capabilities.