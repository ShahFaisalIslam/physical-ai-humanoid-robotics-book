---
sidebar_position: 10
---

# ROS 2 Architecture Diagrams

This section describes key ROS 2 architecture diagrams that help visualize the concepts discussed in this module.

## 1. Basic Publisher-Subscriber Architecture

```
[Node A] ---- Publish ----> [Topic: /chatter] <---- Subscribe ---- [Node B]
     |                           |                                      |
     |------ Publish -----> [Topic: /chatter] <----- Subscribe ------|
```

Description: Multiple nodes can publish to and subscribe from the same topic. The ROS 2 middleware handles message distribution.

## 2. Service Architecture

```
[Service Client] ---- Request ----> [Service Server]
         ^                                    |
         |--------- Response -----------------|
```

Description: Service clients send requests to service servers and wait for responses. This is a synchronous communication pattern.

## 3. Overall ROS 2 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Application Layer                  │
├─────────────────────────────────────────────────────────────┤
│  Node A          Node B          Node C          Node D    │
│  ├── Publisher   ├── Publisher   ├── Service     ├── Client │
│  ├── Subscriber  ├── Service     ├── Server              │
│  └── Timer       └── Client      └── Publisher           │
├─────────────────────────────────────────────────────────────┤
│                 ROS 2 Client Libraries                      │
│  (rclcpp, rclpy, rcljava, etc.)                           │
├─────────────────────────────────────────────────────────────┤
│                    ROS 2 Core                              │
│  ├── rcl (ROS Client Library)                             │
│  ├── rmw (ROS Middleware)                                 │
│  └── DDS Implementation (Fast DDS, Cyclone DDS, etc.)     │
└─────────────────────────────────────────────────────────────┘
```

Description: This diagram shows the layered architecture of ROS 2, from application nodes down to the underlying DDS middleware.

## 4. Lifecycle Node States

```
   ┌─────────────┐
   │ Unconfigured│
   └──────┬──────┘
          │ configure()
          ▼
   ┌─────────────┐
   │   Inactive  │◄────────────────────┐
   └──────┬──────┘                     │
          │ activate()                 │ deactivate()
          ▼                            │
   ┌─────────────┐                     │
   │    Active   │─────────────────────┤
   └──────┬──────┘                     │
          │ cleanup() or shutdown()    │
          ▼                            │
   ┌─────────────┐                     │
   │  Finalized  │◄────────────────────┘
   └─────────────┘
```

Description: The state machine for lifecycle nodes, showing possible states and transitions.

## 5. Launch File Structure

```
Launch File
     │
     ├─ Launch Argument: use_sim_time
     ├─ Launch Argument: robot_model
     │
     ├─ Node: robot_state_publisher
     │   └─ Parameters: [config_file, use_sim_time]
     │
     ├─ Node: controller_manager
     │   └─ Parameters: [config_file, use_sim_time]
     │
     └─ Node: laser_driver
         └─ Remappings: [scan:=front_scan]
```

Description: A launch file contains arguments, nodes, and other launch actions that are executed together when the launch file is run.