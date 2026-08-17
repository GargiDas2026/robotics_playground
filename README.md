# Planar 2R Manipulator — ROS 2 Robotics Pipeline

> **Beginner Robotics + ROS 2 Project**
>
> This is a beginner-level project to understand and replicate a complete **robotics + ROS 2 pipeline** using **Python and C++**.
>
> The goal is to understand how a robotics system is built step-by-step — starting from robot modeling and ROS 2 communication, then moving through kinematics and eventually toward control.
>
> I will be posting **weekly updates** as the project develops.

---

## Project Goal

The goal of this project is to build a complete **planar 2R robotic manipulator** in ROS 2 while understanding how the different components of a robotics system communicate and work together.

Rather than building everything at once, the project is being developed incrementally:

```text
Robot Description
       ↓
ROS 2 + TF2
       ↓
Forward Kinematics
       ↓
Inverse Kinematics
       ↓
Jacobian
       ↓
Trajectory / Control
       ↓
Closed-Loop Robot System
```

The project focuses on both:

1. Robotics mathematics
2. Practical ROS 2 implementation

The intention is to understand not only the equations, but also how those equations become ROS 2 nodes that communicate with the rest of the system.
----
## Robot
The robot used in this project is a simple planar 2R manipulator with:

1. 2 revolute joints
2. 2 links
3. A fixed base
4. An end-effector frame

The kinematic structure is:

base_link
    │
    │ joint1
    ▼
  link1
    │
    │ joint2
    ▼
  link2
    │
    ▼
  ee_link


Current link dimensions:
Link 1 = 1.0 m
Link 2 = 0.8 m

## Technology Stack
#Operating System
    Ubuntu 24.04 LTS
#ROS 2
    ROS 2 Jazzy
    TF2
    RViz2
    robot_state_publisher
    joint_state_publisher_gui
    colcon
#Programming
    Python
    C++
#Robot Modeling
    URDF
    Xacro
