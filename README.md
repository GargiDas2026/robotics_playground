# Planar 2R Manipulator — ROS 2 Robotics Pipeline

> **Beginner Robotics + ROS 2 Project**
>
> This is a beginner-level project to understand and replicate a complete **robotics + ROS 2 pipeline** using **Python and C++**.
>
> The goal is to understand how a robotics system is built step-by-step, starting from robot modeling and ROS 2 communication, then moving through kinematics and eventually toward control.
>
> I will be posting **weekly updates** as the project develops.

---

## Project Goal

The goal of this project is to build a complete **planar 2R robotic manipulator** in ROS 2 while understanding how the different components of a robotics system communicate and work together.

The project is being developed incrementally:

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

- Robotics mathematics
- Practical ROS 2 implementation

The intention is to understand not only the equations, but also how those equations become ROS 2 nodes that communicate with the rest of the system.

---

## Robot

The robot used in this project is a simple planar 2R manipulator with:

1. 2 revolute joints
2. 2 links
3. A fixed base
4. An end-effector frame

The kinematic structure is:

```text
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
```

Current link dimensions:

- Link 1 = 1.0 m
- Link 2 = 0.8 m

---

## Technology Stack

### Operating System

- Ubuntu 24.04 LTS

### ROS 2

- ROS 2 Jazzy
- TF2
- RViz2
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `colcon`

### Programming

- Python
- C++

### Robot Modeling

- URDF
- Xacro

---

## Project Structure

The project is being organized into separate ROS 2 packages as the system develops.

Current structure:

```text
robotics_playground/
│
├── docs/
│   └── notes/
│
├── ros2_ws/
│   └── src/
│       │
│       ├── planar_2r_description/
│       │   ├── urdf/
│       │   │   ├── planar_2r.urdf.xacro
│       │   │   ├── properties.xacro
│       │   │   ├── materials.xacro
│       │   │   ├── links.xacro
│       │   │   └── joints.xacro
│       │   │
│       │   ├── launch/
│       │   │   └── display.launch.py
│       │   │
│       │   ├── rviz/
│       │   │   └── planar_2r.rviz
│       │   │
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       └── planar_2r_kinematics/
│           ├── planar_2r_kinematics/
│           │   ├── __init__.py
│           │   └── forward_kinematics_node.py
│           │
│           ├── resource/
│           │   └── planar_2r_kinematics
│           │
│           ├── test/
│           │   ├── test_copyright.py
│           │   ├── test_flake8.py
│           │   └── test_pep257.py
│           │
│           ├── package.xml
│           ├── setup.py
│           └── setup.cfg
│
├── scripts/
│
├── .gitignore
│
└── README.md
```
The `ros2_ws` directory is the ROS 2 workspace. ROS 2 packages are created inside `ros2_ws/src`.

The workspace is built from the workspace root:

```bash
cd ~/Documents/robotics_playground/ros2_ws
colcon build
```

The packages themselves are not built by running `colcon build` from inside an individual package directory.

Additional packages will be added as the project progresses.

---

# Phase 1 : Robot Description

## Status: Complete

The first phase was to create and visualize the planar 2R robot in ROS 2.

The robot was initially defined using URDF and then reorganized using Xacro.

The robot description was divided into separate files:

```text
planar_2r_description/
└── urdf/
    ├── planar_2r.urdf.xacro
    ├── properties.xacro
    ├── materials.xacro
    ├── links.xacro
    └── joints.xacro
```

### Xacro Properties

Robot dimensions and other reusable properties are defined separately.

For example:

```xml
<xacro:property name="link1_length" value="1.0"/>
<xacro:property name="link2_length" value="0.8"/>
```

This avoids hard-coding the same values throughout the robot description.

### Xacro Macros

Reusable Xacro macros were introduced for the robot components.

The links are created using a reusable cylindrical-link macro rather than repeating the complete visual, collision, and inertial definitions for every link.

The joints are similarly organized using a reusable joint macro.

This makes the robot description easier to read and extend.

### Robot Visualization

The robot is launched using:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz2

The joint-state GUI allows the two revolute joints to be moved interactively.

RViz displays the resulting robot configuration.

### TF Tree

The current robot frame structure is:

```text
base_link
    │
    └── link1
          │
          └── link2
                │
                └── ee_link
```

TF2 is used to represent the relationships between these frames.

This TF structure will also be used by the kinematics nodes.

---

# Phase 2 : Forward Kinematics

## Status: Complete

The second phase was to implement forward kinematics as an actual ROS 2 node.

A Python ROS 2 package was created:

```text
planar_2r_kinematics
```

with the node:

```text
forward_kinematics_node.py
```

### ROS 2 Communication

The FK node subscribes to:

```text
/joint_states
```

and receives the current joint positions:

```text
q1
q2
```

The basic information flow is:

```text
Joint State Publisher GUI
          │
          │ /joint_states
          ▼
Forward Kinematics Node
          │
          ▼
       q1, q2
```

### Robot Geometry from TF

Instead of hard-coding the link lengths inside the Python FK node, the node obtains the robot geometry from TF.

The node uses:

```text
link1 → link2
link2 → ee_link
```

to determine:

```text
L1 = 1.0 m
L2 = 0.8 m
```

The geometry is initialized from TF once when the FK node starts.

After initialization, the link lengths are stored as ROS 2 parameters and used by the FK calculation.

This keeps the URDF/Xacro robot description as the authoritative source of the robot geometry.

### Forward Kinematics Calculation

For the planar 2R manipulator, the end-effector position is calculated using:

$$
x_{EE}
=
L_1\cos(q_1)
+
L_2\cos(q_1+q_2)
$$

$$
y_{EE}
=
L_1\sin(q_1)
+
L_2\sin(q_1+q_2)
$$

Since the robot is planar:

$$
z_{EE}=0
$$

The Python ROS 2 node performs this calculation in real time using the current joint angles.

### FK Validation

The calculated EE position is validated against the position obtained directly from TF.

The comparison is:

```text
                  Our FK
                    │
                    ▼
              (x_FK, y_FK)
                    │
                    │ compare
                    ▼
              (x_TF, y_TF)
                    ▲
                    │
                   TF
```

Example result:

```text
q1 = 1.545
q2 = -1.409

FK = (0.8184, 1.1079)
TF = (0.8184, 1.1079)

error = 4.97e-16 m
```

The error is effectively zero at numerical precision.

This provides a direct validation of the implemented FK equations against the ROS 2 TF representation of the robot.

### EE Position Publisher

The FK node publishes the calculated end-effector position on:

```text
/ee_position
```

using:

```text
geometry_msgs/msg/Point
```

Therefore, the current FK node performs both:

**Subscription**

```text
/joint_states
```

**Publication**

```text
/ee_position
```

The current ROS 2 data flow is:

```text
              /joint_states
                    │
                    ▼
          ┌────────────────────┐
          │ Forward Kinematics │
          │       Node         │
          │                    │
          │ q1, q2             │
          │       ↓            │
          │       FK           │
          │       ↓            │
          │    x, y, z         │
          └─────────┬──────────┘
                    │
                    ▼
              /ee_position
```

This means other ROS 2 nodes can subscribe to `/ee_position` without needing to know how the FK calculation was implemented.

---

# Python and C++ in the Project

A goal of this project is to understand how Python and C++ can coexist within the same ROS 2 system.

The current plan is to use:

### Python

For:

- Forward kinematics
- Inverse kinematics
- Jacobian calculations
- Numerical experiments

### C++

For:

- Controller implementation
- Control-oriented nodes
- Future performance-sensitive components

ROS 2 nodes written in Python and C++ can communicate through the same ROS 2 topics, services, and messages.

For example:

```text
Python FK Node
      │
      │ /ee_position
      ▼
C++ Controller
```

The communication interface is defined by the ROS message type, not by the programming language used to implement the node.

---

# Current ROS 2 Pipeline

At the end of Phase 2, the system currently looks like:

```text
                 URDF / Xacro
                      │
                      ▼
             robot_state_publisher
                      │
                      ▼
                     TF
                      │
                      ▼
              Robot frame data


/joint_states
      │
      ▼
Forward Kinematics Node
      │
      ├── q1, q2
      │
      ├── L1, L2 from TF
      │
      ├── FK calculation
      │
      ▼
 /ee_position
```

The robot can currently be moved through the joint-state GUI, and the FK node calculates and publishes the corresponding EE position in real time.

---

# Next Phase — Inverse Kinematics

## Status: Next

The next stage is to implement inverse kinematics.

The basic goal will be:

```text
Desired EE Position
        │
        ▼
       IK
        │
        ▼
      q1, q2
```

The IK implementation will then be validated by feeding the calculated joint angles back through FK.

```text
Desired Position
       │
       ▼
      IK
       │
       ▼
   q1, q2
       │
       ▼
      FK
       │
       ▼
Reconstructed Position
```

The main concepts will include:

- Analytical inverse kinematics
- Multiple IK solutions
- Reachability
- Joint limits
- IK → FK validation

---

# Future Phases

After inverse kinematics, the project will gradually move toward:

### Jacobian

- Jacobian calculation
- Joint velocity and EE velocity relationships
- Singularities
- Basic manipulability analysis

### Trajectory Generation

- Desired EE trajectories
- Joint trajectories
- Time-dependent motion

### Controller

A C++ ROS 2 controller will eventually be developed to close the loop between desired motion and the robot.

### Closed-Loop Integration

The long-term goal is to connect the kinematics, trajectory generation, controller, and robot simulation into a complete feedback system.

These phases will be developed and documented incrementally.

---

# Weekly Development Log

This repository will be updated weekly as the project progresses.

## Week 1 — Robot Description

- Created ROS 2 workspace
- Created planar 2R robot description package
- Built the initial URDF
- Converted the robot description to Xacro
- Added Xacro properties
- Added Xacro macros
- Separated properties, materials, links, and joints
- Added Link 2 and Joint 2
- Added the EE link and fixed joint
- Created and saved RViz configuration
- Verified the TF tree

### Result

The planar 2R robot can be launched and visualized in RViz, and the joints can be moved using the joint-state GUI.

---

## Week 2 — Forward Kinematics

- Created the `planar_2r_kinematics` Python ROS 2 package
- Created the forward kinematics node
- Learned ROS 2 workspace, package, and node structure
- Subscribed to `/joint_states`
- Used TF2 to obtain robot geometry
- Removed hard-coded link lengths from the FK calculation
- Implemented analytical planar 2R FK
- Compared FK results against TF
- Achieved numerical agreement
- Added `/ee_position` publisher

### Validation

Example:

```text
FK position = (0.8184, 1.1079)
TF position = (0.8184, 1.1079)

Position error = 4.97e-16 m
```

---

# Project Philosophy

This project is intentionally being built step-by-step.

The goal is not to immediately create a complicated robotics framework, but to understand how each individual component works and then connect those components together.

The development approach is:

```text
Understand
    ↓
Implement
    ↓
Test
    ↓
Validate
    ↓
Integrate
    ↓
Extend
```

The project is therefore both a learning exercise and a practical demonstration of a robotics software pipeline.

---

# Current Status

| Component | Status |
|---|---|
| ROS 2 Workspace | Complete |
| URDF Robot Description | Complete |
| Xacro Properties | Complete |
| Xacro Macros | Complete |
| Links and Joints | Complete |
| EE Frame | Complete |
| TF2 | Complete |
| RViz Configuration | Complete |
| Forward Kinematics | Complete |
| `/joint_states` Subscription | Complete |
| `/ee_position` Publisher | Complete |
| Inverse Kinematics | Next |
| Jacobian | Planned |
| Trajectory Generation | Planned |
| C++ Controller | Planned |
| Closed-Loop Control | Planned |

---

# Author

**Gargi Das**

This is a personal beginner-level project to learn and replicate a complete robotics and ROS 2 pipeline using **Python and C++**, while gradually connecting robotics theory with practical implementation.
