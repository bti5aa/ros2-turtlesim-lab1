# ROS 2 Jazzy Lab 1 — Turtlesim, Actions & Services

## 1. Project Overview

This project implements a ROS 2 application using Turtlesim to demonstrate custom interfaces, services, actions, feedback, and launch files.

The application includes a turtle navigation action, a reset service, and an action that draws a square by sending four navigation goals.

## 2. Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Turtlesim
- Python 3
- `colcon` build tools

## 3. Workspace Structure

```text
ros2_ws/
├── src/
│   ├── lab1_interfaces/
│   │   ├── action/
│   │   │   ├── DrawSquare.action
│   │   │   └── GoToPose.action
│   │   ├── msg/
│   │   │   └── TargetPose.msg
│   │   ├── srv/
│   │   │   └── ResetTurtle.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── lab1_controller/
│       ├── lab1_controller/
│       ├── launch/
│       │   └── lab1.launch.py
│       ├── config/
│       ├── package.xml
│       └── setup.py
├── build/
├── install/
└── log/
```

## 4. Custom Interfaces

### TargetPose.msg

Defines the target position and orientation:

- `x`: Target X coordinate
- `y`: Target Y coordinate
- `yaw`: Target orientation

### GoToPose.action

Requests the turtle to reach a target pose. It provides a success result and feedback containing the remaining distance.

### ResetTurtle.srv

Resets the turtle position and optionally clears the Turtlesim screen.

### DrawSquare.action

Requests a square using a specified side length and speed. Feedback reports the number of completed sides.

## 5. Build Instructions

Open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 6. Run the Application

Launch the application:

```bash
ros2 launch lab1_controller lab1.launch.py
```

The launch file starts Turtlesim and the project nodes.

## 7. Testing

### List available actions

```bash
ros2 action list
```

### Test the reset service

```bash
ros2 service call /reset_turtle lab1_interfaces/srv/ResetTurtle "{clear_screen: true}"
```

### Draw a square

```bash
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 2.0, speed: 1.0}" --feedback
```

### Change the square speed

```bash
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 2.0, speed: 0.5}" --feedback
```

### Test turtle navigation

```bash
ros2 action send_goal /goToPose lab1_interfaces/action/GoToPose "{target_pose: {x: 7.0, y: 7.0, yaw: 0.0}}" --feedback
```

## 8. Expected Results

- The launch file starts the required nodes.
- The reset service returns a success result when the reset completes.
- The navigation action reports progress and a final result.
- The square action reports completed sides and uses four navigation goals.

## 9. Notes

The ROS 2 environment must be sourced in each new terminal. The workspace should be rebuilt after changing source files or interface definitions.

## 10. Author

Student: [Your Name]

Course: ROS 2 Lab 1 — Turtlesim, Actions & Services
