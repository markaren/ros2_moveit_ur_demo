# Moveit + UR robots demo

A ROS2 workspace showcasing usage of MoveIt with UR robots.

This repository contains a `fake_controller` node that allows for testing MoveIt planning and execution without requiring real robots or URsim.
Works on both Linux and Windows, but Windows users must robostack to ensure compatibility with ROS 2 packages. See [ROBOSTACK.md](doc/ROBOSTACK.md) for more details.

> Note: The (optional) CLion integration assumes robostack to be set up at `C:\robostack`.

A Docker setup is also included for users who want to interact with URsim or real hardware on Windows. 
See [DOCKER.md](doc/DOCKER.md) for instructions on using Docker.

## Overview of Nodes in the packages

### fake_controller
A lightweight ROS 2 node that simulates a robot joint trajectory controller 
without requiring real hardware or other controller packages like `ur_robot_driver`. 

It replaces the functionality of `ros2_control_node` and its associated mock components and controllers.
Necessary on Windows, where `ros2_control` support is currently limited.
Use `ur_robot_driver` to target real hardware or URsim.

`fake_controller` accepts FollowJointTrajectory action goals and interpolates joint positions over time,
publishing the resulting joint states on `/joint_states`.
This allows MoveIt to plan and "execute" trajectories in a purely simulated environment.


### target_planner

target_planner listens for a target pose on the `target_pose` topic, plans a collision-free trajectory 
using MoveIt's MoveGroupInterface, and executes it when triggered via the `execute_plan` topic.

### kine_environment

A visualisation environment for URDF models that also allows users to publish target poses for the target_planner node.
Listens to `/joint_states` for showing the current state of the robot. Also listens to `/display_planned_path` for showing planned paths.

## Building

Since this workspace uses a root CMakeLists.txt for IDE integration, 
you need to specify the build directory when calling `colcon` from a terminal:

```bash
# Unix
colcon build --symlink-install --base-paths src
# Windows
colcon build --merge-install --base-paths src
```

## Usage

Pure URDF visualisation:
```
ros2 launch ur_bringup display_robot.launch.py launch_rviz:=false|true 
```

URDF visualisation and MoveIt planning (fake controller):
```
ros2 launch ur_bringup move_robot.launch.py launch_rviz:=false|true
```

URDF visualisation and MoveIt planning (real robot/ URsim):
```
ros2 launch ur_bringup move_robot.launch.py fake_controller:=false launch_rviz:=false|true
```

## Requirements

### Linux

```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-ur-description ros-jazzy-ur-movit-config ros-jazzy-moveit
```

### Windows
On Windows, using the [robostack](https://robostack.github.io/index.html) virtual environment is recommended to ensure compatibility with ROS 2 packages.
See [ROBOSTACK.md](doc/ROBOSTACK.md) for more details.

However, native Windows support is only for interacting with the `fake_controller` node. To interact with URsim or real hardware, 
you will need to use the provided Docker setup or set up a Linux environment. See [DOCKER.md](doc/DOCKER.md) for instructions on using Docker.

# TLDR;

Some commands that you often need to run are:

```bash
docker compose up --build
docker exec -it ursim-ros2_dev-1 bash -c "cd ros2_ws && bash"

colcon build --symlink-install --base-paths src

ros2 topic echo /joint_states --once

source install/setup.sh

ros2 launch ur_bringup move_robot.launch.py fake_controller:=true launch_rviz:=false
ros2 launch ur_bringup move_robot.launch.py fake_controller:=false launch_rviz:=false
```


![](doc/screenshots/kine_control.png)
![](doc/screenshots/ursim.png)
![](doc/screenshots/external_control.png)
