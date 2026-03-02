# Moveit demo

A ROS2 workspace showcasing usage of MoveIt with UR robots.

## Overview of Nodes in the packages

## fake_controller
A lightweight ROS 2 node that simulates a robot joint trajectory controller 
without requiring real hardware or Gazebo. 

`fake_controller` accepts FollowJointTrajectory action goals and interpolates joint positions over time,
publishing the resulting joint states on `/joint_states`.
This allows MoveIt to plan and "execute" trajectories in a purely simulated environment.

Replaces the functionality of `ros2_control_node` and its associated mock components and controllers.

Necessary on Windows, where `ros2_control` support is currently limited.
This node should be replaced with the standard ros2_control stack for real hardware.


## target_planner

target_planner listens for a target pose on the `target_pose` topic, plans a collision-free trajectory 
using MoveIt's MoveGroupInterface, and executes it when triggered via the `execute_plan` topic.

## kine_environment

A visualisation environment for URDF models that also allows users to publish target poses for the target_planner node.
Listens to `/joint_states` for showing the current state of the robot. Also listens to `/display_planned_path` for showing planned paths.

## Building

Since this workspace uses a root CMakeLists.txt for IDE integration, 
you need to specify the build directory when calling colcon from a terminal:

```bash
# Unix
colcon build --symlink-install --base-paths src
# Windows
colcon build ---merge-install --base-paths src
```

## Usage

Pure URDF visualisation:
```
ros2 launch ur_bringup display_robot.launch.py launch_rviz:=false|true
```

URDF visualisation and MoveIt planning:
```
ros2 launch ur_bringup move_robot.launch.py launch_rviz:=false|true
```

## Requirements

### Linux

```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-ur-description ros-jazzy-ur-movit-config ros-jazzy-moveit
```

### Windows
On Windows, using the [robostack](https://robostack.github.io/index.html) virtual environment is recommended to ensure compatibility with ROS 2 packages.

#### Install RoboStack using Pixi

See the [RoboStack installation guide](https://robostack.github.io/GettingStarted.html) for detailed instructions on setting up the virtual environment and installing ROS 2 packages.

You can use the following simplified `pixi.toml` in place of the one listed from robostack:

```toml
[workspace]
name = "robostack"
description = "Development environment for RoboStack ROS packages"
channels = ["https://prefix.dev/conda-forge"]
platforms = ["win-64"]

# This will automatically activate the ros workspace on activation
[target.win.activation]
scripts = ["install/setup.bat"]

# To build you can use - `pixi run -e <ros distro> build <Any other temporary args>`
[feature.build.target.win-64.tasks]
build = "colcon build --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython_FIND_VIRTUALENV=ONLY -DPython3_FIND_VIRTUALENV=ONLY"

# Dependencies used by all environments
[dependencies]
python = "*"
# Build tools
pkg-config = "*"
make = "*"
ninja = "*"
# ROS specific tools
rosdep = "*"
colcon-common-extensions = "*"
semver = "*"
filelock = "*"

# Define all the different ROS environments
# Each environment corresponds to a different ROS distribution
# and can be activated using the `pixi run/shell -e <environment>` command.
[environments]
jazzy = { features = ["jazzy", "build"] }
kilted = { features = ["kilted", "build"] }

### ROS Jazzy ####
[feature.jazzy]
channels = ["https://prefix.dev/robostack-jazzy"]

[feature.jazzy.dependencies]
ros-jazzy-desktop = "*"
ros-jazzy-ur-description = "*"
ros-jazzy-ur-moveit-config = "*"
ros-jazzy-moveit = "*"

### ROS Kilted ####
[feature.kilted]
channels = ["https://prefix.dev/robostack-kilted"]

[feature.kilted.dependencies]
ros-kilted-desktop = "*"
```

> Note: The (optional) CLion integration assumes robostack to be setup at `C:\robostack`.
