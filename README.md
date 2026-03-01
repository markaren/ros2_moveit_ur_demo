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


## Requirements

On Windows, using the robostack virtual environment is recommended to ensure compatibility with ROS 2 packages.

### Install RoboStack using Pixi

See the [RoboStack installation guide](https://robostack.github.io/GettingStarted.html) for detailed instructions on setting up the virtual environment and installing ROS 2 packages.
