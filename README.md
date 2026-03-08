# MoveIt + UR Robots Demo

A ROS 2 workspace demonstrating MoveIt motion planning with Universal Robots (UR) arms.

Supports three execution targets:
- **Fake controller** — fully simulated, no hardware required (Linux and Windows)
- **URsim** — UR's official robot simulator (Docker, Linux)
- **Real hardware** — via `ur_robot_driver` running somewhere on the network.

See [DOCKER.md](doc/DOCKER.md) for Docker/URsim setup and [ROBOSTACK.md](doc/ROBOSTACK.md) for Windows setup.

> **Windows note:** URsim requires the Docker setup.

---

## Nodes

### `fake_controller`
Simulates a UR joint trajectory controller without real hardware or `ros2_control`.
Accepts `FollowJointTrajectory` action goals, interpolates joint positions over time, and publishes them on `/joint_states` — giving MoveIt a complete plan-and-execute loop in simulation.

### `target_planner`
Subscribes to `target_pose` (`geometry_msgs/PoseStamped`), plans a collision-free trajectory with MoveIt, and executes it on receipt of a message on `execute_plan`.

### `kine_environment`
3D visualiser for URDF robot models. Subscribes to `/joint_states` to reflect the current robot state and to `/display_planned_path` to preview planned trajectories. In goal-planning mode, provides an interactive gizmo for setting target poses.

---

## Requirements

### Linux
```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-ur-description ros-jazzy-ur-moveit-config ros-jazzy-moveit
```
Or use the provided Docker image, which includes all dependencies.

### Windows
Install [robostack](https://robostack.github.io/index.html) and follow [ROBOSTACK.md](doc/ROBOSTACK.md).
The optional CLion integration assumes robostack is installed at `C:\robostack`.

---

## Building

The workspace has a root `CMakeLists.txt` for IDE integration, so pass `--base-paths src` to colcon:

```bash
# Linux
colcon build --symlink-install --base-paths src

# Windows
colcon build --merge-install --base-paths src
```

---

## Usage

**URDF visualisation only:**
```bash
ros2 launch ur_bringup display_robot.launch.py launch_rviz:=false
```

**MoveIt planning + fake controller (no hardware needed):**
```bash
ros2 launch ur_bringup move_robot.launch.py launch_rviz:=false
```

**MoveIt planning + real robot or URsim:**
```bash
ros2 launch ur_bringup move_robot.launch.py fake_controller:=false launch_rviz:=false
```

---

## Quick reference

```bash
# Start Docker environment
docker compose up --build
docker exec -it ursim-ros2_dev-1 bash -c "cd ros2_ws && bash"

# Build and source
colcon build --symlink-install --base-paths src
source install/setup.sh

# Check connectivity (sometimes nothing appears, try restarting container or resetting the daemon)
ros2 topic echo /joint_states --once

# Launch (fake controller)
ros2 launch ur_bringup move_robot.launch.py fake_controller:=true launch_rviz:=false

# Launch (URsim / real hardware)
ros2 launch ur_bringup move_robot.launch.py fake_controller:=false launch_rviz:=false
```

---

![kine environment](doc/screenshots/kine_control.png)
![URsim](doc/screenshots/ursim.png)
![External control](doc/screenshots/external_control.png)