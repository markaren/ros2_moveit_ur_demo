# MoveIt + UR Robots Demo

A ROS 2 workspace demonstrating MoveIt motion planning with Universal Robots (UR) arms.

Supports three execution targets:
- **Simulated joint controller** — fully simulated joint controller, no hardware required (works without Docker/URsim on Linux and Windows)
- **URsim** — UR's official robot simulator controlled via `ur_robot_driver` (Docker, Linux)
- **Real hardware** — via `ur_robot_driver` running somewhere on the network.

See [DOCKER.md](doc/DOCKER.md) for Docker/URsim setup and [ROBOSTACK.md](doc/ROBOSTACK.md) 
for Windows setup.

> **Windows note:** URsim requires the Docker setup.

---

## Nodes

### `simulated_controller`
Simulates a joint trajectory controller without real hardware or `ros2_control`.
Accepts `FollowJointTrajectory` action goals on `/simulated_joint_controller/follow_joint_trajectory`,
interpolates joint positions between waypoints using linear or cubic-Hermite interpolation,
and publishes the result on `/joint_states` at a fixed rate — giving MoveIt a complete
plan-and-execute loop in simulation. Also accepts direct position overrides on `joint_commands`
(`sensor_msgs/JointState`).

### `target_planner`
Exposes MoveIt planning and execution as three actions, mirroring the Plan / Execute /
Plan & Execute workflow of the RViz MotionPlanning panel:
- `plan` (`target_planner/action/Plan`): plan to a target pose and store the trajectory.
- `execute` (`target_planner/action/Execute`): execute the last stored plan.
- `plan_and_execute` (`target_planner/action/PlanAndExecute`): plan then immediately execute.

All goals are rejected while another action is in progress. Planning retries up to three
times on failure. MoveIt parameters (planning time, tolerances, velocity/acceleration
scaling) are exposed as ROS parameters and can be updated at runtime.

### `kine_environment`
3D visualiser for URDF robot models. Subscribes to `/joint_states` to reflect the
current robot state and to `/display_planned_path` to preview planned trajectories.
In goal-planning mode (`goal_planning:=true`), it provides an interactive gizmo for
setting target poses and an ImGui panel with Plan / Execute / Plan & Execute buttons,
planner settings sliders, joint sliders, and an end-effector position/orientation readout.

#### Gizmo controls

- **Q**: toggle between world/robot frame <br>
- **W**: set gizmo mode to translation <br>
- **E**: set gizmo mode to rotation <br>

---

## Requirements

### Linux
```bash
sudo apt install libglfw3-dev ros-jazzy-control-msgs ros-jazzy-ur-description ros-jazzy-ur-moveit-config ros-jazzy-moveit
```
Or use the provided Docker image, which includes all dependencies.

### Windows
Install [robostack](https://robostack.github.io/index.html) and follow [ROBOSTACK.md](doc/ROBOSTACK.md).
The optional CLion integration assumes robostack is installed at `C:\robostack`.

---

## Building

The workspace has a root `CMakeLists.txt` for IDE integration, so pass `--base-paths src` 
to colcon when calling colcon from the terminal:

```bash
# Linux
colcon build --symlink-install --base-paths src

# Windows
colcon build --merge-install --base-paths src
```

Otherwise, just invoke the `colcon_build` targets within the IDE.

---

## Usage

**URDF visualisation only:**
```bash
ros2 launch ur_bringup display_robot.launch.py launch_rviz:=false
```

**MoveIt planning + simulated joint controller (no hardware needed):**
```bash
ros2 launch ur_bringup move_robot.launch.py launch_rviz:=false
```

**MoveIt planning + real robot or URsim:**
```bash
ros2 launch ur_bringup move_robot.launch.py simulated_controller:=false launch_rviz:=false
```

---

## Quick reference

Premade UR/ROS tools:
```bash
# Start Docker environment
docker compose down # stop and remove old containers/volumes
docker compose up --build # start new containers (build if needed)

# Access URsim container shell in another terminal
docker exec -it ursim-ros2_dev-1 bash -c "cd ros2_ws && bash" 

# Check connectivity (sometimes nothing appears, try restarting container or resetting the daemon)
ros2 topic echo /joint_states --once

# Test default Moveit configuration (no custom nodes, just MoveIt + RViz)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

```

Custom workspace:
```bash
# Inside ursim container:

# Build custom workspace 
colcon build --symlink-install --base-paths src
source install/setup.sh 

# Launch (simulated controller - Runs without docker and URsim)
ros2 launch ur_bringup move_robot.launch.py sim_controller:=true launch_rviz:=false

# Launch (URsim / real hardware)
ros2 launch ur_bringup move_robot.launch.py sim_controller:=false launch_rviz:=false
```

---

![kine environment](doc/screenshots/kine_control.png)
![URsim](doc/screenshots/ursim.png)
![External control](doc/screenshots/external_control.png)