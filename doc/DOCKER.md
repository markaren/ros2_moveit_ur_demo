# Docker setup for ursim + ur_robot_driver

The docker setup included in this repository runs URSim alongside the `ur_robot_driver`. This allows commanding a simulated UR robot using ROS2.
Currently unable to connect to ROS node from outside of docker, however, a development container is booted for this purpose.

## Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)

### Windows specific
- [vcxsrv](https://github.com/marchaesen/vcxsrv/releases) (Neded for running GUI apps like RViz within the dev container)



## Containers

| Container   | Description |
|-------------|-------------|
| `ursim`     | Universal Robots simulator, accessible at `http://localhost:6080` |
| `ur_driver` | ROS2 Jazzy `ur_robot_driver` connected to URSim |
| `ros2_dev`  | Interactive ROS2 development shell |

## Usage

We first build a common base image for `ur_driver` and `ros2_dev` containers:

```bash
docker build -t ros_ur_base:latest -f Dockerfile.base .
```
Run the above from the `/docker_files` directory. You only need to do this once.

To start the application framework, run (from any directory):

**Windows:**
```bash
docker compuse down # stop and remove old containers/volumes
docker compose up --build 
```
**Linux**
```bash                                                                                  
xhost +local:docker
docker compuse down
docker compose -f docker-compose.yml -f docker-compose.linux.yml up  
```

Wait a few seconds for URSim to boot and `ur_driver` to connect. The very first build might take some time.

### URsim configuration

User interface: http://localhost:6080/vnc.html

1. Power ON and START the robot from lower left menu option. Then exit menu.
2. Navigate to Installation ->  URCaps -> External Control. Insert `172.20.0.3` as the Host IP.
3. Navigate to Program. Click URCaps/External Control.
4. Navigate to Run. Press play.


### Development shell

Open an interactive shell with full access to all ROS2 nodes and topics:

```bash
docker exec -it ursim-ros2_dev-1 bash -c "cd ros2_ws && bash"
```

Inside the shell:

```bash
# First run: reset the daemon so it discovers live nodes
ros2 node list      # shows all ur_driver nodes
ros2 topic list     # shows all ur_driver topics
ros2 topic echo /joint_states --once  # test connectivity
```

### Test joint_trajectory_controller

First move the robot to HOME position within URsim, then run:
```bash
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

### Using MoveIt

_Note: Run XLaunch on Windows (install vcxsrv) before trying to run GUI apps within the dev container._

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
