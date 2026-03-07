#!/bin/bash
set -e

# Write ROS sourcing into .bashrc so every `docker exec ... bash` session
# gets a fully configured environment automatically
cat >> /root/.bashrc <<'BASHRC'

# ROS 2 environment
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export XDG_RUNTIME_DIR=/tmp/runtime
BASHRC

# WSLg runtime dir
mkdir -p /tmp/runtime

# Keep container alive so `docker exec -it docker-ros2_dev-1 bash` works
exec sleep infinity