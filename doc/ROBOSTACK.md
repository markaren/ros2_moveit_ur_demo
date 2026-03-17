
## Install RoboStack using Pixi on Windows

See the [RoboStack installation guide](https://robostack.github.io/GettingStarted.html) for detailed instructions on setting up the virtual environment and installing ROS 2 packages.

> Advice: Create and place the pixi.toml in the folder `C:\robostack`

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

### ROS Jazzy ####
[feature.jazzy]
channels = ["https://prefix.dev/robostack-jazzy"]

[feature.jazzy.dependencies]
ros-jazzy-desktop = "*"
ros-jazzy-ur-description = "*"
ros-jazzy-ur-moveit-config = "*"
ros-jazzy-moveit = "*"

```

Run `pixi shell -e jazzy` to source the environment.