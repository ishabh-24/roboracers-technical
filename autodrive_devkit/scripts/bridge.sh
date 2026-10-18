#!/bin/bash
# Build the workspace and start the simulator bridge. Run inside the container.
set -e
source /opt/ros/humble/setup.bash
cd /home/autodrive_devkit
colcon build
source install/setup.bash
exec ros2 launch autodrive_roboracer bringup_headless.launch.py
