#!/bin/bash
# Start the wall follower. Run inside the container. Extra args are passed to
# the node, e.g.  ./node.sh -p max_throttle:=0.6 -p wall:=left
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
if [ $# -gt 0 ]; then
  exec ros2 run autodrive_roboracer wall_follow_node --ros-args "$@"
else
  exec ros2 run autodrive_roboracer wall_follow_node
fi
