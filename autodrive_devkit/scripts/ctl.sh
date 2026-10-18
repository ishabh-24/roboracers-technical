#!/bin/bash
# Convenience control script. Run inside the container:
#   ctl.sh stop              stop node and monitor, zero the commands
#   ctl.sh reset             reset the car to the start line
#   ctl.sh start [ros args]  start the node (e.g. ctl.sh start -p max_throttle:=0.5)
#   ctl.sh monitor           start the lap monitor in the background (log: /tmp/monitor.log)
#   ctl.sh record NAME       start recording to /home/autodrive_devkit/log_data/NAME.npz
#   ctl.sh status            tail the logs
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
S=/home/autodrive_devkit/src/autodrive_devkit/scripts
NODE_PAT="autodrive_roboracer/wall_follow"
zero() {
  ros2 topic pub --once /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null
  ros2 topic pub --once /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null
}
case "$1" in
  stop)
    pkill -f "$NODE_PAT"; pkill -f "scripts/recorder.py"; sleep 1; zero
    echo "stopped (node procs: $(pgrep -fc "$NODE_PAT"))";;
  reset)
    zero
    ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: true}" >/dev/null; sleep 1
    ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: false}" >/dev/null
    echo "reset sent";;
  start)
    shift
    if [ $# -gt 0 ]; then
      nohup ros2 run autodrive_roboracer wall_follow_node --ros-args "$@" > /tmp/node.log 2>&1 &
    else
      nohup ros2 run autodrive_roboracer wall_follow_node > /tmp/node.log 2>&1 &
    fi
    echo "node started";;
  monitor)
    pkill -f "scripts/monitor.py"; sleep 0.5
    nohup python3 $S/monitor.py > /tmp/monitor.log 2>&1 &
    echo "monitor started";;
  record)
    mkdir -p /home/autodrive_devkit/log_data
    nohup python3 $S/recorder.py /home/autodrive_devkit/log_data/$2.npz > /tmp/recorder.log 2>&1 &
    echo "recording to $2.npz";;
  status)
    echo "--- monitor ---"; tail -${2:-3} /tmp/monitor.log 2>/dev/null
    echo "--- node ---"; tail -${2:-3} /tmp/node.log 2>/dev/null | cut -c40-
    echo "--- recorder ---"; tail -1 /tmp/recorder.log 2>/dev/null;;
  *) echo "usage: ctl.sh stop|reset|start|monitor|record NAME|status";;
esac
