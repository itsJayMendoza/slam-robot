#!/usr/bin/env bash
# Laptop (Jazzy) RViz showing the Pi's /scan. Start the lidar on the Pi first:
#   ros2 launch slam_robot_bringup lidar.launch.py
# The Pi runs ROS_DOMAIN_ID=42; this shell may default to something else.
# VS Code's snap environment breaks rviz2 (symbol lookup errors), so those variables are dropped.
for v in $(env | grep -E '^(GTK_|GIO_|GDK_|XDG_DATA_|LOCPATH|SNAP|GTK_EXE_PREFIX)' | cut -d= -f1); do unset "$v"; done
unset LD_LIBRARY_PATH_VSCODE_SNAP_ORIG
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
exec rviz2 -d "$(cd "$(dirname "$0")/.." && pwd)/rviz/lidar.rviz" "$@"
