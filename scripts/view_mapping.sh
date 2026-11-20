#!/usr/bin/env bash
# Laptop (Jazzy) RViz for mapping: the map, the live scan and the robot model, following the robot.
# Start on the Pi first:  ros2 launch slam_robot_bringup mapping.launch.py
# The robot model's meshes are package:// paths; the laptop does not build the Humble packages, so a
# throwaway ament index pointing at src/slam_robot_description lets RViz find them.
for v in $(env | grep -E '^(GTK_|GIO_|GDK_|XDG_DATA_|LOCPATH|SNAP|GTK_EXE_PREFIX)' | cut -d= -f1); do unset "$v"; done
unset LD_LIBRARY_PATH_VSCODE_SNAP_ORIG
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="$(mktemp -d)"; trap 'rm -rf "$PREFIX"' EXIT
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
touch "$PREFIX/share/ament_index/resource_index/packages/slam_robot_description"
ln -s "$ROOT/src/slam_robot_description" "$PREFIX/share/slam_robot_description"
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH"
export ROS_DOMAIN_ID=42
rviz2 -d "$ROOT/rviz/mapping.rviz" "$@"
