#!/usr/bin/env bash
# Build the YDLidar SDK and ydlidar_ros2_driver on the Pi (native Humble) into ~/ydlidar_ws.
# The upstream driver uses the Foxy-style declare_parameter("name") call, which does not compile on
# Humble, so the declares are patched to pass the variable's default. No sudo: the SDK installs
# to ~/ydlidar_ws/sdk_install. Use the result with: source ~/ydlidar_ws/install/setup.bash
set -euo pipefail
WS="$HOME/ydlidar_ws"
mkdir -p "$WS/src"; cd "$WS"
[ -d YDLidar-SDK ] || git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git
[ -d src/ydlidar_ros2_driver ] || git clone --depth 1 https://github.com/YDLIDAR/ydlidar_ros2_driver.git src/ydlidar_ros2_driver
mkdir -p YDLidar-SDK/build && (cd YDLidar-SDK/build && cmake -Wno-dev -DCMAKE_INSTALL_PREFIX="$WS/sdk_install" .. >/dev/null && make -j3 && make install)
python3 - <<PY
import re
p="$WS/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp"
L=open(p).read().split("\n")
for i,l in enumerate(L):
    m=re.match(r'(\s*)node->declare_parameter\("(\w+)"\);',l)
    if m:
        g=re.search(r'get_parameter\("%s",\s*(\w+)\)'%m.group(2),L[i+1]); assert g,(i,L[i+1])
        L[i]='%snode->declare_parameter("%s", %s);'%(m.group(1),m.group(2),g.group(1))
open(p,"w").write("\n".join(L))
PY
source /opt/ros/humble/setup.bash
CMAKE_PREFIX_PATH="$WS/sdk_install" colcon build --symlink-install \
  --cmake-args -DCMAKE_EXE_LINKER_FLAGS="-L$WS/sdk_install/lib"
