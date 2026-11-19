# Meshes

These STL files are the 3D-printed chassis the robot is built from, taken unchanged from
https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Plan-Navigation
(`Section9_Build_the_Robot/bumperbot_ws/src/bumperbot_description/meshes`), licensed under the
Apache License 2.0 (copy in `LICENSE`).

How the URDF uses them on this robot:
- `base_link.STL`: unchanged, wheels on its motor mounts
- `wheel_*_link.STL`: scaled from 65 mm to the 72 mm wheels (35 mm wide)
- `caster_front_link.STL`: the only caster (no rear caster on this robot)
- lidar and IMU at the chassis' own mount points
- `laser_link.STL`: an RPLIDAR A1 model, used as a stand-in for the YDLidar X3
- `imu_link.STL`: the IMU board
