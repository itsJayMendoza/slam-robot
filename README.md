# diffbot_slam_ws (ROS 2 Jazzy + Gazebo Harmonic)

Build:   colcon build --symlink-install && source install/setup.bash
Run:     ros2 launch diffbot_bringup slam_sim.launch.py
Drive:   ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save map: ros2 run nav2_map_server map_saver_cli -f ~/map
In RViz: set Fixed Frame to "map", add Map (/map), LaserScan (/scan), RobotModel, TF.

Packages: diffbot_description (URDF/world), diffbot_bringup (launch + slam_toolbox config)

## Robot packages (ROS 2 Humble, built in Docker)

The robot runs Humble on Ubuntu 22.04 (Raspberry Pi 4B); the Gazebo sim above is Jazzy.
Humble packages are built in a container so they never mix with the Jazzy build:

    docker/run.sh "colcon build --build-base build_humble --install-base install_humble \
        --packages-select slam_robot_interfaces slam_robot_wifi_scanner"
    docker/run.sh "python3 -m pytest -q src/slam_robot_wifi_scanner/test"

On the Pi (native Humble), clone and build only the robot packages:

    colcon build --packages-select slam_robot_interfaces slam_robot_wifi_scanner

Packages: slam_robot_interfaces (actions/messages), slam_robot_wifi_scanner (passive
monitor-mode scanner, ScanAtWaypoint action).
