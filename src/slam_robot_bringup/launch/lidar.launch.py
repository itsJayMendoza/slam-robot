"""YDLidar X3 driver on /dev/ydlidar -> /scan (frame laser_link). Needs ~/ydlidar_ws sourced
(see scripts/setup_ydlidar_pi.sh)."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(get_package_share_directory("slam_robot_bringup"), "config", "ydlidar_x3.yaml")
    return LaunchDescription([
        Node(package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
             name="ydlidar_ros2_driver_node", output="screen", parameters=[params]),
    ])
