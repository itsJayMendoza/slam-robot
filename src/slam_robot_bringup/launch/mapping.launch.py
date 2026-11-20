"""Build a map: the whole robot (base, IMU, EKF, lidar) plus slam_toolbox.

    ros2 launch slam_robot_bringup mapping.launch.py          # on the Pi, ~/ydlidar_ws sourced
    (laptop) scripts/view_mapping.sh                          # watch the map grow
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>   # save it (.pgm + .yaml)

Keep the robot still for the first 3 s (IMU calibration), then drive slowly with teleop.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = FindPackageShare("slam_robot_bringup")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([here, "launch", "robot.launch.py"])),
        launch_arguments={"use_mock": LaunchConfiguration("use_mock"),
                          "lidar": LaunchConfiguration("lidar"),
                          "imu": LaunchConfiguration("imu")}.items())
    slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
        output="screen", parameters=[PathJoinSubstitution([here, "config", "slam.yaml"])])
    return LaunchDescription([
        DeclareLaunchArgument("use_mock", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("imu", default_value="true"),
        robot, slam,
    ])
