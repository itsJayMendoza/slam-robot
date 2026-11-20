"""The whole robot on the Pi: base (ros2_control), IMU, EKF and, optionally, the lidar.

    ros2 launch slam_robot_bringup robot.launch.py                 # needs ~/ydlidar_ws sourced
    ros2 launch slam_robot_bringup robot.launch.py lidar:=false
    ros2 launch slam_robot_bringup robot.launch.py use_mock:=true lidar:=false imu:=false   # laptop test

Keep the robot still for the first 3 s: the IMU measures its gyro bias at start-up.
Out: /odometry/filtered, TF odom -> base_footprint (from the EKF), /scan.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = FindPackageShare("slam_robot_bringup")
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([here, "launch", "base.launch.py"])),
        launch_arguments={"use_mock": LaunchConfiguration("use_mock"), "use_ekf": "true"}.items())
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([here, "launch", "lidar.launch.py"])),
        condition=IfCondition(LaunchConfiguration("lidar")))
    imu = Node(
        package="slam_robot_imu", executable="imu_node", name="imu", output="screen",
        parameters=[PathJoinSubstitution([FindPackageShare("slam_robot_imu"), "config", "imu.yaml"])],
        condition=IfCondition(LaunchConfiguration("imu")))
    ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
        parameters=[PathJoinSubstitution([here, "config", "ekf.yaml"])])
    return LaunchDescription([
        DeclareLaunchArgument("use_mock", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("imu", default_value="true"),
        base, imu, lidar, ekf,
    ])
