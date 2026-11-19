"""Robot model + ros2_control + diff_drive_controller for the slam-robot base.

    ros2 launch slam_robot_bringup base.launch.py                  # real robot (ESP32 on /dev/esp32)
    ros2 launch slam_robot_bringup base.launch.py use_mock:=true   # no hardware, for testing

Topics: /cmd_vel (geometry_msgs/Twist) in, /odom out, TF odom -> base_footprint -> ...
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution(
            [FindPackageShare("slam_robot_description"), "urdf", "slam_robot.urdf.xacro"]),
        " use_mock:=", LaunchConfiguration("use_mock"),
        " port:=", LaunchConfiguration("port"),
        " laser_yaw:=", LaunchConfiguration("laser_yaw"),
    ]), value_type=str)
    controllers = PathJoinSubstitution(
        [FindPackageShare("slam_robot_bringup"), "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[{"robot_description": description}, controllers],
        remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
                    ("/diff_drive_controller/odom", "/odom")],
        output="screen")
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": description}])
    joint_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    drive = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument("use_mock", default_value="false"),
        DeclareLaunchArgument("port", default_value="/dev/esp32"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        control_node, state_publisher, joint_broadcaster,
        # start the drive controller after the joint states are up
        RegisterEventHandler(OnProcessExit(target_action=joint_broadcaster, on_exit=[drive])),
    ])
