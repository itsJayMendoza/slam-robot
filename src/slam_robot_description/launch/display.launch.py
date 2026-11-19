"""robot_state_publisher with the slam-robot URDF (for checking the model in RViz)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("slam_robot_description"), "urdf", "slam_robot.urdf.xacro"])
    description = ParameterValue(
        Command(["xacro ", xacro_file, " laser_yaw:=", LaunchConfiguration("laser_yaw")]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
    ])
