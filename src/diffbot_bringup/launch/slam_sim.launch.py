import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    desc = share('diffbot_description')
    bringup = share('diffbot_bringup')
    robot_xml = xacro.process_file(os.path.join(desc, 'urdf', 'diffbot.urdf.xacro')).toxml()

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(desc, 'worlds', 'room.sdf')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_xml, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'diffbot', '-z', '0.1'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': os.path.join(bringup, 'config', 'bridge.yaml')}])

    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                parameters=[os.path.join(bringup, 'config', 'slam_params.yaml')])

    rviz = Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}])

    return LaunchDescription([gz, rsp, spawn, bridge, slam, rviz])
