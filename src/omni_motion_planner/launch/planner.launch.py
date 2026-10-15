"""Kino A* + MINCO + MPPI/MPC 仿真测试 (定位/地图复用 omni_nav_bringup, 不起 Nav2).

  ros2 launch hospital_sim sim.launch.py
  ros2 launch omni_motion_planner planner.launch.py                    # MPPI + GICP 重定位
  ros2 launch omni_motion_planner planner.launch.py controller:=mpc localization:=static

RViz 用 "2D Goal Pose" (/goal_pose) 发目标.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('omni_motion_planner')
    bringup = get_package_share_directory('omni_nav_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mppi', description='mppi | mpc'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation.launch.py')),
            launch_arguments={'nav2': 'false',
                              'rviz_config': os.path.join(share, 'rviz', 'planner.rviz')}.items()),

        Node(package='omni_motion_planner', executable='planner_node', output='screen',
             parameters=[os.path.join(share, 'config', 'planner.yaml'),
                         {'use_sim_time': True, 'controller': LaunchConfiguration('controller')}]),
    ])
