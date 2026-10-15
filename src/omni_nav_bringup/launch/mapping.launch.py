"""Point-LIO 建图 (先起仿真: ros2 launch hospital_sim sim.launch.py).

  ros2 launch omni_nav_bringup mapping.launch.py [map_name:=hospital]
  # 另开终端遥控 (全向: 按住 shift 横移)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # 建完:
  ros2 service call /save_map std_srvs/srv/Trigger          # 存 maps/<map_name>.pcd (退出时也会自动存)
  ros2 run omni_nav_bringup pcd_to_gridmap.py <maps>/hospital.pcd <maps>/hospital   # -> pgm/yaml

map 系 = 建图起点的 base_link (odom), 建图时 map->odom 为单位阵.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('omni_nav_bringup')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser(
            '~/hospital_nav_ws/src/omni_nav_bringup/maps')),
        DeclareLaunchArgument('map_name', default_value='hospital'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='point_lio', executable='pointlio_mapping', name='laserMapping', output='screen',
             parameters=[os.path.join(share, 'config', 'point_lio_sim.yaml'), {'use_sim_time': True}]),

        Node(package='omni_nav_bringup', executable='lio_odom_adapter', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'accumulate_map': True,
                 'map_voxel_size': 0.05,
                 'map_save_path': PathJoinSubstitution([map_dir, [map_name, '.pcd']]),
             }]),

        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),

        Node(condition=IfCondition(LaunchConfiguration('rviz')),
             package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'nav.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
