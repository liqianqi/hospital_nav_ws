"""定位 (Point-LIO + 重定位) + Nav2 (先起仿真: ros2 launch hospital_sim sim.launch.py).

  ros2 launch omni_nav_bringup navigation.launch.py                       # FFT+GICP 自动全局重定位
  ros2 launch omni_nav_bringup navigation.launch.py localization:=static  # map->odom 用静态 TF
      (机器人从建图起点出发时可用, 或给 init_x/init_y/init_yaw)

TF: map --(gicp_relocalization | static)--> odom --(lio_odom_adapter)--> base_link
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

NAV2_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower',
]


def generate_launch_description():
    share = get_package_share_directory('omni_nav_bringup')
    params = LaunchConfiguration('params_file')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    sim = {'use_sim_time': True}

    use_nav2 = IfCondition(LaunchConfiguration('nav2'))
    nav2 = []
    remap = {
        'controller_server': [('cmd_vel', 'cmd_vel_nav')],
        'velocity_smoother': [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
    }
    for name in NAV2_NODES:
        pkg = {
            'controller_server': 'nav2_controller', 'smoother_server': 'nav2_smoother',
            'planner_server': 'nav2_planner', 'behavior_server': 'nav2_behaviors',
            'velocity_smoother': 'nav2_velocity_smoother', 'collision_monitor': 'nav2_collision_monitor',
            'bt_navigator': 'nav2_bt_navigator', 'waypoint_follower': 'nav2_waypoint_follower',
        }[name]
        nav2.append(Node(condition=use_nav2, package=pkg, executable=name, name=name, output='screen',
                         parameters=[params, sim], remappings=remap.get(name, [])))

    return LaunchDescription([
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser(
            '~/hospital_nav_ws/src/omni_nav_bringup/maps')),
        DeclareLaunchArgument('map_name', default_value='hospital'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('localization', default_value='gicp', description='gicp | static'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'nav.rviz')),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='false: 只起定位+地图 (给 omni_motion_planner 用)'),
        DeclareLaunchArgument('nav2_start_delay', default_value='20.0',
                              description='秒, 等重定位全局搜索结束后再激活 Nav2'),

        # ---------------- 里程计: Point-LIO -> odom->base_link ----------------
        Node(package='point_lio', executable='pointlio_mapping', name='laserMapping', output='screen',
             parameters=[os.path.join(share, 'config', 'point_lio_sim.yaml'), sim]),
        Node(package='omni_nav_bringup', executable='lio_odom_adapter', output='screen',
             parameters=[sim]),

        # ---------------- 全局定位: map->odom ----------------
        Node(condition=LaunchConfigurationEquals('localization', 'gicp'),
             package='gicp_relocalization', executable='gicp_relocalization_exec',
             output='screen', emulate_tty=True,
             parameters=[os.path.join(share, 'config', 'relocalization.yaml'), sim, {
                 'prior_pcd_file': PathJoinSubstitution([map_dir, [map_name, '.pcd']]),
             }]),
        Node(condition=LaunchConfigurationEquals('localization', 'static'),
             package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--x', LaunchConfiguration('init_x'), '--y', LaunchConfiguration('init_y'),
                        '--yaw', LaunchConfiguration('init_yaw'),
                        '--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim]),

        # ---------------- 地图 ----------------
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, sim, {
                 'yaml_filename': PathJoinSubstitution([map_dir, [map_name, '.yaml']])}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': ['map_server']}]),

        # ---------------- Nav2 ----------------
        *nav2,
        # 延时启动: 开机时重定位全局搜索(16 线程)+Point-LIO 初始化把 CPU 占满, 此时配置 Nav2,
        # Fast DDS 偶发丢失 change_state 的服务响应, 而 Jazzy 的 lifecycle_manager 没有服务超时,
        # 会永远卡在 "Configuring xxx", Nav2 起不来 (RViz 里只有灰色静态地图)。
        TimerAction(condition=use_nav2, period=LaunchConfiguration('nav2_start_delay'), actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[sim, {'autostart': True, 'node_names': NAV2_NODES,
                                   'bond_timeout': 10.0}]),
        ]),

        Node(condition=IfCondition(LaunchConfiguration('rviz')),
             package='rviz2', executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[sim]),
    ])
