"""医院场景 + 全向底盘 + 双 Mid360 + IMU 的 Gazebo (Harmonic) 仿真.

  ros2 launch hospital_sim sim.launch.py              # 带 gz GUI
  ros2 launch hospital_sim sim.launch.py gui:=false   # 无界面 (只跑 server, 省资源)
  ros2 launch hospital_sim sim.launch.py x:=-4.3 y:=-3.0 yaw:=1.5708   # 换出生点 (world 系)
  默认出生点 (0, 10, -90°) 在门厅, 也是建图起点 = map 系原点

输出:
  /livox/lidar (CustomMsg, 双雷达合并, 10Hz)  /livox/points (PointCloud2)  /livox/imu (200Hz)
  /ground_truth/odom (world->base_link 真值)  /joint_states (轮子)  /clock
输入:
  /cmd_vel (geometry_msgs/Twist, 全向: linear.x, linear.y, angular.z; 经加速度限幅后驱动 gz)
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawn_robot(context):
    # URDF -> SDF 并把碰撞体设为零摩擦 (见 omni_sim_description/scripts/urdf_to_sim_sdf.py)
    desc_share = get_package_share_directory('omni_sim_description')
    sdf = '/tmp/omni_robot_sim.sdf'
    subprocess.run([
        os.path.join(desc_share, '..', '..', 'lib', 'omni_sim_description', 'urdf_to_sim_sdf.py'),
        os.path.join(desc_share, 'urdf', 'robot.urdf.xacro'), sdf,
        'lidar_hz:=' + LaunchConfiguration('lidar_hz').perform(context)], check=True)
    return [Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-file', sdf, '-name', 'omni_robot',
                            '-x', LaunchConfiguration('x').perform(context),
                            '-y', LaunchConfiguration('y').perform(context),
                            '-z', '0.03', '-Y', LaunchConfiguration('yaw').perform(context)])]


def merger_node(context):
    # 每个子帧抽样点数随 lidar_hz 换算, 每台 100k 点/秒 (Mid360 的一半).
    # 合并帧要 < 512KB (Fast DDS 共享内存段): 200k 点/秒时一帧 0.8MB, 被分片后 best-effort
    # 订阅会丢约 30% 帧, Point-LIO 断帧 -> odom->base_link 断档 -> RViz "Could not transform"
    hz = float(LaunchConfiguration('lidar_hz').perform(context))
    return [Node(package='hospital_sim', executable='livox_sim_merger', output='screen',
                 parameters=[os.path.join(get_package_share_directory('hospital_sim'), 'config',
                                          'livox_sim_merger.yaml'),
                             {'use_sim_time': True, 'points_per_subscan': int(100000 / hz)}])]


def generate_launch_description():
    sim_share = get_package_share_directory('hospital_sim')
    desc_share = get_package_share_directory('omni_sim_description')
    gz_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'robot.urdf.xacro'),
                 ' lidar_hz:=', LaunchConfiguration('lidar_hz')]),
        value_type=str)

    gz_args = PythonExpression([
        "'-r -v 2 ", world, "' + ('' if '", gui, "' == 'true' else ' -s')"])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(sim_share, 'worlds', 'hospital.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='10.0'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        DeclareLaunchArgument('lidar_hz', default_value='50',
                              description='gz 雷达子帧频率 (合并后固定 10Hz). 不建议降低: '
                                          '20Hz 时 gz CPU 约 -30%, 但 Point-LIO 在长走廊跳了 0.8m '
                                          '(50Hz 同路段 <0.1m)'),

        # 医院模型(AWS) + 机器人 mesh (package:// 解析需要 share 的上一级)
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        OpaqueFunction(function=spawn_robot),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(sim_share, 'config', 'gz_bridge.yaml'),
                          'use_sim_time': True}]),

        OpaqueFunction(function=merger_node),

        Node(package='hospital_sim', executable='omni_wheel_kinematics.py',
             parameters=[{'use_sim_time': True}]),
    ])
