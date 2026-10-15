# hospital_nav_ws — 医院场景全向底盘仿真 (Gazebo Harmonic + Point-LIO + Nav2)

ROS 2 Jazzy / gz sim 8 (Harmonic) / Ubuntu 24.04

- 场景: AWS RoboMaker Hospital (转成 gz sim 格式, 去天花板)
- 车体: nav_ws 的 POKE 底盘 (45° X 型 6 寸全向轮), CAD 网格
- 传感器: 双 Livox Mid360 对角安装, 外倾 25° (与 nav_ws CAD 一致) + 前右 Mid360 内置 IMU (200Hz)
- 里程计/建图: Point-LIO (LihanChen2004/Point-LIO, ROS2)
- 全局定位: nav_ws 的 gicp_relocalization (多层 FFT 粗搜 + small_gicp 精配/跟踪)
- 导航: Nav2 (SmacPlanner2D + MPPI Omni + velocity_smoother + collision_monitor)

## 目录

```
src/
├── hospital_sim/          gz world、sim launch、gz<->ROS 桥
│   ├── livox_sim_merger       两台雷达 50Hz 子帧 -> 10Hz Livox CustomMsg (逐点时间戳, 合并到主雷达系)
│   ├── omni_wheel_kinematics  /cmd_vel 加速度限幅 -> gz; X 型全向轮逆解 -> /joint_states /wheel_speeds
│   ├── auto_tour.py           建图用自动巡游 (真值闭环, 只采数据)
│   └── convert_hospital_world.py  Classic world -> gz SDF
├── omni_sim_description/  URDF/xacro + 网格; urdf_to_sim_sdf.py (生成 SDF 并把碰撞体设为零摩擦)
├── omni_nav_bringup/      Point-LIO/Nav2/重定位 配置与 launch
│   ├── lio_odom_adapter       Point-LIO -> odom->base_link TF、/odom、/lio/cloud_registered、建图存 PCD
│   ├── pcd_to_gridmap.py      PCD -> Nav2 pgm/yaml
│   └── gt_evaluator.py        与 gz 真值对比定位误差
├── point_lio/             Point-LIO (改为 C++17 以兼容 Jazzy)
├── gicp_relocalization/   来自 nav_ws
├── omni_motion_planner/   来自 192.168.110.110:~/AI/nav_ws (kino A* / MINCO / MPPI 核心库, 尚无 ROS 节点)
├── livox_ros_driver2/     仅消息 (CustomMsg), 实车换回官方驱动
└── third_party/aws-robomaker-hospital-world  (COLCON_IGNORE, 模型由 hospital_sim 安装)
```

TF: `map ─(gicp_relocalization | static)→ odom ─(lio_odom_adapter)→ base_link ─(URDF)→ livox_frame → imu_link`
（odom = Point-LIO 启动时刻的 base_link; map = 建图起点的 base_link）

## 编译 / 环境

本机 PATH 里的 python3 是 `~/.local/bin/python3.10`, ROS Jazzy 需要系统 3.12, 所以工作空间带一个
基于系统 python3.12 的 venv (`--system-site-packages`, rclpy/colcon/catkin_pkg 直接可用)。

```bash
cd ~/hospital_nav_ws
/usr/bin/python3 -m venv --system-site-packages .venv    # 只需一次 (已建好)
source run_env.sh               # 每个终端都要: 激活 .venv + source ROS/工作空间 + DDS 只在本机发现
python -m colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
./stop_all.sh                   # 停掉所有仿真/导航进程
```

## 1. 建图

```bash
ros2 launch hospital_sim sim.launch.py                    # gz GUI; gui:=false 无界面
ros2 launch omni_nav_bringup mapping.launch.py            # Point-LIO + RViz
ros2 run hospital_sim auto_tour.py --ros-args -p use_sim_time:=true   # 自动绕一圈 (~3 分钟)
#   或手动: ros2 run teleop_twist_keyboard teleop_twist_keyboard  (按住 shift 横移)
ros2 service call /save_map std_srvs/srv/Trigger          # -> src/omni_nav_bringup/maps/hospital.pcd
ros2 run omni_nav_bringup pcd_to_gridmap.py src/omni_nav_bringup/maps/hospital.pcd src/omni_nav_bringup/maps/hospital
```

仓库里已带一份建好的 `maps/hospital.{pcd,pgm,yaml}`。

## 2. 定位 + 导航

```bash
ros2 launch hospital_sim sim.launch.py x:=-4.3 y:=-3.0 yaw:=1.5708     # 任意出生点
ros2 launch omni_nav_bringup navigation.launch.py                        # FFT+GICP 自动全局定位
#   localization:=static [init_x:= init_y:= init_yaw:=]   跳过重定位, 用静态 map->odom
ros2 run omni_nav_bringup gt_evaluator.py --ros-args -p use_sim_time:=true -p align:=spawn   # 可选: 真值误差
```

RViz 里用 "Nav2 Goal" 发目标, "2D Pose Estimate" 可以手动给重定位初值。

## 已验证结果 (无界面, RTF 1.0)

| 项目 | 结果 |
|---|---|
| 仿真 | RTF 1.0; 合并雷达严格 10Hz, 40k 点/帧; IMU 200Hz |
| Point-LIO 建图 (约 110m 闭环, 0.6m/s, 转弯 ≤54°/s) | 最大误差 0.115 m, 回到起点 0.009 m, yaw < 0.5° |
| Nav2 (static 定位, 30m 穿走廊到右下走廊) | 成功, 74 s, 0 次 recovery, 终点定位误差 0.12 m |
| 全局重定位 (出生点离建图起点 ~15m, 转了 180°) | 自动成功, 跟踪误差 0.024 m / 0.01°; FFT 搜索约 8 s (16 线程) |

## 仿真里踩过的坑 (改模型前先看)

1. **碰撞体必须零摩擦**: 底盘由 VelocityControl 直接给速度; 若轮子/车体与地面有摩擦, IMU 会测到
   ≈1g 的反向比力, 但车速被强行保持 → LIO 发散。URDF 的 `<mu1>/<mu2>` 对这些碰撞体不生效,
   所以 sim.launch 用 `urdf_to_sim_sdf.py` 在 SDF 里直接写零摩擦。
2. **速度不能阶跃**: omni_wheel_kinematics 对 /cmd_vel 做了加速度限幅 (2, 2, 3), 否则 IMU 尖峰。
3. **别贴墙走**: 离墙 < ~0.4m 时墙上的点大多落进 blind(0.3m)/自遮挡框, Point-LIO 会跳。
4. **走廊里的家具**: y=-8.5 横廊有拖把车 (1.08,-8.59), 右走廊 y>-7 有推床, 撞上会把车弹起来 → LIO 直接飞。
5. **别让慢订阅者挂在 40k 点的话题上**: Python 订阅 /livox/lidar 会拖住 Fast DDS, Nav2 生命周期服务超时。

## 3. 自研规划: Kino A* + MINCO + MPPI/MPC (不起 Nav2)

```bash
ros2 launch hospital_sim sim.launch.py
ros2 launch omni_motion_planner planner.launch.py                  # MPPI; controller:=mpc 换 MPC
#   localization:=static 等参数同 navigation.launch.py (内部 include 它, nav2:=false)
```

RViz 用 "2D Goal Pose" 发 `/goal_pose`。黄 `/kino_path`、绿 `/plan` (MINCO)、紫 `/local_plan` (控制器预测)。
参数在 `omni_motion_planner/config/planner.yaml`。只用静态地图做 ESDF, 没有动态障碍和 collision_monitor。

## 接自研规划/控制 (omni_motion_planner)

- 全局: 把 kino A* (+MINCO) 写成 `nav2_core::GlobalPlanner` 插件, 替换 `planner_server.GridBased`;
  或独立节点订阅 `/goal_pose` + `/map` (或 global_costmap)。
- 局部: 写成 `nav2_core::Controller` 插件替换 `controller_server.FollowPath`;
  或关掉 controller_server, 自己的 MPC/MPPI 直接发 `/cmd_vel_nav` (仍经 velocity_smoother + collision_monitor)。
- 注意: `omni_planner::Controller::Command` 目前只有 (v, ω), 全向底盘需要加 vy。
- 可用输入: `/odom` (odom 系, 带 body 速度), TF map->base_link, `/livox/points` (主雷达系合并点云),
  `/local_costmap/costmap`, `/global_costmap/costmap`。
