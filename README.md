# hospital_nav_ws — 医院场景全向底盘仿真

医院场景里的 45° X 型全向底盘 + 双 Livox Mid360 仿真, 跑通 建图 → 全局重定位 → 导航 (Nav2 或自研规划)。

## 环境

- Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic (gz sim 8)
- 依赖: Nav2、ros_gz、PCL、Eigen、small_gicp; 大文件用 Git LFS

```bash
git clone --recursive git@github.com:liqianqi/hospital_nav_ws.git ~/hospital_nav_ws   # 先装 git-lfs
cd ~/hospital_nav_ws
/usr/bin/python3 -m venv --system-site-packages .venv   # 只需一次: 让 python3 指向系统 3.12
source run_env.sh                                        # 每个终端都要
python -m colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`./stop_all.sh` 停掉所有仿真/定位/导航进程。

## 启动流程

先起仿真 (每种用法都要):

```bash
ros2 launch hospital_sim sim.launch.py            # gui:=false 无界面; x:= y:= yaw:= 改出生点
```

| 用途 | 命令 | RViz 操作 |
|---|---|---|
| 建图 | `ros2 launch omni_nav_bringup mapping.launch.py`<br>`ros2 run hospital_sim auto_tour.py --ros-args -p use_sim_time:=true`<br>`ros2 service call /save_map std_srvs/srv/Trigger`<br>`ros2 run omni_nav_bringup pcd_to_gridmap.py <pcd> <输出前缀>` | — |
| Nav2 导航 | `ros2 launch omni_nav_bringup navigation.launch.py` | Nav2 Goal |
| 自研规划 | `ros2 launch omni_motion_planner planner.launch.py` (`controller:=mpc` 换 MPC) | 2D Goal Pose |

- 仓库已带建好的地图 `omni_nav_bringup/maps/hospital.{pcd,pgm,yaml}`。
- 定位默认自动全局重定位, 启动后约 10 s 定位成功再发目标; `localization:=static` 跳过重定位
  (从建图起点出发, 或给 `init_x/init_y/init_yaw`)。RViz 的 2D Pose Estimate 可手动给初值。

## 算法

| 模块 | 算法 | 代码 |
|---|---|---|
| 雷达仿真 | 两台雷达 50Hz 子帧合成 10Hz Livox 帧 (逐点时间戳, 合到主雷达系) | `hospital_sim/livox_sim_merger` |
| 里程计 / 建图 | Point-LIO (逐点 IEKF, 主雷达 + 内置 IMU) | `point_lio`, `omni_nav_bringup/lio_odom_adapter` |
| 全局重定位 | 粗: 多高度层栅格 + FFT 互相关 (每 2° 一个 yaw); 精: small_gicp 验证候选; 跟踪: GICP, 连续失败自动重新全局搜索 | `gicp_relocalization` |
| Nav2 导航 | SmacPlanner2D + MPPI (Omni) + velocity_smoother + collision_monitor | `omni_nav_bringup/config/nav2_params.yaml` |
| 自研规划 | Kino A* (ESDF 碰撞 + Dijkstra 启发) → MINCO 风格平滑 (jerk + ESDF 避障) → MPPI / MPC 跟踪 | `omni_motion_planner` |

TF: `map ─(gicp_relocalization)→ odom ─(lio_odom_adapter)→ base_link ─(URDF)→ livox_frame`

自研规划的限制: 控制器是 unicycle 模型 (只发 vx, wz, 没用横移); ESDF 只来自静态地图, 不避动态障碍。

## 仿真注意事项

1. 碰撞体必须零摩擦 (`urdf_to_sim_sdf.py` 已处理), 否则 IMU 测到反向比力, LIO 发散。
2. `/cmd_vel` 不能阶跃, `omni_wheel_kinematics` 做了加速度限幅 (2, 2, 3)。
3. 别贴墙走 (< 0.4m) 也别撞走廊里的拖把车/推床, Point-LIO 会跳。
4. 合并点云每帧要 < 512KB (Fast DDS 共享内存段), 否则 best-effort 订阅丢帧, TF 断档; 现在每台雷达 100k 点/秒。
5. 别用 Python 订阅 `/livox/lidar` 这类大点云话题, 会拖慢 DDS。
