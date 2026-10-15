#!/usr/bin/env bash
# 停掉本工作空间启动的仿真/建图/导航进程
pats="gz sim|ros2 launch|ros2 run|pointlio_mapping|lio_odom_adapter|livox_sim_merger|parameter_bridge|robot_state_publisher|omni_wheel_kinematics|static_transform_publisher|gicp_relocalization|auto_tour|gt_evaluator|recorder\.py|nav2_|_server|lifecycle_manager|velocity_smoother|collision_monitor|bt_navigator|rviz2"
pids=$(pgrep -f "$pats" | grep -vw $$)
[ -n "$pids" ] && kill $pids 2>/dev/null; sleep 2
pids=$(pgrep -f "$pats" | grep -vw $$)
[ -n "$pids" ] && kill -9 $pids 2>/dev/null
echo "stopped"
