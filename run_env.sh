# 每个终端: source ~/hospital_nav_ws/run_env.sh   (bash/zsh 通用)
# .venv 基于系统 python3.12 (--system-site-packages), 让 python3 不再指向 ~/.local/bin/python3.10
source ~/hospital_nav_ws/.venv/bin/activate
# 仿真只在本机做 DDS 发现: 避免 40k 点/帧的点云流量跑到 Wi-Fi 上, 也避免与局域网里别的 ROS 机器串话
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
if [ -n "$ZSH_VERSION" ]; then
  source /opt/ros/jazzy/setup.zsh
  [ -f ~/hospital_nav_ws/install/setup.zsh ] && source ~/hospital_nav_ws/install/setup.zsh
else
  source /opt/ros/jazzy/setup.bash
  [ -f ~/hospital_nav_ws/install/setup.bash ] && source ~/hospital_nav_ws/install/setup.bash
fi
