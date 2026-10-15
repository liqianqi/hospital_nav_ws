#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/search/path_planner.hpp>

#include <optional>
#include <vector>

namespace omni_planner
{

// 轨迹跟踪控制器抽象基类。
// 所有局部控制器(MPC / MPPI / DWA / ...)都继承它,
// 让上层模块用统一接口调用,便于运行时切换。
//
// 设计原则:
//   - 输入: 当前位姿 + 参考轨迹(MINCO/Kino A* 输出)
//   - 输出: 一个 Command(v, ω)直接喂底盘
//   - 频率: 20~50Hz,rolling horizon
class Controller
{
public:
    // 控制输出:线速度 + 角速度,直接喂底盘 cmd_vel
    struct Command {
        double v{0.0};
        double omega{0.0};
    };

    virtual ~Controller() = default;

    // 计算 cmd_vel。
    //   robot_state: (x, y, theta)
    //   ref_path   : 参考轨迹(MINCO 输出)
    // 返回 std::nullopt 表示失败(参考轨迹空 / 超界)
    virtual std::optional<Command> computeCommand(
        const Eigen::Vector3d &robot_state,
        const std::vector<TrajectoryPoint> &ref_path) = 0;

    // 调试用:本次求解的预测轨迹(可视化用)
    virtual const std::vector<TrajectoryPoint> &predictedTrajectory() const = 0;

    // 重置内部状态(切换参考轨迹 / 紧急停车时调用)
    virtual void reset() {}
};

}  // namespace omni_planner
