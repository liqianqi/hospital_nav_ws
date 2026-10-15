#pragma once

#include <eigen3/Eigen/Core>

#include <vector>

namespace omni_planner
{

// 轨迹中的一个采样点:位置 + 朝向 + 时间 + 累积代价
// 普通 2D A* 的朝向/时间字段填 0,只使用 xy。
struct TrajectoryPoint {
    Eigen::Vector2d pos{0.0, 0.0};
    double theta{0.0};
    double t{0.0};
    double cost{0.0};
};

// 所有路径规划器的抽象基类。
// 2D A*、Kino-dynamic A*、Hybrid A* 等都继承它,
// 让上层模块(轨迹跟踪、可视化、ROS 节点)能用统一接口调用。
class PathPlanner
{
public:
    virtual ~PathPlanner() = default;

    // 不同的规划器接受的 start/goal 维度不同(2D/3D/带朝向),
    // 但回溯后的"路径点"统一用 TrajectoryPoint 表达。
    virtual bool plan(const Eigen::Vector2d &start,
                      const Eigen::Vector2d &goal) = 0;

    virtual void reset() { path_.clear(); }

    const std::vector<TrajectoryPoint> &path() const { return path_; }

protected:
    std::vector<TrajectoryPoint> path_;
};

}  // namespace omni_planner
