#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/map/grid_map.hpp>
#include <omni_motion_planner/search/path_planner.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace omni_planner
{

// 2D A*: grid 上的经典 8 邻域搜索。
// 输入输出仍是 grid 坐标(Vector2i),通过 PathPlanner 接口对外暴露 path_。
class AStar : public PathPlanner
{
public:
    explicit AStar(const GridMap *grid_map) : grid_map_(grid_map) {}

    // 原生 grid 接口(保留向后兼容)
    bool search(const Eigen::Vector2i &start, Eigen::Vector2i goal);

    // PathPlanner 接口:世界坐标 → grid → search → 转 TrajectoryPoint
    bool plan(const Eigen::Vector2d &start,
              const Eigen::Vector2d &goal) override;

    // Octile 距离:8 邻域下的最优启发式(admissible + consistent)
    static double heuristic(
        const Eigen::Vector2i &current,
        const Eigen::Vector2i &goal)
    {
        double dx = std::abs(current.x() - goal.x());
        double dy = std::abs(current.y() - goal.y());
        constexpr double SQRT2 = 1.4142135623730951;
        return SQRT2 * std::min(dx, dy) + std::abs(dx - dy);
    }

    // 兼容旧调用方
    const std::vector<Eigen::Vector2i> &gridPath() const { return grid_path_; }

private:
    // 每个格子的搜索状态。用 generation 标记是否属于本次 search,
    // 避免每次 search 都把整张图清零(O(W*H) → O(实际访问数))
    struct Cell {
        std::uint32_t generation{0};
        double        g{std::numeric_limits<double>::infinity()};
        double        f{std::numeric_limits<double>::infinity()};
        int           parent{-1};
    };

    bool hasLineOfSight(int x0, int y0, int x1, int y1) const;
    void simplifyPath();

    const GridMap *grid_map_;
    std::vector<Eigen::Vector2i> grid_path_;   // grid 坐标路径(原 path_)

    std::vector<Cell> cells_;
    int               cells_w_{0};
    int               cells_h_{0};
    std::uint32_t     generation_{0};
};

}  // namespace omni_planner
