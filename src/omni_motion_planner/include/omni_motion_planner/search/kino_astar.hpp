#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/map/esdf_map.hpp>
#include <omni_motion_planner/search/path_planner.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace omni_planner
{

// Kino A* 参数(独立结构体)
struct KinoParams {
    double robot_radius{0.3};         // 碰撞安全距离(米)
    double max_vel{2.5};              // 最大线速度(m/s)
    double max_acc{1.0};              // 最大加速度(m/s²)
    double max_omega{1.0};            // 最大角速度(rad/s)
    double dt{0.5};                   // 单步前向积分时间(s)
    int    n_theta{36};               // 朝向离散数
    int    n_acc{3};                  // 加速度采样数
    int    n_omega{3};                // 角速度采样数
    double goal_tol_xy{0.2};          // 终点位置容差(米)
    double goal_tol_theta{0.2};       // 终点朝向容差(rad)
    double lambda{0.5};               // g 中运动学代价 vs 时间的权重
    double lambda_obs{2.0};           // 障碍净空惩罚权重(贴墙时为正)
    double obs_decay{0.2};            // 指数衰减尺度:距离每增加 obs_decay,惩罚衰减 e 倍
    double xy_resolution{0.0};        // 状态去重的 xy 格大小(米), <=0 用 ESDF 分辨率
    double h_weight{1.0};             // 启发式权重(h 单位是米, g 约为秒, >1 会更贪心更快)
    std::size_t max_nodes{200000};    // 节点上限,防爆内存
};

// Kino-dynamic A* for a unicycle/differential-drive robot.
//
// 状态空间:(x, y, theta, v),连续 x/y/v,theta 离散到 N_THETA 个方向。
// 控制输入:(a, omega) 离散采样若干组,前向积分 motion model 得到下一状态。
// 碰撞检查:O(1) ESDF 距离查询(机器人视为半径 r 的圆盘)。
// 启发式  :max(2D ESDF Dijkstra 距离, 直线距离)
//
// 继承 PathPlanner,与 AStar 共用 plan()/path() 接口。
class KinoDynamicAStar : public PathPlanner
{
public:
    explicit KinoDynamicAStar(const ESDFMap *esdf)
        : esdf_(esdf), params_() {}

    KinoDynamicAStar(const ESDFMap *esdf, const KinoParams &params)
        : esdf_(esdf), params_(params) {}

    // PathPlanner 接口:start/goal 是世界坐标(x,y),theta 默认 0。
    // 如果想指定朝向,用 planWithHeading。
    bool plan(const Eigen::Vector2d &start,
              const Eigen::Vector2d &goal) override
    {
        return planWithHeading(start, 0.0, goal, 0.0);
    }

    // 完整版:start_theta/goal_theta 为起末朝向(rad)。
    bool planWithHeading(const Eigen::Vector2d &start, double start_theta,
                         const Eigen::Vector2d &goal, double goal_theta);

    void reset() override
    {
        PathPlanner::reset();
        lookup_.clear();
        generation_ = 0;
    }

    // 给可视化/调试用:本次 search 扩展过的所有状态
    const std::vector<Eigen::Vector4d> &expandedStates() const { return expanded_; }

    const KinoParams &params() const { return params_; }
    void setParams(const KinoParams &p) { params_ = p; }

private:
    // 离散状态 key:(x_g, y_g, theta_g, v_g)
    struct StateKey {
        int x;
        int y;
        int theta;
        int v;
        bool operator==(const StateKey &o) const noexcept
        {
            return x == o.x && y == o.y && theta == o.theta && v == o.v;
        }
    };
    struct StateKeyHash {
        std::size_t operator()(const StateKey &k) const noexcept
        {
            return std::hash<long long>()(
                (static_cast<long long>(k.x) * 73856093LL) ^
                (static_cast<long long>(k.y) * 19349663LL) ^
                (static_cast<long long>(k.theta) * 83492791LL) ^
                (static_cast<long long>(k.v)));
        }
    };

    // 节点:连续状态 + 累积代价 g + 启发式 h + 父指针
    struct Node {
        Eigen::Vector4d state;   // (x, y, theta, v)
        double g{std::numeric_limits<double>::infinity()};
        double h{0.0};
        int   parent{-1};
    };

    using QEntry = std::pair<double, int>;

    StateKey discretize(const Eigen::Vector4d &s) const;

    Eigen::Vector4d integrate(const Eigen::Vector4d &s,
                              double a, double omega, double dt) const;

    bool inCollision(const Eigen::Vector4d &s0,
                     const Eigen::Vector4d &s1) const;

    void buildHeuristicField(const Eigen::Vector2d &goal);
    double heuristic(const Eigen::Vector2d &pos) const;

    void backtrack(int goal_node_idx, double goal_theta);

    const ESDFMap *esdf_;
    KinoParams     params_;

    std::vector<Node> node_pool_;
    std::unordered_map<StateKey, int, StateKeyHash> lookup_;

    std::vector<double> h_field_;
    int   h_w_{0}, h_h_{0};
    double h_res_{0.0};
    Eigen::Vector2d h_origin_{0.0, 0.0};

    std::vector<Eigen::Vector4d> expanded_;
    std::uint32_t generation_{0};
};

}  // namespace omni_planner
