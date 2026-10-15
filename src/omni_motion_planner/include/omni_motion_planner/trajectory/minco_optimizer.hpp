#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/map/esdf_map.hpp>
#include <omni_motion_planner/search/path_planner.hpp>

#include <vector>

namespace omni_planner
{

// MINCO 优化器参数(独立结构体,避免 NSDMI 在类内引发的解析顺序问题)
struct MincoParams {
    double w_smooth{1.0};        // 平滑性权重(惩罚 jerk)
    double w_collision{50.0};    // 避障权重
    double w_length{1.0};        // 路径长度正则项
    double safe_distance{0.4};   // 障碍安全距离(米,应 > robot_radius)
    double max_vel{1.5};         // 速度上限(m/s)
    double dt_sample{0.05};      // 轨迹采样间隔(秒)
    int    max_iters{200};       // 最大迭代次数
    double learning_rate{0.01};  // 初始学习率
    double converge_threshold{1e-4};  // 损失变化阈值
};

// Minimum-Control 轨迹优化器(MINCO 思想的简化实现)。
//
// 输入:Kino A* / A* 输出的离散路径点(粗轨迹)
// 输出:平滑的、避免障碍的、考虑机器人运动学的连续轨迹
//
// 优化目标:J(p) = w_smooth · ∫|jerk|² + w_collision · Σ(dist - r_safe)² + w_length · Σ|Δp|
// 优化方式:梯度下降(数值差分求 jerk,ESDF 提供 collision 梯度)
//
// 注:这是教学版实现,完整版 MINCO 用 QP + L-BFGS,代码量 5-10 倍。
//     但数学/几何直觉完全一致。
class MincoOptimizer
{
public:
    explicit MincoOptimizer(const ESDFMap *esdf)
        : esdf_(esdf), params_() {}

    MincoOptimizer(const ESDFMap *esdf, const MincoParams &params)
        : esdf_(esdf), params_(params) {}

    // 把粗轨迹(kino A* 输出)优化为平滑轨迹。
    // 返回 false 表示输入无效或优化失败。
    bool optimize(const std::vector<TrajectoryPoint> &rough_path);

    // 优化后的轨迹(轨迹点的 t 字段被重新时间参数化)
    const std::vector<TrajectoryPoint> &smoothPath() const { return smooth_path_; }

    // 调试用:每次迭代的损失历史
    const std::vector<double> &lossHistory() const { return loss_history_; }

    // 调试用:碰撞惩罚项的 ESDF 梯度场(可选,可视化用)
    const std::vector<Eigen::Vector2d> &collisionGradients() const
    {
        return collision_grads_;
    }

    const MincoParams &params() const { return params_; }
    void setParams(const MincoParams &p) { params_ = p; }

private:
    std::vector<Eigen::Vector2d> resamplePath(
        const std::vector<TrajectoryPoint> &path, int n_samples) const;

    double smoothnessLoss(const std::vector<Eigen::Vector2d> &pts,
                          std::vector<Eigen::Vector2d> &grad_out) const;

    double lengthLoss(const std::vector<Eigen::Vector2d> &pts,
                      std::vector<Eigen::Vector2d> &grad_out) const;

    double collisionLoss(const std::vector<Eigen::Vector2d> &pts,
                         std::vector<Eigen::Vector2d> &grad_out) const;

    const ESDFMap *esdf_;
    MincoParams    params_;
    std::vector<TrajectoryPoint> smooth_path_;
    std::vector<double>          loss_history_;
    std::vector<Eigen::Vector2d> collision_grads_;
};

}  // namespace omni_planner
