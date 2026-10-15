#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/search/path_planner.hpp>
#include <omni_motion_planner/tracking/controller.hpp>

#include <optional>
#include <vector>

namespace omni_planner
{

// MPC 控制器参数(独立结构体,避免 NSDMI 类内解析问题)
struct MpcParams {
    // ===== 预测 horizon =====
    int    horizon{20};             // 预测步数
    double dt{0.05};                // 离散步长(秒) → 1s 预测窗口

    // ===== 机器人动力学约束(硬限幅) =====
    double max_vel{1.5};            // 最大线速度 m/s
    double max_omega{1.0};          // 最大角速度 rad/s
    double max_acc{2.0};            // 最大线加速度 m/s²
    double max_alpha{2.0};          // 最大角加速度 rad/s²

    // ===== 代价权重 =====
    double w_pos{1.0};              // 跟踪位置误差
    double w_theta{0.3};            // 跟踪朝向误差
    double w_vel{0.1};              // 跟踪参考速度
    double w_ctrl_vel{0.01};        // 控制量(线速度)平滑性
    double w_ctrl_omega{0.01};      // 控制量(角速度)平滑性
    double w_delta_vel{0.5};        // 控制量增量(线速度)平滑性
    double w_delta_omega{0.5};      // 控制量增量(角速度)平滑性

    // ===== 求解器 =====
    int    max_iters{10};           // 迭代次数(SQP / 投影梯度)
    double learning_rate{0.1};      // 步长
    double converge_threshold{1e-4};

    // ===== 终端处理 =====
    bool   allow_backward{false};   // 是否允许倒车
};

// MPC 控制器:跟踪 MINCO / Kino A* 输出的轨迹。
//
// 工作原理:
//   1. 用 unicycle 模型从当前状态预测 horizon 步
//   2. 优化 (v_seq, ω_seq) 让预测轨迹贴近参考轨迹
//   3. 输出第一步的 (v, ω) 作为 cmd_vel
//   4. 下个控制周期重新求解(rolling horizon)
//
// 不依赖外部 QP 求解器,用投影梯度法 + 多次迭代,
// 适合学习理解。生产建议换 osqp / acado。
class MPC : public Controller
{
public:
    using Controller::Command;

    explicit MPC(MpcParams params = MpcParams{})
        : params_(params) {}

    // 计算 cmd_vel。
    //   robot_state: (x, y, theta)
    //   ref_path   : 参考轨迹(MINCO 输出)
    // 返回 std::nullopt 表示失败(参考轨迹空 / 超界)
    std::optional<Command> computeCommand(
        const Eigen::Vector3d &robot_state,
        const std::vector<TrajectoryPoint> &ref_path) override;

    // 调试用:本次求解的预测轨迹
    const std::vector<TrajectoryPoint> &predictedTrajectory() const override
    {
        return predicted_;
    }

    void reset() override
    {
        prev_cmd_ = Eigen::Vector2d::Zero();
        predicted_.clear();
        loss_history_.clear();
    }

    // 调试用:本次迭代 loss 历史
    const std::vector<double> &lossHistory() const { return loss_history_; }

    const MpcParams &params() const { return params_; }
    void setParams(const MpcParams &p) { params_ = p; }

private:
    // 从参考轨迹上提取当前时刻的参考点序列
    // (从机器人当前位置投影到 ref_path,取后 horizon 步)
    void extractReference(
        const Eigen::Vector3d &robot_state,
        const std::vector<TrajectoryPoint> &ref_path,
        std::vector<Eigen::Vector3d> &ref_states,    // (x, y, theta)
        std::vector<double>    &ref_vel) const;       // 参考线速度

    // 给定初始状态 + 控制序列,前向积分得到预测轨迹
    void predictTrajectory(
        const Eigen::Vector3d &init,
        const Eigen::Vector2d *controls,              // (v, ω) × horizon
        Eigen::Vector3d *states) const;               // (x, y, theta) × (horizon+1)

    // 计算代价 + 对 controls 的梯度(投影梯度法用)
    double computeCostAndGradient(
        const Eigen::Vector3d &init,
        const std::vector<Eigen::Vector3d> &ref_states,
        const std::vector<double>    &ref_vel,
        const Eigen::Vector2d *controls,
        Eigen::Vector2d *grad) const;

    // 把控制量投影到可行域(速度/加速度限幅)
    void projectToFeasible(
        Eigen::Vector2d *controls,
        const Eigen::Vector2d &prev_cmd) const;

    // unicycle 模型单步前向积分
    Eigen::Vector3d stepModel(const Eigen::Vector3d &s,
                              double v, double omega) const;

    MpcParams params_;

    // 上一周期的控制量(用于平滑性约束)
    Eigen::Vector2d prev_cmd_{0.0, 0.0};

    std::vector<TrajectoryPoint> predicted_;
    std::vector<double>          loss_history_;
};

}  // namespace omni_planner
