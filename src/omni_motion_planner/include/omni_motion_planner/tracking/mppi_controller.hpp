#pragma once

#include <eigen3/Eigen/Core>
#include <omni_motion_planner/search/path_planner.hpp>
#include <omni_motion_planner/tracking/controller.hpp>

#include <optional>
#include <random>
#include <vector>

namespace omni_planner
{

// MPPI 控制器参数(独立结构体,避免 NSDMI 类内解析问题)
struct MppiParams {
    // ===== 预测 horizon =====
    int    horizon{20};            // 预测步数
    double dt{0.05};               // 离散步长(秒) → 1s 预测窗口

    // ===== 采样配置(和 MPC 最大区别在这里) =====
    int    num_samples{1024};      // 每拍采样多少条候选控制序列
    double sigma_v{0.2};           // 线速度采样噪声标准差(m/s)
    double sigma_omega{0.4};       // 角速度采样噪声标准差(rad/s)
    double lambda{0.3};            // softmax 温度:越小越贪心(只挑最好的),
                                   //              越大越平均(更平滑)

    // ===== 机器人动力学约束(硬限幅,拒收越界样本) =====
    double max_vel{1.5};           // 最大线速度 m/s
    double max_omega{1.0};         // 最大角速度 rad/s
    double max_acc{2.0};           // 最大线加速度 m/s² (软约束,通过 w_delta 实现)
    double max_alpha{2.0};         // 最大角加速度 rad/s²

    // ===== 代价权重(和 MPC 一致,便于横向对比) =====
    double w_pos{1.0};             // 跟踪位置误差
    double w_theta{0.3};           // 跟踪朝向误差
    double w_vel{0.1};             // 跟踪参考速度
    double w_ctrl_vel{0.01};       // 控制量(线速度)幅度
    double w_ctrl_omega{0.01};     // 控制量(角速度)幅度
    double w_delta_vel{0.5};       // 控制量增量(线速度)平滑性
    double w_delta_omega{0.5};     // 控制量增量(角速度)平滑性

    // ===== 终端处理 =====
    bool   allow_backward{false};  // 是否允许倒车

    // ===== 随机数种子(复现实验用,默认随机) =====
    unsigned seed{0};              // 0 表示用 random_device
};

// MPPI (Model Predictive Path Integral) 控制器。
//
// 和 MPC 的区别(算法核心):
//   MPC  = 求导型:对代价函数 J 求梯度,梯度下降找最优控制
//   MPPI = 采样型:随机撒 N 条候选控制,前向仿真算代价,加权平均
//
// 优点(相对 MPC):
//   - 不需要梯度(非线性代价/非凸避障都能直接用)
//   - 实现简单(核心 ~50 行,不用推雅可比)
//   - 天然处理非凸(不会卡局部极小)
//   - GPU/OpenMP 友好(并行采样)
//
// 缺点:
//   - CPU 单核慢(必须并行才实用)
//   - 输出有随机抖动(需要平滑)
//   - 温度 lambda 对性能敏感
//
// 适合:复杂代价函数 / 非凸场景 / 算力充足。
// Nav2 已经把 MPPI 作为默认 controller。
class MPPI : public Controller
{
public:
    using Controller::Command;

    explicit MPPI(MppiParams params = MppiParams{})
        : params_(params),
          rng_(params.seed != 0 ? params.seed
                                 : std::random_device{}()),
          noise_v_(0.0, params.sigma_v),
          noise_w_(0.0, params.sigma_omega) {}

    std::optional<Command> computeCommand(
        const Eigen::Vector3d &robot_state,
        const std::vector<TrajectoryPoint> &ref_path) override;

    const std::vector<TrajectoryPoint> &predictedTrajectory() const override
    {
        return predicted_;
    }

    void reset() override
    {
        prev_cmd_ = Eigen::Vector2d::Zero();
        nominal_u_.clear();
        predicted_.clear();
    }

    const MppiParams &params() const { return params_; }
    void setParams(const MppiParams &p) { params_ = p; }

private:
    // 从参考轨迹上提取当前时刻的参考点序列(和 MPC 完全一致)
    void extractReference(
        const Eigen::Vector3d &robot_state,
        const std::vector<TrajectoryPoint> &ref_path,
        std::vector<Eigen::Vector3d> &ref_states,    // (x, y, theta)
        std::vector<double>    &ref_vel) const;       // 参考线速度

    // 给定初始状态 + 控制序列,前向积分得到预测轨迹
    void predictTrajectory(
        const Eigen::Vector3d &init,
        const Eigen::Vector2d *controls,
        Eigen::Vector3d *states) const;

    // unicycle 模型单步前向积分(中点法,和 MPC 完全一致)
    Eigen::Vector3d stepModel(const Eigen::Vector3d &s,
                              double v, double omega) const;

    // 单步代价(只算一步,采样时循环调用累加)
    double stepCost(const Eigen::Vector3d &state,
                    const Eigen::Vector3d &ref_state,
                    double ref_vel,
                    const Eigen::Vector2d &u,
                    const Eigen::Vector2d &prev_u) const;

    // 单条样本轨迹的总代价(撞墙/越界返回 +∞)
    double rolloutCost(
        const Eigen::Vector3d &init,
        const std::vector<Eigen::Vector2d> &controls,
        const std::vector<Eigen::Vector3d> &ref_states,
        const std::vector<double> &ref_vel) const;

    MppiParams params_;

    // 上一周期的控制量(用于平滑性约束)
    Eigen::Vector2d prev_cmd_{0.0, 0.0};

    // 标称控制序列(每拍从上一拍平移初始化,warm start)
    std::vector<Eigen::Vector2d> nominal_u_;

    // 随机数生成器
    std::mt19937 rng_;
    std::normal_distribution<double> noise_v_;
    std::normal_distribution<double> noise_w_;

    std::vector<TrajectoryPoint> predicted_;
};

}  // namespace omni_planner
