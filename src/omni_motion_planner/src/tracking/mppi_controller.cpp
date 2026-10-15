#include "omni_motion_planner/tracking/mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace omni_planner
{

namespace {
constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double a)
{
    while (a >  kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}  // namespace

// =============== 主入口 ===============
std::optional<MPPI::Command> MPPI::computeCommand(
    const Eigen::Vector3d &robot_state,
    const std::vector<TrajectoryPoint> &ref_path)
{
    if (ref_path.empty()) return std::nullopt;

    // ===== 1. 提取参考点序列(从机器人当前位置往后看 horizon 步) =====
    std::vector<Eigen::Vector3d> ref_states(params_.horizon + 1);
    std::vector<double>          ref_vel(params_.horizon + 1, 0.0);
    extractReference(robot_state, ref_path, ref_states, ref_vel);

    // ===== 2. 初始化标称控制序列 (warm start: 上一拍结果左移一格) =====
    // 第一拍或重置后:nearmal_u_ 为空,用 prev_cmd_ 填充
    if (static_cast<int>(nominal_u_.size()) != params_.horizon) {
        nominal_u_.assign(params_.horizon,
                          Eigen::Vector2d(prev_cmd_.x(), prev_cmd_.y()));
    } else {
        // 左移一格(丢弃已执行的 u_0,最后一格复制上一格)
        for (int k = 0; k < params_.horizon - 1; ++k) {
            nominal_u_[k] = nominal_u_[k + 1];
        }
        // 末尾保持上一拍最后一格(任意,反正会被采样噪声覆盖)
    }

    // ===== 3. 采样 + 算代价 =====
    // noise_samples[i][k] = 第 i 条样本的第 k 步噪声
    // (采样后要保留,加权平均时需要)
    std::vector<double> costs(params_.num_samples);
    std::vector<std::vector<Eigen::Vector2d>> noise_samples(
        params_.num_samples,
        std::vector<Eigen::Vector2d>(params_.horizon));

    for (int i = 0; i < params_.num_samples; ++i) {
        // 生成带噪声的控制序列 = 标称 + 高斯噪声
        std::vector<Eigen::Vector2d> u_sampled(params_.horizon);
        for (int k = 0; k < params_.horizon; ++k) {
            double nv = noise_v_(rng_);
            double nw = noise_w_(rng_);
            noise_samples[i][k] = Eigen::Vector2d(nv, nw);
            u_sampled[k] = nominal_u_[k] + Eigen::Vector2d(nv, nw);
        }

        // 前向仿真算总代价
        costs[i] = rolloutCost(robot_state, u_sampled, ref_states, ref_vel);
    }

    // ===== 4. Softmax 把代价转成权重 =====
    // 数值稳定的 softmax: 先减最小代价
    double min_cost = *std::min_element(costs.begin(), costs.end());
    std::vector<double> weights(params_.num_samples);
    double sum_w = 0.0;
    for (int i = 0; i < params_.num_samples; ++i) {
        // 代价越低 → 权重越大;lambda 控制锐度
        // 等价信息论 MPPI:w_i = exp(-(cost_i - min) / lambda)
        weights[i] = std::exp(-(costs[i] - min_cost) / params_.lambda);
        sum_w += weights[i];
    }
    if (sum_w < 1e-12) {
        // 极端情况(所有样本都撞墙),退化到标称控制
        for (int i = 0; i < params_.num_samples; ++i) weights[i] = 1.0;
        sum_w = static_cast<double>(params_.num_samples);
    }
    for (int i = 0; i < params_.num_samples; ++i) weights[i] /= sum_w;

    // ===== 5. 加权平均 → 更新标称控制序列 =====
    // new_nominal[k] = Σ w_i · (old_nominal[k] + noise_i[k])
    for (int k = 0; k < params_.horizon; ++k) {
        Eigen::Vector2d update = Eigen::Vector2d::Zero();
        for (int i = 0; i < params_.num_samples; ++i) {
            update += weights[i] * noise_samples[i][k];
        }
        nominal_u_[k] += update;
    }

    // 约束投影(只对实际输出的第 0 步严格限速,其余靠 w_ctrl/w_delta 软约束)
    Eigen::Vector2d u0 = nominal_u_[0];
    double v_lo = params_.allow_backward ? -params_.max_vel : 0.0;
    u0.x() = std::clamp(u0.x(), v_lo, params_.max_vel);
    u0.y() = std::clamp(u0.y(), -params_.max_omega, params_.max_omega);

    // 加速度约束(只对第 0 步硬约束,因为是真发出去的)
    double max_dv = params_.max_acc   * params_.dt;
    double max_dw = params_.max_alpha * params_.dt;
    double dv = std::clamp(u0.x() - prev_cmd_.x(), -max_dv, max_dv);
    double dw = std::clamp(u0.y() - prev_cmd_.y(), -max_dw, max_dw);
    u0.x() = prev_cmd_.x() + dv;
    u0.y() = prev_cmd_.y() + dw;

    // ===== 6. 记录 prev_cmd_ 和预测轨迹(可视化用) =====
    prev_cmd_ = u0;

    predicted_.clear();
    std::vector<Eigen::Vector3d> states(params_.horizon + 1);
    // 用更新后的 nominal_u_ 做最终预测(显示给可视化)
    predictTrajectory(robot_state, nominal_u_.data(), states.data());
    for (int k = 0; k <= params_.horizon; ++k) {
        TrajectoryPoint pt;
        pt.pos   = states[k].head<2>();
        pt.theta = states[k].z();
        pt.t     = k * params_.dt;
        pt.cost  = 0.0;
        predicted_.push_back(pt);
    }

    Command cmd;
    cmd.v     = u0.x();
    cmd.omega = u0.y();
    return cmd;
}

// =============== 参考点提取(和 MPC 完全一致) ===============
void MPPI::extractReference(
    const Eigen::Vector3d &robot_state,
    const std::vector<TrajectoryPoint> &ref_path,
    std::vector<Eigen::Vector3d> &ref_states,
    std::vector<double>    &ref_vel) const
{
    // 找到 ref_path 上离机器人最近的点(投影)
    int closest = 0;
    double min_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(ref_path.size()); ++i) {
        double dx = ref_path[i].pos.x() - robot_state.x();
        double dy = ref_path[i].pos.y() - robot_state.y();
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) { min_d2 = d2; closest = i; }
    }

    for (int k = 0; k <= params_.horizon; ++k) {
        int idx = std::min(closest + k, static_cast<int>(ref_path.size()) - 1);
        ref_states[k] = Eigen::Vector3d(
            ref_path[idx].pos.x(),
            ref_path[idx].pos.y(),
            ref_path[idx].theta);

        if (idx + 1 < static_cast<int>(ref_path.size())) {
            double dx = ref_path[idx + 1].pos.x() - ref_path[idx].pos.x();
            double dy = ref_path[idx + 1].pos.y() - ref_path[idx].pos.y();
            double dt = std::max(1e-3, ref_path[idx + 1].t - ref_path[idx].t);
            ref_vel[k] = std::hypot(dx, dy) / dt;
        } else {
            ref_vel[k] = 0.0;
        }
    }

    if (!params_.allow_backward) {
        for (double &v : ref_vel) v = std::abs(v);
    }
}

// =============== 前向预测(和 MPC 完全一致) ===============
void MPPI::predictTrajectory(
    const Eigen::Vector3d &init,
    const Eigen::Vector2d *controls,
    Eigen::Vector3d *states) const
{
    states[0] = init;
    for (int k = 0; k < params_.horizon; ++k) {
        states[k + 1] = stepModel(states[k], controls[k].x(), controls[k].y());
    }
}

Eigen::Vector3d MPPI::stepModel(const Eigen::Vector3d &s,
                                 double v, double omega) const
{
    // unicycle 模型(中点法,比 Euler 准)
    double theta_mid = s.z() + 0.5 * omega * params_.dt;
    double x_new = s.x() + v * std::cos(theta_mid) * params_.dt;
    double y_new = s.y() + v * std::sin(theta_mid) * params_.dt;
    double t_new = wrapAngle(s.z() + omega * params_.dt);
    return Eigen::Vector3d(x_new, y_new, t_new);
}

// =============== 单步代价(和 MPC 一致,便于横向对比) ===============
double MPPI::stepCost(const Eigen::Vector3d &state,
                       const Eigen::Vector3d &ref_state,
                       double ref_vel,
                       const Eigen::Vector2d &u,
                       const Eigen::Vector2d &prev_u) const
{
    Eigen::Vector2d pos_err = state.head<2>() - ref_state.head<2>();
    double theta_err = wrapAngle(state.z() - ref_state.z());
    double v_err     = u.x() - ref_vel;

    double cost = 0.0;
    cost += params_.w_pos   * pos_err.squaredNorm();
    cost += params_.w_theta * theta_err * theta_err;
    cost += params_.w_vel   * v_err * v_err;
    cost += params_.w_ctrl_vel   * u.x() * u.x();
    cost += params_.w_ctrl_omega * u.y() * u.y();
    Eigen::Vector2d du = u - prev_u;
    cost += params_.w_delta_vel   * du.x() * du.x();
    cost += params_.w_delta_omega * du.y() * du.y();
    return cost;
}

// =============== 单条样本轨迹总代价 ===============
double MPPI::rolloutCost(
    const Eigen::Vector3d &init,
    const std::vector<Eigen::Vector2d> &controls,
    const std::vector<Eigen::Vector3d> &ref_states,
    const std::vector<double> &ref_vel) const
{
    Eigen::Vector3d x = init;
    Eigen::Vector2d prev_u(prev_cmd_.x(), prev_cmd_.y());
    double total = 0.0;

    for (int k = 0; k < params_.horizon; ++k) {
        const Eigen::Vector2d &u = controls[k];

        // ---- 软约束:超速度界限的样本重罚(不是直接拒收,保留梯度信息) ----
        // 用二次惩罚代替硬拒收, 避免所有样本都撞墙导致权重退化
        double v_lo = params_.allow_backward ? -params_.max_vel : 0.0;
        double v_pen = 0.0;
        if (u.x() < v_lo)         v_pen += (u.x() - v_lo) * (u.x() - v_lo);
        if (u.x() > params_.max_vel) v_pen += (u.x() - params_.max_vel) * (u.x() - params_.max_vel);
        if (u.y() < -params_.max_omega) v_pen += (u.y() + params_.max_omega) * (u.y() + params_.max_omega);
        if (u.y() >  params_.max_omega) v_pen += (u.y() - params_.max_omega) * (u.y() - params_.max_omega);
        // 限速惩罚权重(重,让越界样本几乎不会被选中)
        total += 50.0 * v_pen;

        // 前向仿真一步
        x = stepModel(x, u.x(), u.y());

        // 累积跟踪 + 能量 + 平滑代价
        total += stepCost(x, ref_states[k + 1], ref_vel[k], u, prev_u);
        prev_u = u;
    }
    return total;
}

}  // namespace omni_planner
