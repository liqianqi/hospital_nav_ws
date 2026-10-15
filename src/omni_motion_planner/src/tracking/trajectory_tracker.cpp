#include "omni_motion_planner/tracking/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>

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
std::optional<MPC::Command> MPC::computeCommand(
    const Eigen::Vector3d &robot_state,
    const std::vector<TrajectoryPoint> &ref_path)
{
    if (ref_path.empty()) return std::nullopt;

    // ===== 1. 提取参考点序列(从机器人当前位置往后看 horizon 步) =====
    std::vector<Eigen::Vector3d> ref_states(params_.horizon + 1);
    std::vector<double>          ref_vel(params_.horizon + 1, 0.0);
    extractReference(robot_state, ref_path, ref_states, ref_vel);

    // ===== 2. 初始化控制序列(上一周期的 cmd 重复) =====
    std::vector<Eigen::Vector2d> controls(params_.horizon,
                                          Eigen::Vector2d(prev_cmd_.x(), prev_cmd_.y()));

    // ===== 3. 投影梯度下降 =====
    loss_history_.clear();
    double prev_loss = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < params_.max_iters; ++iter) {
        std::vector<Eigen::Vector2d> grad(params_.horizon, Eigen::Vector2d::Zero());
        double loss = computeCostAndGradient(robot_state, ref_states, ref_vel,
                                             controls.data(), grad.data());

        loss_history_.push_back(loss);
        if (iter > 0 && std::abs(prev_loss - loss) < params_.converge_threshold) break;
        prev_loss = loss;

        // 梯度下降步
        double lr = params_.learning_rate;
        for (int k = 0; k < params_.horizon; ++k) {
            controls[k] -= lr * grad[k];
        }

        // 投影到可行域
        projectToFeasible(controls.data(), prev_cmd_);
    }

    // ===== 4. 取第一步作为 cmd_vel =====
    Command cmd;
    cmd.v     = controls[0].x();
    cmd.omega = controls[0].y();
    prev_cmd_ = controls[0];

    // ===== 5. 记录预测轨迹(可视化用) =====
    predicted_.clear();
    std::vector<Eigen::Vector3d> states(params_.horizon + 1);
    predictTrajectory(robot_state, controls.data(), states.data());
    for (int k = 0; k <= params_.horizon; ++k) {
        TrajectoryPoint pt;
        pt.pos   = states[k].head<2>();
        pt.theta = states[k].z();
        pt.t     = k * params_.dt;
        pt.cost  = 0.0;
        predicted_.push_back(pt);
    }

    return cmd;
}

// =============== 参考点提取 ===============
void MPC::extractReference(
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

    // 从 closest 开始按索引取 horizon+1 个参考点
    // 如果 ref_path 不够长, 把最后一个点重复
    for (int k = 0; k <= params_.horizon; ++k) {
        int idx = std::min(closest + k, static_cast<int>(ref_path.size()) - 1);
        ref_states[k] = Eigen::Vector3d(
            ref_path[idx].pos.x(),
            ref_path[idx].pos.y(),
            ref_path[idx].theta);

        // 参考速度:用相邻点的弧长 / 时间
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

// =============== 前向预测 ===============
void MPC::predictTrajectory(
    const Eigen::Vector3d &init,
    const Eigen::Vector2d *controls,
    Eigen::Vector3d *states) const
{
    states[0] = init;
    for (int k = 0; k < params_.horizon; ++k) {
        states[k + 1] = stepModel(states[k], controls[k].x(), controls[k].y());
    }
}

Eigen::Vector3d MPC::stepModel(const Eigen::Vector3d &s,
                               double v, double omega) const
{
    // unicycle 模型(中点法,比 Euler 准)
    double theta_mid = s.z() + 0.5 * omega * params_.dt;
    double x_new = s.x() + v * std::cos(theta_mid) * params_.dt;
    double y_new = s.y() + v * std::sin(theta_mid) * params_.dt;
    double t_new = wrapAngle(s.z() + omega * params_.dt);
    return Eigen::Vector3d(x_new, y_new, t_new);
}

// =============== 代价 + 解析梯度 ===============
// 代价:
//   J = Σ_k  w_pos   · |pos_err|²          (位置跟踪)
//       +    w_theta · |angle_err|²         (朝向跟踪)
//       +    w_vel   · |v - v_ref|²         (速度跟踪)
//       +    w_ctrl  · |v|² + |ω|²          (控制能量)
//       +    w_delta · |Δv|² + |Δω|²        (控制平滑)
//
// 梯度:用解析方式推导(对 v_k 和 ω_k 各一项)
// 注:这里用最简单的"Euler 法 + 数值梯度",代码 ~30 行,
//     生产 MPC 用 SQP/解析雅可比,代码量 5-10 倍但快 10-100 倍。
double MPC::computeCostAndGradient(
    const Eigen::Vector3d &init,
    const std::vector<Eigen::Vector3d> &ref_states,
    const std::vector<double>    &ref_vel,
    const Eigen::Vector2d *controls,
    Eigen::Vector2d *grad) const
{
    // 前向积分得到预测轨迹
    std::vector<Eigen::Vector3d> states(params_.horizon + 1);
    predictTrajectory(init, controls, states.data());

    double loss = 0.0;
    for (int k = 0; k < params_.horizon; ++k)
        grad[k].setZero();

    // ===== 累加各项代价 =====
    Eigen::Vector2d prev_u(prev_cmd_.x(), prev_cmd_.y());
    for (int k = 0; k < params_.horizon; ++k) {
        const Eigen::Vector2d &u_k = controls[k];

        // ---- 位置 / 朝向 / 速度跟踪误差 ----
        Eigen::Vector2d pos_err = states[k + 1].head<2>() - ref_states[k + 1].head<2>();
        double theta_err = wrapAngle(states[k + 1].z() - ref_states[k + 1].z());
        double v_err     = u_k.x() - ref_vel[k];

        loss += params_.w_pos   * pos_err.squaredNorm();
        loss += params_.w_theta * theta_err * theta_err;
        loss += params_.w_vel   * v_err * v_err;

        // ---- 控制能量 ----
        loss += params_.w_ctrl_vel   * u_k.x() * u_k.x();
        loss += params_.w_ctrl_omega * u_k.y() * u_k.y();

        // ---- 控制增量(平滑) ----
        Eigen::Vector2d du = u_k - prev_u;
        loss += params_.w_delta_vel   * du.x() * du.x();
        loss += params_.w_delta_omega * du.y() * du.y();
        prev_u = u_k;
    }

    // ===== 数值梯度(central difference,简单稳健) =====
    // 注:数值梯度慢但易懂。 生产代码应该用解析雅可比或自动微分。
    const double eps = 1e-4;
    for (int k = 0; k < params_.horizon; ++k) {
        for (int dim = 0; dim < 2; ++dim) {
            std::vector<Eigen::Vector2d> u_plus(controls, controls + params_.horizon);
            std::vector<Eigen::Vector2d> u_minus(u_plus);
            u_plus[k][dim]  += eps;
            u_minus[k][dim] -= eps;

            // 重算 plus / minus 代价
            std::vector<Eigen::Vector3d> s_p(params_.horizon + 1);
            std::vector<Eigen::Vector3d> s_m(params_.horizon + 1);
            predictTrajectory(init, u_plus.data(),  s_p.data());
            predictTrajectory(init, u_minus.data(), s_m.data());

            double L_p = 0.0, L_m = 0.0;
            Eigen::Vector2d prev_p(prev_cmd_.x(), prev_cmd_.y());
            Eigen::Vector2d prev_m(prev_cmd_.x(), prev_cmd_.y());
            for (int j = 0; j < params_.horizon; ++j) {
                auto acc = [&](const Eigen::Vector3d &s, const Eigen::Vector2d &u,
                               const Eigen::Vector2d &prev) {
                    Eigen::Vector2d pe = s.head<2>() - ref_states[j + 1].head<2>();
                    double te = wrapAngle(s.z() - ref_states[j + 1].z());
                    double ve = u.x() - ref_vel[j];
                    double L = 0.0;
                    L += params_.w_pos   * pe.squaredNorm();
                    L += params_.w_theta * te * te;
                    L += params_.w_vel   * ve * ve;
                    L += params_.w_ctrl_vel   * u.x() * u.x();
                    L += params_.w_ctrl_omega * u.y() * u.y();
                    Eigen::Vector2d du = u - prev;
                    L += params_.w_delta_vel   * du.x() * du.x();
                    L += params_.w_delta_omega * du.y() * du.y();
                    return L;
                };
                L_p += acc(s_p[j + 1], u_plus[j],  prev_p);
                L_m += acc(s_m[j + 1], u_minus[j], prev_m);
                prev_p = u_plus[j];
                prev_m = u_minus[j];
            }
            grad[k][dim] = (L_p - L_m) / (2.0 * eps);
        }
    }

    return loss;
}

// =============== 投影到可行域 ===============
void MPC::projectToFeasible(
    Eigen::Vector2d *controls,
    const Eigen::Vector2d &prev_cmd) const
{
    for (int k = 0; k < params_.horizon; ++k) {
        Eigen::Vector2d &u = controls[k];

        // 线速度限幅
        if (params_.allow_backward) {
            u.x() = std::clamp(u.x(), -params_.max_vel, params_.max_vel);
        } else {
            u.x() = std::clamp(u.x(), 0.0, params_.max_vel);
        }
        // 角速度限幅
        u.y() = std::clamp(u.y(), -params_.max_omega, params_.max_omega);

        // 加速度限幅(相对上一时刻控制量)
        double dv = u.x() - prev_cmd.x();
        double dw = u.y() - prev_cmd.y();
        double max_dv = params_.max_acc   * params_.dt;
        double max_dw = params_.max_alpha * params_.dt;
        dv = std::clamp(dv, -max_dv, max_dv);
        dw = std::clamp(dw, -max_dw, max_dw);

        // 注:这里只对 k=0 做严格的加速度限制
        //      k>0 时,加速度约束在代价里以软约束形式存在
        if (k == 0) {
            u.x() = prev_cmd.x() + dv;
            u.y() = prev_cmd.y() + dw;
        }
    }
}

}  // namespace omni_planner
