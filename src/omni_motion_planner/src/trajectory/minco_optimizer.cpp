#include "omni_motion_planner/trajectory/minco_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace omni_planner
{

namespace {
constexpr double kEps = 1e-9;
}  // namespace

// =============== 主入口 ===============
bool MincoOptimizer::optimize(const std::vector<TrajectoryPoint> &rough_path)
{
    if (rough_path.size() < 2) return false;

    // ===== 1. 重采样到 N 个均匀点 =====
    const int N = std::max(20, std::min<int>(200, rough_path.size() * 3));
    std::vector<Eigen::Vector2d> pts = resamplePath(rough_path, N);
    if (pts.size() < 3) return false;

    // 保留起止点(边界条件)
    Eigen::Vector2d start = pts.front();
    Eigen::Vector2d goal  = pts.back();

    loss_history_.clear();
    collision_grads_.assign(N, Eigen::Vector2d::Zero());

    // ===== 2. 梯度下降主循环 =====
    double lr = params_.learning_rate;
    double prev_loss = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < params_.max_iters; ++iter) {
        std::vector<Eigen::Vector2d> grad_smooth(N, Eigen::Vector2d::Zero());
        std::vector<Eigen::Vector2d> grad_length(N, Eigen::Vector2d::Zero());
        std::vector<Eigen::Vector2d> grad_col(N, Eigen::Vector2d::Zero());

        double L_smooth   = smoothnessLoss(pts, grad_smooth);
        double L_length   = lengthLoss(pts, grad_length);
        double L_col      = collisionLoss(pts, grad_col);

        double L_total = params_.w_smooth   * L_smooth +
                         params_.w_collision * L_col +
                         params_.w_length   * L_length;
        loss_history_.push_back(L_total);

        // 收敛判断
        if (iter > 0 && std::abs(prev_loss - L_total) < params_.converge_threshold) {
            break;
        }
        prev_loss = L_total;

        // 累加梯度(权重已乘)
        std::vector<Eigen::Vector2d> grad_total(N, Eigen::Vector2d::Zero());
        for (int i = 0; i < N; ++i) {
            grad_total[i] = params_.w_smooth   * grad_smooth[i] +
                            params_.w_length   * grad_length[i] +
                            params_.w_collision * grad_col[i];
        }

        // ===== 3. 梯度下降步 + 起止点强制约束 =====
        double max_grad = kEps;
        for (const auto &g : grad_total)
            max_grad = std::max(max_grad, g.norm());

        // 自适应学习率:梯度大时减小步长,避免振荡
        double effective_lr = lr / (1.0 + max_grad);

        for (int i = 1; i < N - 1; ++i) {   // 跳过起止点
            pts[i] -= effective_lr * grad_total[i];
        }
        pts.front() = start;
        pts.back()  = goal;

        // 记录碰撞梯度(可视化)
        collision_grads_ = grad_col;
    }

    // ===== 4. 重新时间参数化(按弧长 + max_vel) =====
    smooth_path_.clear();
    smooth_path_.reserve(pts.size());

    double cum_dist = 0.0;
    double t = 0.0;
    smooth_path_.push_back({pts[0], 0.0, t, 0.0});

    for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
        double seg = (pts[i] - pts[i - 1]).norm();
        cum_dist += seg;
        // 时间 = 距离 / max_vel(乐观估计;实际控制器可能更慢)
        t = cum_dist / params_.max_vel;
        // 朝向用前后差分估计
        Eigen::Vector2d dir = (pts[i] - pts[i - 1]);
        double theta = (dir.norm() > kEps)
                       ? std::atan2(dir.y(), dir.x()) : 0.0;
        smooth_path_.push_back({pts[i], theta, t, cum_dist});
    }
    smooth_path_[0].theta = smooth_path_[1].theta;
    return true;
}

// =============== 重采样 ===============
std::vector<Eigen::Vector2d> MincoOptimizer::resamplePath(
    const std::vector<TrajectoryPoint> &path, int n_samples) const
{
    if (path.empty()) return {};

    // 累计弧长
    std::vector<double> s(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        s[i] = s[i - 1] + (path[i].pos - path[i - 1].pos).norm();
    }
    double total = s.back();
    if (total < kEps) {
        // 起止点重合,直接返回少量点
        return {path.front().pos};
    }

    // 按等弧长插值
    std::vector<Eigen::Vector2d> out;
    out.reserve(n_samples);
    out.push_back(path.front().pos);

    size_t j = 0;
    for (int i = 1; i < n_samples - 1; ++i) {
        double target = total * static_cast<double>(i) / (n_samples - 1);
        while (j + 1 < s.size() && s[j + 1] < target) ++j;
        if (j + 1 >= s.size()) {
            out.push_back(path.back().pos);
            continue;
        }
        double denom = s[j + 1] - s[j];
        double alpha = denom > kEps ? (target - s[j]) / denom : 0.0;
        Eigen::Vector2d p = (1.0 - alpha) * path[j].pos + alpha * path[j + 1].pos;
        out.push_back(p);
    }
    out.push_back(path.back().pos);
    return out;
}

// =============== 平滑性 loss(三阶差分近似 jerk) ===============
// 对于路径点序列 p0, p1, ..., pn:
//   一阶差分 v_i = p_{i+1} - p_i
//   二阶差分 a_i = v_{i+1} - v_i
//   三阶差分 j_i = a_{i+1} - a_i ≈ jerk
// 平滑性 loss = Σ |j_i|²
//
// 梯度:
//   ∂(|j_i|²)/∂p_k 非零仅当 k ∈ {i, i+1, i+2, i+3}
//   分别为 -1, +3, -3, +1 (j = p_{i+3} - 3p_{i+2} + 3p_{i+1} - p_i)
double MincoOptimizer::smoothnessLoss(
    const std::vector<Eigen::Vector2d> &pts,
    std::vector<Eigen::Vector2d> &grad_out) const
{
    int N = pts.size();
    if (N < 4) return 0.0;

    double loss = 0.0;
    for (int i = 0; i + 3 < N; ++i) {
        Eigen::Vector2d jk = pts[i + 3] - 3.0 * pts[i + 2]
                             + 3.0 * pts[i + 1] - pts[i];
        loss += jk.squaredNorm();

        // ∂loss/∂p_i   =  2 * jk * (-1)
        // ∂loss/∂p_i+1 =  2 * jk * (+3)
        // ∂loss/∂p_i+2 =  2 * jk * (-3)
        // ∂loss/∂p_i+3 =  2 * jk * (+1)
        grad_out[i]     -= 2.0 * jk;
        grad_out[i + 1] += 6.0 * jk;
        grad_out[i + 2] -= 6.0 * jk;
        grad_out[i + 3] += 2.0 * jk;
    }
    return loss;
}

// =============== 长度正则项 ===============
// L = Σ |p_{i+1} - p_i|
// 梯度(对 p_i):单位向量差分
double MincoOptimizer::lengthLoss(
    const std::vector<Eigen::Vector2d> &pts,
    std::vector<Eigen::Vector2d> &grad_out) const
{
    int N = pts.size();
    if (N < 2) return 0.0;

    double loss = 0.0;
    for (int i = 0; i + 1 < N; ++i) {
        Eigen::Vector2d d = pts[i + 1] - pts[i];
        double len = d.norm();
        loss += len;
        if (len > kEps) {
            Eigen::Vector2d unit = d / len;
            grad_out[i]     -= unit;
            grad_out[i + 1] += unit;
        }
    }
    return loss;
}

// =============== 碰撞 loss(基于 ESDF) ===============
// 对于每个采样点 p, 设 d = ESDF(p):
//   若 d ≥ safe_distance:loss = 0(安全)
//   若 d < safe_distance:loss = (safe_distance - d)²
// 梯度 = -2 * (safe_distance - d) * ∇ESDF
// 其中 ∇ESDF 是指向"远离障碍"方向的单位向量。
double MincoOptimizer::collisionLoss(
    const std::vector<Eigen::Vector2d> &pts,
    std::vector<Eigen::Vector2d> &grad_out) const
{
    double loss = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        float d = esdf_->getDistance(pts[i].x(), pts[i].y());
        if (d >= params_.safe_distance) continue;
        // 在障碍内或离障碍太近
        double penetration = params_.safe_distance - d;
        loss += penetration * penetration;

        // 梯度:沿 ESDF 梯度方向(指向远离障碍)推
        Eigen::Vector2d grad_esdf = esdf_->getGradient(pts[i].x(), pts[i].y());
        // 注:ESDF 梯度是单位向量(见 esdf_map.hpp 实现)
        // 当 d < 0(在障碍内)时梯度可能为 0,用数值差分兜底
        if (grad_esdf.norm() < kEps && d > 0) {
            grad_esdf = esdf_->getGradient(pts[i].x(), pts[i].y());
        }
        grad_out[i] -= 2.0 * penetration * grad_esdf;
    }
    return loss;
}

}  // namespace omni_planner
