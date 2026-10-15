#include "omni_motion_planner/search/kino_astar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

namespace omni_planner
{

namespace {
constexpr double kPi = 3.14159265358979323846;

// 把任意角度归一化到 [-pi, pi]
inline double wrapAngle(double a)
{
    while (a > kPi)  a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}  // namespace

bool KinoDynamicAStar::planWithHeading(
    const Eigen::Vector2d &start, double start_theta,
    const Eigen::Vector2d &goal, double goal_theta)
{
    path_.clear();
    expanded_.clear();
    node_pool_.clear();
    lookup_.clear();

    // ===== 1. 边界校验 =====
    if (!esdf_->isSafe(start.x(), start.y(), params_.robot_radius) ||
        !esdf_->isSafe(goal.x(), goal.y(), params_.robot_radius)) {
        return false;
    }

    // ===== 2. 预计算 2D ESDF Dijkstra 启发式场 =====
    // 这是 Kino A* 性能关键:比纯欧氏距离紧很多,扩展节点数大幅下降
    buildHeuristicField(goal);

    // ===== 3. 起点 =====
    Eigen::Vector4d start_state(start.x(), start.y(), start_theta, 0.0);
    Node start_node;
    start_node.state  = start_state;
    start_node.g      = 0.0;
    start_node.h      = heuristic(start_state.head<2>());
    start_node.parent = -1;

    node_pool_.push_back(std::move(start_node));
    int start_idx = 0;
    lookup_[discretize(start_state)] = start_idx;
    expanded_.push_back(start_state);

    auto cmp = [](const QEntry &a, const QEntry &b) {
        return a.first > b.first;
    };
    std::priority_queue<QEntry, std::vector<QEntry>, decltype(cmp)> open(cmp);
    open.push({node_pool_[start_idx].g + node_pool_[start_idx].h, start_idx});

    // ===== 4. 控制量采样 =====
    // 加速度:从 -max_acc 到 +max_acc,n_acc 个等分点
    std::vector<double> acc_samples;
    for (int i = 0; i < params_.n_acc; ++i) {
        double t = params_.n_acc == 1 ? 0.5 : static_cast<double>(i) / (params_.n_acc - 1);
        acc_samples.push_back(-params_.max_acc + t * 2.0 * params_.max_acc);
    }
    // 角速度:同上
    std::vector<double> omega_samples;
    for (int i = 0; i < params_.n_omega; ++i) {
        double t = params_.n_omega == 1 ? 0.5 : static_cast<double>(i) / (params_.n_omega - 1);
        omega_samples.push_back(-params_.max_omega + t * 2.0 * params_.max_omega);
    }

    int goal_node_idx = -1;

    // ===== 5. 主循环 =====
    while (!open.empty()) {
        auto [f_push, cur_idx] = open.top();
        open.pop();

        const Node cur = node_pool_[cur_idx];  // 拷贝: 下面 push_back 扩容会让引用悬空

        // stale check:同一离散状态可能被 push 多次,只认 g 最小的那个
        // (这里的 stale 判断:open list 里的 f 落后于 node 自身 g+h)
        if (f_push > cur.g + cur.h + 1e-9) continue;

        // 抵达 goal?
        Eigen::Vector2d dxy = cur.state.head<2>() - goal;
        if (dxy.norm() < params_.goal_tol_xy &&
            std::abs(wrapAngle(cur.state.z() - goal_theta)) < params_.goal_tol_theta) {
            goal_node_idx = cur_idx;
            break;
        }

        // 节点上限保护
        if (node_pool_.size() >= params_.max_nodes) continue;

        // 枚举 (a, omega) 控制量
        for (double a : acc_samples) {
            for (double w : omega_samples) {
                Eigen::Vector4d next_state = integrate(cur.state, a, w, params_.dt);

                // 速度限幅
                if (std::abs(next_state.w()) > params_.max_vel + 1e-6) continue;

                // 碰撞检查(沿轨迹)
                if (inCollision(cur.state, next_state)) continue;

                // 代价:g = 时间 + 加权控制能量 + 障碍净空惩罚
                // 时间项鼓励尽快到达,控制能量项鼓励平滑轨迹
                // 净空项用指数衰减核: 离障碍越近惩罚越大,远处几乎为零
                //   penalty = lambda_obs * exp(-(clearance - robot_radius) / obs_decay)
                //   clearance = robot_radius 时 penalty = lambda_obs (最大)
                //   clearance = robot_radius + obs_decay 时 penalty ≈ lambda_obs / e
                const double clearance = esdf_->getDistance(next_state.x(), next_state.y());
                const double margin = clearance - params_.robot_radius;
                const double obs_penalty =
                    (margin < 0.0)
                        ? std::numeric_limits<double>::infinity()  // 已侵入障碍,不可走
                        : params_.lambda_obs * std::exp(-margin / params_.obs_decay);

                double edge_cost = params_.dt +
                                   params_.lambda * (a * a + w * w) * params_.dt +
                                   obs_penalty * params_.dt;
                double tentative_g = cur.g + edge_cost;

                // 离散 key 查 closed/open 表
                StateKey nk = discretize(next_state);
                auto it = lookup_.find(nk);
                if (it != lookup_.end()) {
                    Node &existing = node_pool_[it->second];
                    if (tentative_g >= existing.g) {
                        // 已有更优路径到达此离散状态, 跳过
                        continue;
                    }
                    // 找到更短路径,更新现有节点
                    existing.state  = next_state;
                    existing.g      = tentative_g;
                    existing.h      = heuristic(next_state.head<2>());
                    existing.parent = cur_idx;
                    open.push({existing.g + existing.h, it->second});
                } else {
                    Node nnode;
                    nnode.state  = next_state;
                    nnode.g      = tentative_g;
                    nnode.h      = heuristic(next_state.head<2>());
                    nnode.parent = cur_idx;
                    node_pool_.push_back(std::move(nnode));
                    int new_idx = static_cast<int>(node_pool_.size()) - 1;
                    lookup_[nk] = new_idx;
                    expanded_.push_back(next_state);
                    open.push({tentative_g + node_pool_[new_idx].h, new_idx});
                }
            }
        }
    }

    if (goal_node_idx < 0) return false;

    // ===== 6. 回溯路径 =====
    backtrack(goal_node_idx, goal_theta);
    return true;
}

KinoDynamicAStar::StateKey KinoDynamicAStar::discretize(const Eigen::Vector4d &s) const
{
    const double res = params_.xy_resolution > 0.0 ? params_.xy_resolution : esdf_->getResolution();
    const Eigen::Vector2d &o = esdf_->getOrigin();
    int gx = static_cast<int>(std::floor((s.x() - o.x()) / res));
    int gy = static_cast<int>(std::floor((s.y() - o.y()) / res));

    // theta 离散到 [0, n_theta)
    double theta = wrapAngle(s.z());
    double t01 = (theta + kPi) / (2.0 * kPi);   // [0, 1]
    int theta_idx = static_cast<int>(t01 * params_.n_theta) % params_.n_theta;

    // 速度离散到 5 个桶:[-max, -half, 0, +half, +max]
    int v_idx = 0;
    if (params_.max_vel > 1e-6) {
        double r = s.w() / params_.max_vel;          // [-1, 1]
        v_idx = static_cast<int>(std::round((r + 1.0) * 0.5 * 4.0));  // [0, 4]
        v_idx = std::clamp(v_idx, 0, 4);
    }

    return {gx, gy, theta_idx, v_idx};
}

Eigen::Vector4d KinoDynamicAStar::integrate(const Eigen::Vector4d &s,
                                            double a, double omega,
                                            double dt) const
{
    // unicycle / bicycle 运动学(Euler 积分,够 grid 规划用)
    double x     = s.x();
    double y     = s.y();
    double theta = s.z();
    double v     = s.w();

    // 加速度限幅
    double v_new = v + a * dt;
    v_new = std::clamp(v_new, -params_.max_vel, params_.max_vel);

    // 用半步速度更新位置(比纯 Euler 准一点)
    double v_mid = 0.5 * (v + v_new);
    double theta_mid = theta + 0.5 * omega * dt;

    x     += v_mid * std::cos(theta_mid) * dt;
    y     += v_mid * std::sin(theta_mid) * dt;
    theta += omega * dt;
    theta  = wrapAngle(theta);

    return Eigen::Vector4d(x, y, theta, v_new);
}

bool KinoDynamicAStar::inCollision(const Eigen::Vector4d &s0,
                                   const Eigen::Vector4d &s1) const
{
    // 沿直线采样若干点做 ESDF 检测
    Eigen::Vector2d p0 = s0.head<2>();
    Eigen::Vector2d p1 = s1.head<2>();
    double dist = (p1 - p0).norm();
    int steps = std::max(1, static_cast<int>(std::ceil(dist / (0.5 * esdf_->getResolution()))));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector2d p = p0 + t * (p1 - p0);
        if (!esdf_->isSafe(p.x(), p.y(), params_.robot_radius))
            return true;
    }
    return false;
}

void KinoDynamicAStar::buildHeuristicField(const Eigen::Vector2d &goal)
{
    // 在 ESDF grid 上从 goal 做一次 Dijkstra, 得到每个 cell 到 goal 的
    // 最短无障碍距离。 这比欧氏距离紧很多, 启发式质量决定 A* 速度。
    h_w_   = esdf_->getWidth();
    h_h_   = esdf_->getHeight();
    h_res_ = esdf_->getResolution();
    h_origin_ = esdf_->getOrigin();

    h_field_.assign(static_cast<std::size_t>(h_w_) * h_h_,
                    std::numeric_limits<double>::infinity());

    int gx, gy;
    esdf_->worldToGrid(goal.x(), goal.y(), gx, gy);
    if (!esdf_->isInside(gx, gy)) return;

    auto idx = [this](int x, int y) { return y * h_w_ + x; };

    // 用 std::priority_queue 做 Dijkstra
    using Entry = std::pair<double, int>;  // (dist, cell_idx)
    auto cmp = [](const Entry &a, const Entry &b) { return a.first > b.first; };
    std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> pq(cmp);

    h_field_[idx(gx, gy)] = 0.0;
    pq.push({0.0, idx(gx, gy)});

    static constexpr int dx8[] = {-1, 1,  0, 0, -1, -1,  1, 1};
    static constexpr int dy8[] = { 0, 0, -1, 1, -1,  1, -1, 1};
    static constexpr double step8[] = {1.0, 1.0, 1.0, 1.0,
                                       1.41421356, 1.41421356,
                                       1.41421356, 1.41421356};

    while (!pq.empty()) {
        auto [d, ci] = pq.top();
        pq.pop();
        if (d > h_field_[ci]) continue;    // stale

        int x = ci % h_w_;
        int y = ci / h_w_;

        for (int k = 0; k < 8; ++k) {
            int nx = x + dx8[k];
            int ny = y + dy8[k];
            if (nx < 0 || nx >= h_w_ || ny < 0 || ny >= h_h_) continue;

            // 障碍或离障碍太近的格子不允许穿过
            if (esdf_->getDistance(nx, ny) <= 0.0f) continue;

            int nidx = ny * h_w_ + nx;
            double nd = d + step8[k] * h_res_;
            if (nd < h_field_[nidx]) {
                h_field_[nidx] = nd;
                pq.push({nd, nidx});
            }
        }
    }
}

double KinoDynamicAStar::heuristic(const Eigen::Vector2d &pos) const
{
    // 用预计算的 Dijkstra 场(紧),fallback 到欧氏距离
    int gx, gy;
    esdf_->worldToGrid(pos.x(), pos.y(), gx, gy);
    if (esdf_->isInside(gx, gy)) {
        double d = h_field_[gy * h_w_ + gx];
        if (d < std::numeric_limits<double>::infinity()) return params_.h_weight * d;
    }
    // 走不到 goal 的连通分量:用欧氏距离作为下界
    return 0.0;   // 已经无路可达,A* 会自然失败,但启发式别小于 0
}

void KinoDynamicAStar::backtrack(int goal_node_idx, double /*goal_theta*/)
{
    // 从 goal 节点反向追溯 parent,收集所有节点状态
    std::vector<int> chain;
    for (int i = goal_node_idx; i != -1; i = node_pool_[i].parent) {
        chain.push_back(i);
    }
    std::reverse(chain.begin(), chain.end());

    // 把链上每段(node[i] → node[i+1])重新积分,加密输出
    // 这样最终 trajectory 比 dt 的粗采样密得多,适合给下游控制器用
    path_.clear();
    double t = 0.0;
    if (chain.empty()) return;

    // 起点
    const Node &n0 = node_pool_[chain.front()];
    path_.push_back({n0.state.head<2>(), n0.state.z(), t, n0.g});

    for (size_t i = 1; i < chain.size(); ++i) {
        const Node &prev = node_pool_[chain[i - 1]];
        const Node &cur  = node_pool_[chain[i]];

        // 估计这段用的控制量(从两端状态反推)
        double dtheta = wrapAngle(cur.state.z() - prev.state.z());
        double dv    = cur.state.w() - prev.state.w();

        double a_est     = dv / params_.dt;
        double omega_est = dtheta / params_.dt;

        // 细化采样:每段分 5 个子步
        const int substeps = 5;
        double sub_dt = params_.dt / substeps;
        Eigen::Vector4d s = prev.state;
        for (int k = 0; k < substeps; ++k) {
            s = integrate(s, a_est, omega_est, sub_dt);
            t += sub_dt;
            path_.push_back({s.head<2>(), s.z(), t, cur.g});
        }
    }
}

}  // namespace omni_planner
