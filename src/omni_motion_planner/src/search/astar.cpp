#include "omni_motion_planner/search/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace omni_planner
{

bool AStar::search(const Eigen::Vector2i &start, Eigen::Vector2i goal)
{
    // ===== 1. 边界校验 =====
    if (!grid_map_->isInside(start[0], start[1]) ||
        !grid_map_->isInside(goal[0], goal[1])) {
        return false;
    }

    if (grid_map_->getCell(start[0], start[1]) == 255) return false;

    if (start == goal) {
        grid_path_.clear();
        grid_path_.push_back(start);
        return true;
    }

    // ===== 2. goal 是障碍时,向外螺旋搜索最近空闲点 =====
    if (grid_map_->getCell(goal[0], goal[1]) == 255) {
        bool ok = false;
        for (int r = 1; r <= 5 && !ok; ++r) {
            for (int i = -r; i <= r && !ok; ++i) {
                for (int j = -r; j <= r && !ok; ++j) {
                    int nx = goal[0] + i, ny = goal[1] + j;
                    if (!grid_map_->isInside(nx, ny)) continue;
                    if (grid_map_->getCell(nx, ny) == 0) {
                        goal[0] = nx;
                        goal[1] = ny;
                        ok = true;
                    }
                }
            }
        }
        if (!ok) return false;
    }

    // ===== 3. 准备复用的 cells 数组(地图尺寸变化时才重新分配) =====
    const int W = grid_map_->getWidth();
    const int H = grid_map_->getHeight();
    if (cells_w_ != W || cells_h_ != H) {
        cells_.assign(static_cast<std::size_t>(W) * H, Cell{});
        cells_w_ = W;
        cells_h_ = H;
    }
    ++generation_;  // 本次 search 的代次

    // 用 lambda 访问当前代的 cell:generation 不匹配时按"未访问"处理
    auto get_cell = [&](int x, int y) -> Cell & {
        int i = y * W + x;
        Cell &c = cells_[i];
        if (c.generation != generation_) {
            // 第一次碰到这个格子,懒初始化
            c.generation = generation_;
            c.g = std::numeric_limits<double>::infinity();
            c.f = std::numeric_limits<double>::infinity();
            c.parent = -1;
        }
        return c;
    };

    // open_list 存 (f_at_push, x, y)
    using QEntry = std::tuple<double, int, int>;
    auto cmp = [](const QEntry &a, const QEntry &b) {
        return std::get<0>(a) > std::get<0>(b);
    };
    std::priority_queue<QEntry, std::vector<QEntry>, decltype(cmp)> open_list(cmp);

    Cell &start_cell = get_cell(start[0], start[1]);
    start_cell.g = 0.0;
    start_cell.f = heuristic(start, goal);
    open_list.push({start_cell.f, start[0], start[1]});

    constexpr double SQRT2 = 1.4142135623730951;
    bool found = false;

    // ===== 4. 主循环 =====
    while (!open_list.empty()) {
        auto [f_push, cx, cy] = open_list.top();
        open_list.pop();

        Cell &cur = get_cell(cx, cy);
        // stale check:push 时的 f 已经落后于最新 f → 跳过
        if (f_push > cur.f) continue;

        if (cx == goal[0] && cy == goal[1]) {
            found = true;
            break;
        }

        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) continue;

                int nx = cx + i, ny = cy + j;
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (grid_map_->getCell(nx, ny) == 255) continue;

                // 对角线穿墙检查:两个相邻正交格不能同时是障碍
                if (i != 0 && j != 0) {
                    if (grid_map_->getCell(cx + i, cy) == 255 ||
                        grid_map_->getCell(cx, cy + j) == 255) {
                        continue;
                    }
                }

                double step = (i != 0 && j != 0) ? SQRT2 : 1.0;
                double tentative_g = cur.g + step;

                Cell &nb = get_cell(nx, ny);
                if (tentative_g < nb.g) {
                    nb.g = tentative_g;
                    nb.f = tentative_g + heuristic(Eigen::Vector2i(nx, ny), goal);
                    nb.parent = cy * W + cx;
                    open_list.push({nb.f, nx, ny});
                }
            }
        }
    }

    if (!found) return false;

    // ===== 5. 回溯路径 =====
    grid_path_.clear();
    int p = goal[1] * W + goal[0];
    while (p != -1) {
        grid_path_.emplace_back(p % W, p / W);
        Cell &c = cells_[p];
        if (c.generation != generation_) break;  // 安全保护
        p = c.parent;
    }
    std::reverse(grid_path_.begin(), grid_path_.end());

    // ===== 6. 路径简化(去除锯齿) =====
    simplifyPath();

    return true;
}

bool AStar::plan(const Eigen::Vector2d &start, const Eigen::Vector2d &goal)
{
    // 世界坐标 → grid 坐标
    auto [sx, sy] = grid_map_->worldToGrid(start.x(), start.y());
    auto [gx, gy] = grid_map_->worldToGrid(goal.x(), goal.y());

    if (!search(Eigen::Vector2i(sx, sy), Eigen::Vector2i(gx, gy)))
        return false;

    // grid_path_ → PathPlanner::path_(TrajectoryPoint)
    path_.clear();
    path_.reserve(grid_path_.size());
    for (const auto &g : grid_path_) {
        auto [wx, wy] = grid_map_->gridToWorld(g.x(), g.y());
        path_.push_back({Eigen::Vector2d(wx, wy), 0.0, 0.0, 0.0});
    }
    return true;
}

bool AStar::hasLineOfSight(int x0, int y0, int x1, int y1) const
{
    // Bresenham 直线算法,检查直线上每个格子是否空闲
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;

    while (true) {
        if (!grid_map_->isInside(x, y)) return false;
        if (grid_map_->getCell(x, y) == 255) return false;
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
    return true;
}

void AStar::simplifyPath()
{
    if (grid_path_.size() <= 2) return;

    std::vector<Eigen::Vector2i> simplified;
    simplified.push_back(grid_path_.front());

    size_t anchor = 0;
    while (anchor + 1 < grid_path_.size()) {
        // 从 anchor 出发,找最远一个能直接看到的点
        size_t next = anchor + 1;
        for (size_t k = grid_path_.size() - 1; k > anchor + 1; --k) {
            const auto &a = grid_path_[anchor];
            const auto &b = grid_path_[k];
            if (hasLineOfSight(a.x(), a.y(), b.x(), b.y())) {
                next = k;
                break;
            }
        }
        simplified.push_back(grid_path_[next]);
        anchor = next;
    }

    grid_path_ = std::move(simplified);
}

}  // namespace omni_planner
