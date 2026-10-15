#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <queue>
#include <cstdint>

namespace omni_planner
{

class ESDFMap
{
public:
    /**
     * @param width, height  栅格尺寸
     * @param resolution     每格边长(米)
     * @param origin_x, origin_y  栅格(0,0)左下角在世界坐标的位置
     */
    ESDFMap(int width, int height, double resolution,
            double origin_x, double origin_y)
        : width_(width), height_(height),
          resolution_(resolution),
          origin_(origin_x, origin_y),
          distance_(static_cast<size_t>(width) * height,
                    std::numeric_limits<float>::max()) {}

    // 构建:从占据栅格地图(0=空闲, 255=障碍)计算距离场
    void buildFromOccupancyGrid(const std::vector<uint8_t>& grid_data)
    {
        std::fill(distance_.begin(), distance_.end(),
                  std::numeric_limits<float>::max());

        // 多源 BFS:所有障碍格为起点
        // 邻域松弛:正向步长=resolution,对角步长=sqrt(2)*resolution
        struct Node {
            int idx;
            float dist;
        };
        std::queue<Node> queue;

        // 初始化障碍格
        for (int i = 0; i < width_ * height_; ++i) {
            if (grid_data[static_cast<size_t>(i)] == 255) {
                distance_[i] = 0.0f;
                queue.push({i, 0.0f});
            }
        }

        // 8 邻域 (dx, dy, step_cost_in_cells)
        static constexpr int dx8[] = {-1, 1,  0, 0, -1, -1,  1, 1};
        static constexpr int dy8[] = { 0, 0, -1, 1, -1,  1, -1, 1};
        static constexpr float step8[] = {
            1.0f, 1.0f, 1.0f, 1.0f,                    // 上下左右
            1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f  // 对角线
        };

        while (!queue.empty()) {
            auto [idx, dist] = queue.front();
            queue.pop();

            int x = idx % width_;
            int y = idx / width_;

            for (int d = 0; d < 8; ++d) {
                int nx = x + dx8[d];
                int ny = y + dy8[d];
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                    continue;

                int nidx = ny * width_ + nx;
                float new_dist = dist + step8[d];  // 单位:格

                if (new_dist < distance_[nidx]) {
                    distance_[nidx] = new_dist;
                    queue.push({nidx, new_dist});
                }
            }
        }

        // 格数 → 米
        float res = static_cast<float>(resolution_);
        for (auto& d : distance_) {
            d *= res;
        }
    }

    /// 栅格坐标查距离(地图外返回 0=障碍)
    float getDistance(int x, int y) const
    {
        if (!isInside(x, y))
            return 0.0f;
        return distance_[static_cast<size_t>(y) * width_ + x];
    }

    /// 线性索引查距离
    float getDistance(int idx) const
    {
        return distance_[idx];
    }

    /// 世界坐标查距离
    float getDistance(double world_x, double world_y) const
    {
        int gx, gy;
        worldToGrid(world_x, world_y, gx, gy);
        if (!isInside(gx, gy))
            return 0.0f;  // 地图外视为障碍
        return getDistance(gx, gy);
    }

    /// 碰撞检测:考虑机器人半径
    bool isSafe(double world_x, double world_y, double robot_radius) const
    {
        return getDistance(world_x, world_y) > static_cast<float>(robot_radius);
    }

    bool isObstacle(int x, int y) const
    {
        return getDistance(x, y) <= 0.0f;
    }

    Eigen::Vector2d getGradient(int x, int y) const
    {
        // 边界返回零向量
        if (x <= 0 || x >= width_ - 1 || y <= 0 || y >= height_ - 1)
            return Eigen::Vector2d::Zero();

        float dx = getDistance(x + 1, y) - getDistance(x - 1, y);
        float dy = getDistance(x, y + 1) - getDistance(x, y - 1);

        Eigen::Vector2d grad(dx, dy);
        double norm = grad.norm();
        if (norm < 1e-6)
            return Eigen::Vector2d::Zero();
        return grad / norm;
    }

    Eigen::Vector2d getGradient(double world_x, double world_y) const
    {
        int gx, gy;
        worldToGrid(world_x, world_y, gx, gy);
        return getGradient(gx, gy);
    }

    void worldToGrid(double wx, double wy, int& gx, int& gy) const
    {
        gx = static_cast<int>((wx - origin_.x()) / resolution_);
        gy = static_cast<int>((wy - origin_.y()) / resolution_);
    }

    void gridToWorld(int gx, int gy, double& wx, double& wy) const
    {
        wx = origin_.x() + (gx + 0.5) * resolution_;
        wy = origin_.y() + (gy + 0.5) * resolution_;
    }

    bool isInside(int x, int y) const
    {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    // ================================================================
    // Getters
    // ================================================================
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    const Eigen::Vector2d& getOrigin() const { return origin_; }
    const std::vector<float>& getData() const { return distance_; }

private:
    int width_;
    int height_;
    double resolution_;
    Eigen::Vector2d origin_;
    std::vector<float> distance_;  ///< 每格到最近障碍的距离(米)
};

}  // namespace omni_planner
