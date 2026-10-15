#pragma once

#include "omni_motion_planner/map/esdf_map.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace omni_planner
{

/**
 * @brief 碰撞检测器,封装 ESDF 查询,提供点/线段/轨迹三种检测
 *
 * 基于 ESDF 距离场:距离 > 机器人半径 = 安全,距离 ≤ 半径 = 碰撞
 */
class CollisionChecker
{
public:
    CollisionChecker(const ESDFMap& esdf, double robot_radius)
        : esdf_(esdf), robot_radius_(robot_radius) {}

    /**
     * @brief 点碰撞检测
     * @return true=安全, false=碰撞
     */
    bool isPointSafe(double world_x, double world_y) const
    {
        return esdf_.getDistance(world_x, world_y) > robot_radius_;
    }

    /**
     * @brief 线段碰撞检测(沿线段采样多个点逐一检查)
     *
     * Kino-A* 扩展节点时用:检查从当前节点到候选节点的连线是否安全。
     * 采样间距 = ESDF 分辨率,保证不漏检。
     *
     * @param x0,y0 起点(世界坐标)
     * @param x1,y1 终点(世界坐标)
     * @return true=整条线段安全
     */
    bool isSegmentSafe(double x0, double y0, double x1, double y1) const
    {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double length = std::sqrt(dx * dx + dy * dy);

        // 采样步长 = ESDF 分辨率(不漏检)
        double step = esdf_.getResolution();
        int samples = static_cast<int>(length / step) + 1;

        for (int i = 0; i <= samples; ++i) {
            double t = static_cast<double>(i) / samples;
            double px = x0 + t * dx;
            double py = y0 + t * dy;
            if (!isPointSafe(px, py))
                return false;
        }
        return true;
    }

    /**
     * @brief 轨迹碰撞检测(检查整条轨迹的所有点)
     *
     * MINCO 优化后的轨迹是连续的,离散采样后逐一检查。
     *
     * @param points 轨迹点(世界坐标)
     * @return true=整条轨迹安全
     */
    bool isTrajectorySafe(const std::vector<Eigen::Vector2d>& points) const
    {
        for (const auto& pt : points) {
            if (!isPointSafe(pt.x(), pt.y()))
                return false;
        }
        return true;
    }

    /**
     * @brief 查询点到最近障碍的距离(用于轨迹优化的代价计算)
     *
     * MINCO 优化时,距离越近代价越高,推动轨迹远离障碍
     *
     * @return 距离(米)。< robot_radius 说明碰撞。
     */
    double getClearance(double world_x, double world_y) const
    {
        return static_cast<double>(esdf_.getDistance(world_x, world_y));
    }

    /**
     * @brief 碰撞惩罚代价(轨迹优化用)
     *
     * 安全时代价=0,进入机器人半径范围后代价急剧增大
     *                 penalty
     *                   ↑
     *              inf  ┃ █
     *                   ┃ ██
     *                   ┃ ███
     *                   ┃ ████
     *          radius → ┃████████████  ← 进入此范围开始惩罚
     *                   ┃
     *          distance ─────────────→
     *
     * @return 代价值,安全=0,碰撞=极大值
     */
    double getCollisionCost(double world_x, double world_y) const
    {
        double dist = getClearance(world_x, world_y);
        if (dist > robot_radius_)
            return 0.0;  // 安全区,无代价

        if (dist <= 0.0)
            return 1e6;  // 撞墙,极大代价

        // 距离越近代价越高(二次函数)
        double ratio = (robot_radius_ - dist) / robot_radius_;
        return ratio * ratio * 100.0;
    }

    double getRobotRadius() const { return robot_radius_; }

private:
    const ESDFMap& esdf_;
    double robot_radius_;
};

}  // namespace omni_planner
