#pragma once


#include <eigen3/Eigen/Core>

#include <cstdint>
#include <vector>
#include <optional>


namespace omni_planner
{

// data 的值只有 0 127 255 分别代表空闲 未知 障碍
struct GridMap{
    int width;
    int height;
    std::vector<uint8_t> data;
    double resolution{0.05};

    // 世界坐标中，grid(0,0)左下角的位置
    Eigen::Vector2d origin{0.0, 0.0};

    GridMap(int width, int height ,double world_x, double world_y) 
    : width(width), height(height), data(width * height), origin(world_x, world_y) {}

    uint8_t getCell(int x, int y) const {
        return data[x * height + y];
    }

    void setCell(int x, int y, uint8_t state) {
        data[x * height + y] = state;
    }

    int getWidth() const {
        return width;
    }

    int getHeight() const {
        return height;
    }

    bool isInside(const int & index) const {
        return index >= 0 && index < width * height;
    }

    bool isInside(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    std::tuple<int, int> worldToGrid(const double x, const double y) const{
        int grid_x = std::round((x - origin.x()) / resolution);
        int grid_y = std::round((y - origin.y()) / resolution);
        return std::make_tuple(grid_x, grid_y);
    }

    std::tuple<double, double> gridToWorld(int x, int y) const{
        double world_x = origin.x() + x * resolution;
        double world_y = origin.y() + y * resolution;
        return std::make_tuple(world_x, world_y);
    }

};

}