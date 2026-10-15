#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

namespace omni_planner
{

struct RobotState
{
    Eigen::Vector2d position{0.0, 0.0};   // map frame
    Eigen::Vector2d velocity{0.0, 0.0};   // map frame

    double yaw{0.0};
    double wz{0.0};

    double stamp{0.0};
};

}