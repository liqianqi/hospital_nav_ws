cmake_minimum_required(VERSION 3.8)
project(omni_motion_planner)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出 compile_commands.json(给 clangd/IDE 用)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

# 头文件路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ============================================================
# 库:omni_motion_planner_core
# 后续把 .cpp 源文件加到这里
# ============================================================
add_library(omni_motion_planner_core SHARED
  src/search/astar.cpp
  src/search/kino_astar.cpp
  src/trajectory/minco_optimizer.cpp
  src/tracking/trajectory_tracker.cpp
  src/tracking/mppi_controller.cpp
)
#
target_include_directories(omni_motion_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(omni_motion_planner_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# 可执行文件
# ============================================================
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node omni_motion_planner_core)
ament_target_dependencies(planner_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS omni_motion_planner_core LIBRARY DESTINATION lib)
install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

# ============================================================
# 安装
# ============================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
