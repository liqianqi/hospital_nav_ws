cmake_minimum_required(VERSION 3.22)
project(gicp_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenMP REQUIRED)
# small_gicp 由 third_party/install_deps.sh 系统安装
find_package(small_gicp REQUIRED)

add_library(gicp_relocalization_node SHARED
  src/gicp_relocalization.cpp
  src/global_search_2d.cpp
)
target_include_directories(gicp_relocalization_node PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(gicp_relocalization_node
  ${PCL_LIBRARIES}
  small_gicp
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(gicp_relocalization_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
rclcpp_components_register_node(gicp_relocalization_node
  PLUGIN "gicp_relocalization::GicpRelocalizationNode"
  EXECUTABLE gicp_relocalization_exec
  EXECUTOR MultiThreadedExecutor  # 全局搜索(~8s)与 map->odom 发布在不同回调组并行
)

install(TARGETS gicp_relocalization_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
