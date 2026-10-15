cmake_minimum_required(VERSION 3.8)
project(hospital_sim)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
# 导出 compile_commands.json，clangd 才能找到 livox_ros_driver2 等头文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(livox_sim_merger src/livox_sim_merger.cpp)
target_link_libraries(livox_sim_merger Eigen3::Eigen)
ament_target_dependencies(livox_sim_merger rclcpp sensor_msgs tf2_ros tf2_eigen livox_ros_driver2)
install(TARGETS livox_sim_merger DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/omni_wheel_kinematics.py scripts/convert_hospital_world.py scripts/auto_tour.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
# AWS 医院模型 (third_party 下的仓库带 COLCON_IGNORE, 模型从这里安装)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aws-robomaker-hospital-world/models/
  DESTINATION share/${PROJECT_NAME}/models)
ament_package()
