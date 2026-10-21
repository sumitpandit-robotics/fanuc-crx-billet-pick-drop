cmake_minimum_required(VERSION 3.8)
project(fanuc_pick_drop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_drop src/pick_drop.cpp)
target_compile_features(pick_drop PUBLIC cxx_std_17)
ament_target_dependencies(pick_drop
  rclcpp moveit_core geometric_shapes moveit_ros_planning_interface moveit_task_constructor_core
  tf2_eigen tf2_geometry_msgs)

install(TARGETS pick_drop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models urdf srdf DESTINATION share/${PROJECT_NAME})

ament_package()
