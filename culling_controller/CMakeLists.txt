cmake_minimum_required(VERSION 3.8)
project(culling_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# ----- down_mover -----
add_executable(down_mover src/down_mover.cpp)
ament_target_dependencies(down_mover rclcpp cg_interfaces)
install(TARGETS down_mover DESTINATION lib/${PROJECT_NAME})

# ----- path_follower -----
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp cg_interfaces)
install(TARGETS path_follower DESTINATION lib/${PROJECT_NAME})

# ----- path_finder -----
add_executable(path_finder src/path_finder.cpp)
ament_target_dependencies(path_finder rclcpp cg_interfaces)
install(TARGETS path_finder DESTINATION lib/${PROJECT_NAME})

ament_package()
