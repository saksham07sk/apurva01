cmake_minimum_required(VERSION 3.8)
project(rmf_visualization_zones)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

add_executable(zone_visualizer src/ZoneVisualizer.cpp)
ament_target_dependencies(zone_visualizer
  rclcpp
  visualization_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
)

target_link_libraries(zone_visualizer yaml-cpp)

install(
  TARGETS zone_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

