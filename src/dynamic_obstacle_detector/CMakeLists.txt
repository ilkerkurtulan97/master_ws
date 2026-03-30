cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Main node
add_executable(voxel_subtraction_node src/voxel_subtraction_node.cpp)
target_include_directories(voxel_subtraction_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(voxel_subtraction_node ${PCL_LIBRARIES})
ament_target_dependencies(voxel_subtraction_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
)

install(TARGETS
  voxel_subtraction_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
