cmake_minimum_required(VERSION 3.8)
project(navmesh_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build Detour as static library
file(GLOB DETOUR_SOURCES "third_party/Detour/Source/*.cpp")
add_library(detour STATIC ${DETOUR_SOURCES})
target_include_directories(detour PUBLIC third_party/Detour/Include)

# NavMesh planner node
add_executable(navmesh_planner_node
  src/navmesh_planner_node.cpp
  src/navmesh_loader.cpp
)
target_include_directories(navmesh_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navmesh_planner_node detour)
ament_target_dependencies(navmesh_planner_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS navmesh_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
