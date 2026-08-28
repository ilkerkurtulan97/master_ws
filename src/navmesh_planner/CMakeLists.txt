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
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation kdtree search)

# Build Detour as static library
file(GLOB DETOUR_SOURCES "third_party/Detour/Source/*.cpp")
add_library(detour STATIC ${DETOUR_SOURCES})
target_include_directories(detour PUBLIC third_party/Detour/Include)

# Build DetourTileCache as static library
file(GLOB DETOUR_TC_SOURCES "third_party/DetourTileCache/Source/*.cpp")
add_library(detour_tilecache STATIC ${DETOUR_TC_SOURCES})
target_include_directories(detour_tilecache PUBLIC
  third_party/DetourTileCache/Include
  third_party/Detour/Include
)
target_link_libraries(detour_tilecache PUBLIC detour)

# Build FastLZ as static C library
add_library(fastlz STATIC third_party/fastlz/fastlz.c)
target_include_directories(fastlz PUBLIC third_party/fastlz)
set_target_properties(fastlz PROPERTIES LINKER_LANGUAGE C)

# NavMesh planner node
add_executable(navmesh_planner_node
  src/navmesh_planner_node.cpp
  src/navmesh_loader.cpp
)
target_include_directories(navmesh_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(navmesh_planner_node
  detour
  detour_tilecache
  fastlz
  ${PCL_LIBRARIES}
)
ament_target_dependencies(navmesh_planner_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
)

install(TARGETS navmesh_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
