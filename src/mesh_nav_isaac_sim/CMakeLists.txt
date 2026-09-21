cmake_minimum_required(VERSION 3.5)
project(mesh_nav_isaac_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_package()
