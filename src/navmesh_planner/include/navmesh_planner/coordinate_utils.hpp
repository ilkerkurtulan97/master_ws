#pragma once

// Detour uses Y-UP, ROS2 uses Z-UP.
// ROS2: x=forward, y=left, z=up
// Detour: x=forward, y=up, z=left(ish)

inline void rosToDetour(double rx, double ry, double rz, float* d) {
  d[0] = static_cast<float>(rx);
  d[1] = static_cast<float>(rz);  // ROS Z (up) -> Detour Y (up)
  d[2] = static_cast<float>(ry);  // ROS Y -> Detour Z
}

inline void detourToRos(const float* d, double& rx, double& ry, double& rz) {
  rx = static_cast<double>(d[0]);
  ry = static_cast<double>(d[2]);  // Detour Z -> ROS Y
  rz = static_cast<double>(d[1]);  // Detour Y (up) -> ROS Z (up)
}
