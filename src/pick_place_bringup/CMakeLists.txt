cmake_minimum_required(VERSION 3.8)
project(pick_place_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config urdf rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
