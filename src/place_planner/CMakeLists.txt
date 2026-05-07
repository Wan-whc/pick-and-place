cmake_minimum_required(VERSION 3.8)
project(place_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_detector REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(common REQUIRED)

add_executable(place_planner_node src/place_planner_node.cpp)
ament_target_dependencies(place_planner_node
  rclcpp rclcpp_action geometry_msgs std_msgs
  moveit_ros_planning_interface moveit_msgs control_msgs trajectory_msgs
  tf2 tf2_geometry_msgs msgs common
)
target_include_directories(place_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${vision_detector_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS place_planner_node DESTINATION lib/${PROJECT_NAME})

ament_package()
