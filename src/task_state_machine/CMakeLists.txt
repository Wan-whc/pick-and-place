cmake_minimum_required(VERSION 3.8)
project(task_state_machine)

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
find_package(std_msgs REQUIRED)
find_package(vision_detector REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(task_state_machine_node src/task_state_machine_node.cpp)
ament_target_dependencies(task_state_machine_node
  rclcpp rclcpp_action geometry_msgs std_msgs
  moveit_ros_planning_interface moveit_msgs control_msgs
  tf2 tf2_geometry_msgs msgs common ament_index_cpp behaviortree_cpp
)


target_include_directories(task_state_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${vision_detector_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS task_state_machine_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees/ DESTINATION share/${PROJECT_NAME}/trees)

ament_package()
