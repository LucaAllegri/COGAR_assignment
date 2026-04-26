cmake_minimum_required(VERSION 3.8)
project(ur5e_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

set(common_dependencies
  tf2_ros
  rclcpp
  tf2
)

set(msgs_dependencies
  tf2_geometry_msgs
  geometry_msgs
  shape_msgs
)

set(moveit_dependencies
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(motion_planner   src/motion_planner.cpp)

ament_target_dependencies(motion_planner  ${moveit_dependencies} ${msgs_dependencies} ${common_dependencies})

install(TARGETS
  motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
