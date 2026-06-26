cmake_minimum_required(VERSION 3.8)
project(task_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(moveit_planning REQUIRED)

add_executable(pick_and_place src/pick_and_place_task.cpp)

ament_target_dependencies(pick_and_place
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_planning
)

install(TARGETS
  pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()