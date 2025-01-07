cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp rclcpp_lifecycle example_interfaces)

add_executable(robot_move_server src/robot_move_server.cpp)
ament_target_dependencies(robot_move_server rclcpp rclcpp_lifecycle rclcpp_action my_robot_interfaces example_interfaces)

add_executable(lifecycle_robot_move_manager src/lifecycle_robot_move_manager.cpp)
ament_target_dependencies(lifecycle_robot_move_manager rclcpp lifecycle_msgs)

install(TARGETS
  number_publisher
  robot_move_server
  lifecycle_robot_move_manager
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
