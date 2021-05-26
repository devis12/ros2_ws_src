cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

add_executable(led_panel src/led_panel_node.cpp)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)

add_executable(battery src/battery_node.cpp)
ament_target_dependencies(battery rclcpp my_robot_interfaces)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action geometry_msgs my_robot_interfaces)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action geometry_msgs my_robot_interfaces)

add_executable(hanoi_moveontop src/hanoi_moveontop.cpp)
ament_target_dependencies(hanoi_moveontop rclcpp rclcpp_action plansys2_msgs plansys2_executor)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hw_status_publisher
  led_panel
  battery
  action_server
  action_client
  hanoi_moveontop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
