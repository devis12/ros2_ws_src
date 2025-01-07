cmake_minimum_required(VERSION 3.8)
project(components_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)

add_library(number_pub_component SHARED src/number_publisher.cpp)
ament_target_dependencies(number_pub_component rclcpp rclcpp_components example_interfaces)
rclcpp_components_register_nodes(number_pub_component "my_namespace::NumberPublisher")

add_library(turtle_move_component SHARED src/turtle_move.cpp)
ament_target_dependencies(turtle_move_component rclcpp rclcpp_components rclcpp_action rclcpp_lifecycle std_srvs turtlesim my_robot_interfaces geometry_msgs)
rclcpp_components_register_nodes(turtle_move_component "my_namespace::TurtleMoveServer")

add_library(turtlebot_move_component SHARED src/turtlebot_move.cpp)
ament_target_dependencies(turtlebot_move_component rclcpp rclcpp_components rclcpp_action rclcpp_lifecycle std_srvs turtlesim my_robot_interfaces geometry_msgs)
rclcpp_components_register_nodes(turtlebot_move_component "my_namespace::TurtleBotMoveServer")

install(TARGETS
number_pub_component
turtle_move_component
turtlebot_move_component
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

add_executable(manual_composition src/manual_composition.cpp src/node1.cpp src/node2.cpp)
ament_target_dependencies(manual_composition rclcpp)

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
