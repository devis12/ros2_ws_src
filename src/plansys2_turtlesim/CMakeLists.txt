cmake_minimum_required(VERSION 3.5)
project(plansys2_turtlesim)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtlesim_final_interfaces REQUIRED)

set(common_dependencies
    rclcpp
    turtlesim
    turtlesim_final_interfaces
)

add_executable(turtles_master src/turtles_master.cpp)
ament_target_dependencies(turtles_master ${common_dependencies})

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller ${common_dependencies} geometry_msgs)

install(TARGETS
  turtles_master
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
