cmake_minimum_required(VERSION 3.5)
project(my_robot_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/LedStates.msg"
  "srv/ComputeRectangleArea.srv"
  "srv/SetLed.srv"
  "action/CountUntil.action"
  "action/RobotMove.action"
  "action/Test.action"
  "action/TurtleMove.action"
  
  DEPENDENCIES
  geometry_msgs
)

ament_package()
