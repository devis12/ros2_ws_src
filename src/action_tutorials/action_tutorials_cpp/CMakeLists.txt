cmake_minimum_required(VERSION 3.5)
project(action_tutorials_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(fibonacci_action_server src/fibonacci_action_server.cpp)
ament_target_dependencies(fibonacci_action_server rclcpp rclcpp_action action_tutorials_interfaces)

add_executable(fibonacci_action_client src/fibonacci_action_client.cpp)
ament_target_dependencies(fibonacci_action_client rclcpp rclcpp_action action_tutorials_interfaces)

install(TARGETS
  fibonacci_action_server
  fibonacci_action_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
