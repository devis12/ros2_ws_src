cmake_minimum_required(VERSION 3.5)
project(plansys2_twoins_advanced)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(define_problem src/problem_experts/define_problem.cpp)
ament_target_dependencies(define_problem rclcpp plansys2_problem_expert plansys2_pddl_parser)

add_executable(print_problem src/problem_experts/print_problem.cpp)
ament_target_dependencies(print_problem rclcpp example_interfaces plansys2_problem_expert)

install(TARGETS
  define_problem
  print_problem
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
