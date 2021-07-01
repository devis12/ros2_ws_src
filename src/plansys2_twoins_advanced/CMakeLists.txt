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
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(example_interfaces REQUIRED)

set(psys2_problem_exp_dep
    rclcpp
    plansys2_problem_expert
)

set(psys2_act_client_dep
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

set(psys2_plan_exec
  plansys2_planner
  plansys2_executor
)

add_executable(define_problem src/problem_experts/define_problem.cpp)
ament_target_dependencies(define_problem ${psys2_problem_exp_dep} plansys2_pddl_parser plansys2_msgs)

add_executable(print_problem src/problem_experts/print_problem.cpp)
ament_target_dependencies(print_problem ${psys2_problem_exp_dep} example_interfaces)

add_executable(movetoward src/action_performers/movetoward.cpp)
ament_target_dependencies(movetoward ${psys2_act_client_dep})

add_executable(doclean src/action_performers/doclean.cpp)
ament_target_dependencies(doclean ${psys2_act_client_dep})

add_executable(recharge src/action_performers/recharge.cpp)
ament_target_dependencies(recharge ${psys2_act_client_dep})

add_executable(plan_master src/plan_master.cpp)
ament_target_dependencies(
  plan_master 
  ${psys2_problem_exp_dep} 
  plansys2_domain_expert 
  ${psys2_plan_exec} 
  plansys2_msgs 
  plansys2_pddl_parser
  example_interfaces
)

install(TARGETS
  define_problem
  print_problem
  movetoward
  recharge
  doclean
  plan_master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
