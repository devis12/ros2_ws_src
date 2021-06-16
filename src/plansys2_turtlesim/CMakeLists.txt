cmake_minimum_required(VERSION 3.5)
project(plansys2_turtlesim)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtlesim_final_interfaces REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(turtle_dependencies
    turtlesim
    turtlesim_final_interfaces
)

set(psys2_act_client_dep
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

set(experts_lib
  plansys2_domain_expert
  plansys2_problem_expert
)

set(psys2_plan_exec
  plansys2_planner
  plansys2_executor
)

add_executable(turtles_master src/no_plan/turtles_master.cpp)
ament_target_dependencies(turtles_master rclcpp ${turtle_dependencies})

add_executable(turtle_controller src/no_plan/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp ${turtle_dependencies} geometry_msgs)

add_executable(turtle_pose_echo src/no_plan/turtle_pose_echo.cpp)
ament_target_dependencies(turtle_pose_echo rclcpp turtlesim)

add_executable(eating src/actions/eating.cpp)
ament_target_dependencies(eating rclcpp ${psys2_act_client_dep} turtlesim)

add_executable(movetoward src/actions/movetoward.cpp)
ament_target_dependencies(movetoward rclcpp ${psys2_act_client_dep} geometry_msgs ${turtle_dependencies} plansys2_problem_expert)

add_executable(acquiretarget src/actions/acquiretarget.cpp)
ament_target_dependencies(acquiretarget rclcpp ${psys2_act_client_dep})

add_executable(spawner src/problem_experts/spawner.cpp)
ament_target_dependencies(spawner rclcpp plansys2_problem_expert ${turtle_dependencies})

add_executable(compdistances src/problem_experts/computedistances.cpp)
ament_target_dependencies(compdistances rclcpp plansys2_problem_expert ${turtle_dependencies})

add_executable(cleanupKB src/problem_experts/cleanupKB.cpp)
ament_target_dependencies(cleanupKB rclcpp plansys2_problem_expert plansys2_msgs)

add_executable(plan_master src/plan_master.cpp)
ament_target_dependencies(plan_master rclcpp ${experts_lib} ${psys2_plan_exec} plansys2_msgs plansys2_pddl_parser)


install(TARGETS
  turtles_master
  turtle_controller
  eating
  movetoward
  acquiretarget
  spawner
  compdistances
  cleanupKB
  turtle_pose_echo
  plan_master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
