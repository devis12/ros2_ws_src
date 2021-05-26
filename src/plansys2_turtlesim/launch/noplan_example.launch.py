from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    remap_mykill_srv = ("master_kill", "eat")

    spawn_turtles = Node(
        package="plansys2_turtlesim",
        executable="turtles_master",
        parameters=[{"spawning_frequency": 0.5}],
        remappings=[remap_mykill_srv]
    )

    controller = Node(
        package="plansys2_turtlesim",
        executable="turtle_controller",
        parameters=[{"turtle_name": "turtle1"},{"step": 0.16}],
        remappings=[remap_mykill_srv]
    )

    ld.add_action(turtlesim)
    ld.add_action(spawn_turtles)
    ld.add_action(controller)
    return ld