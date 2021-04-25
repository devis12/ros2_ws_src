from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_screen = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    spawn_turtles = Node(
        package="turtlesim_final_utils",
        executable="spawn_turtles"
    )

    controller = Node(
        package="turtlesim_final_utils",
        executable="controller_turtle",
        parameters=[{"turtle_controlled": "turtle1"}]
    )

    ld.add_action(turtlesim_screen)
    ld.add_action(spawn_turtles)
    ld.add_action(controller)
    return ld