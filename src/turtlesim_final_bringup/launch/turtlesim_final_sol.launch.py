from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    spawn_turtles = Node(
        package="turtlesim_final_utils",
        executable="spawn_turtles_sol",
        parameters=[{"catch_closest_turtle_first": True}]
    )

    controller = Node(
        package="turtlesim_final_utils",
        executable="controller_turtle_sol",
        parameters=[{"spawn_frequency": 2.5},{"turtle_name_frequency": "my_turtle"}]
    )

    ld.add_action(turtlesim)
    ld.add_action(spawn_turtles)
    ld.add_action(controller)
    return ld