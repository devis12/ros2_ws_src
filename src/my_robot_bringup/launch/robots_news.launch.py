from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_names = ["Giskard", "BB8", "Daneel", "Lander", "C3PO"]
    robot_news_station_nodes = []

    for name in robot_names:
        robot_news_station_nodes.append(Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name="robot_news_station_" + name.lower(),
            parameters=[{"robot_name": name}]
        ))

    smartphone1 = Node(
        package="my_cpp_pkg",
        executable="smartphone"
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)

    ld.add_action(smartphone1)
    return ld