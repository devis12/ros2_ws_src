from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()

    '''
    reading_laser = Node(
        package="gazebo_training",
        executable="reading_laser",
        output="screen",
        remappings=[
            ("laser_scan", "/dolly/laser_scan")
        ]
    )

    moving_robot = Node(
        package="gazebo_training",
        executable="moving_robot",
        output="screen"
    )

    ld.add_action(reading_laser)
    ld.add_action(moving_robot)
    '''

    obstacle_avoidance = Node(
        package="gazebo_training",
        executable="obstacle_avoidance",
        output="screen",
        remappings=[
            ("laser_scan", "/dolly/laser_scan"),
            ("cmd_vel", "/dolly/cmd_vel")
        ],
        parameters=[
            {"linear_vel": 0.3},
            {"angular_vel": 0.5}
        ]
    )
    ld.add_action(obstacle_avoidance)

    return ld