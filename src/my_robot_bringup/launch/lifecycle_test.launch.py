from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    number_publisher_name = 'my_number_publisher'
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    number_publisher_node = Node(
        package='lifecycle_py',
        executable='number_publisher',
        name=number_publisher_name,
        output='screen',
        parameters=[]
    )

    lifecycle_manager_node = Node(
        package='lifecycle_py',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'managed_node_name': number_publisher_name}
        ]
    )

    ld.add_action(namespace_arg)
    ld.add_action(number_publisher_node)
    ld.add_action(lifecycle_manager_node)

    return ld