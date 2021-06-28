import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    AGENT_NAME = "agent2"
    
    ps2_two_share_dir = get_package_share_directory('plansys2_twoins_advanced')
    
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=AGENT_NAME,
        description='Namespace definition')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),

        launch_arguments={
            'model_file': ps2_two_share_dir + '/pddl/cleaner-domain.pddl',
            'namespace': namespace
            }.items()
    )

    ld = LaunchDescription()
    
    define_problem = Node(
        package='plansys2_twoins',
        executable='define_problem',
        name='define_problem',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": AGENT_NAME}])

    print_problem = Node(
        package='plansys2_twoins',
        executable='print_problem',
        name='print_problem',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": AGENT_NAME}]
    )

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # Problem experts nodes
    ld.add_action(define_problem)
    ld.add_action(print_problem)

    return ld
