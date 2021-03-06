import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ps2_t_share_dir = get_package_share_directory('plansys2_turtlesim')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace1')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    '''
    namespace2 = LaunchConfiguration('namespace2')
    
    declare_namespace2_cmd = DeclareLaunchArgument(
        'namespace2',
        default_value='agent2',
        description='Namespace2')

    plansys2_cmd2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),

        launch_arguments={
            'model_file': ps2_t_share_dir + '/pddl/turtle-domain.pddl',#'/home/devis/ros2_ws/src/plansys2_turtlesim/pddl/turtle-domain.pddl',
            'namespace': namespace2
            }.items()
    )
    '''

    

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),

        launch_arguments={
            'model_file': ps2_t_share_dir + '/pddl/turtle-domain.pddl',#'/home/devis/ros2_ws/src/plansys2_turtlesim/pddl/turtle-domain.pddl',
            'namespace': namespace
            }.items()
    )

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    # Specify the actions
    acquiretarget = Node(
        package='plansys2_turtlesim',
        executable='acquiretarget',
        name='acquiretarget',
        namespace=namespace,
        output='screen',
        parameters=[])

    eating = Node(
        package='plansys2_turtlesim',
        executable='eating',
        name='eating',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    movetoward = Node(  
        package='plansys2_turtlesim',
        executable='movetoward',
        name='movetoward',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    spawner = Node(
        package='plansys2_turtlesim',
        executable='spawner',
        name='spawner',
        namespace=namespace,
        output='screen',
        parameters=[])

    compdistances = Node(
        package='plansys2_turtlesim',
        executable='compdistances',
        name='compdistances',
        namespace=namespace,
        output='screen',
        parameters=[{"controlled_name": "turtle1"}])

    cleanupKB = Node(
        package='plansys2_turtlesim',
        executable='cleanupKB',
        name='cleanupKB',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    ld = LaunchDescription()

    # turtle simulator node
    ld.add_action(turtlesim)

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    #Action executors nodes
    ld.add_action(acquiretarget)
    ld.add_action(eating)
    ld.add_action(movetoward)

    #Problem experts
    ld.add_action(spawner)
    ld.add_action(compdistances)
    ld.add_action(cleanupKB)


    #second namespace
    #ld.add_action(declare_namespace2_cmd)
    #ld.add_action(plansys2_cmd2)

    return ld
