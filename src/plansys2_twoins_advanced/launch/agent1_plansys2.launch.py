import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    AGENT_NAME = "agent1"
    ACTION_SUFF = "_ag1"

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
            'plansys2_bringup_launch_distributed.py')),

        launch_arguments={
            'model_file': ps2_two_share_dir + '/pddl/cleaner-domain1.pddl',
            'namespace': namespace
            }.items()
    )

    ld = LaunchDescription()
    
    define_problem = Node(
        package='plansys2_twoins_advanced',
        executable='define_problem',
        name='define_problem',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": AGENT_NAME}])

    print_problem = Node(
        package='plansys2_twoins_advanced',
        executable='print_problem',
        name='print_problem',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": AGENT_NAME}]
    )

    action_movetoward = Node(
        package='plansys2_twoins_advanced',
        executable='movetoward',
        name='movetoward',
        namespace=namespace,
        output='screen',
        parameters=[{"action_suffix": ACTION_SUFF}]
    )

    action_doclean = Node(
        package='plansys2_twoins_advanced',
        executable='doclean',
        name='doclean',
        namespace=namespace,
        output='screen',
        parameters=[{"action_suffix": ACTION_SUFF}]
    )

    action_recharging = Node(
        package='plansys2_twoins_advanced',
        executable='recharge',
        name='recharge',
        namespace=namespace,
        output='screen',
        parameters=[{"action_suffix": ACTION_SUFF}]
    )

    plan_master = Node(
        package='plansys2_twoins_advanced',
        executable='plan_master',
        name='plan_master',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": AGENT_NAME}]
    )

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # Problem expert nodes
    ld.add_action(define_problem)
    ld.add_action(print_problem)

    #Action performer nodes
    ld.add_action(action_movetoward)
    ld.add_action(action_doclean)
    ld.add_action(action_recharging)
    
    #Plan Master
    ld.add_action(plan_master)

    return ld
