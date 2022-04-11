from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('spraybot_bringup')

    lifecycle_nodes_sensors = ['row_detection', 'row_exit_detection']

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use sim time'
    )
    ld.add_action(declare_use_sim_time)

    params = RewrittenYaml(
        source_file=PathJoinSubstitution([current_pkg, 'params', 'perception.yaml']),
        root_key='',
        param_rewrites={
            'autostart': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time')
            },
        convert_types=True,
    )

    pcl_processor_node = Node(
            package='pcl_processor',
            executable='processor_xyz',
            name='row_detection_processor_xyz',
            output='screen',
            parameters=[params])
    ld.add_action(pcl_processor_node)

    spraybot_row_exit_detection_node = LifecycleNode(
        package='spraybot_exit_detection',
        executable='row_exit_detection',
        name='row_exit_detection',
        namespace='/',
        output='screen',
        parameters=[params]
    )
    ld.add_action(spraybot_row_exit_detection_node)
    
    spraybot_row_detection_node = LifecycleNode(
        package='spraybot_row_detection',
        executable='row_detection',
        name='row_detection',
        namespace='/',
        output='screen',
        parameters=[params],
    )
    ld.add_action(spraybot_row_detection_node)

    # TODO: Create custom lifecycle manger with bond and auto-restart support
    lifecycle_manager_perception = Node(
        package='spraybot_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_perception',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes_sensors, 'bond_timeout': 0.0},
        ]
    )
    ld.add_action(lifecycle_manager_perception)

    return ld
