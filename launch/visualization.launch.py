from pytest import param
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('spraybot_bringup')

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    declare_use_sim_time = DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time)

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([current_pkg, 'rviz', 'simulation.rviz']),
        description='Full path to the Rviz config file to use')
    ld.add_action(declare_rviz_config_file)

    declare_enable_mapviz = DeclareLaunchArgument(
        'mapviz',
        default_value='true',
        description='Start Mapviz')
    ld.add_action(declare_enable_mapviz)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[param_substitutions])
    ld.add_action(rviz_node)

    # Include other launch files
    mapviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'mapviz.launch.py'])),
            condition=IfCondition(LaunchConfiguration('mapviz')),
            launch_arguments=param_substitutions.items())
    ld.add_action(mapviz_launch)

    return ld
