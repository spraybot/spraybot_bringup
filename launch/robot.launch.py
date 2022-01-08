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

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([current_pkg, 'rviz', 'simulation.rviz']),
        description='Full path to the RVIZ config file to use')
    ld.add_action(declare_rviz_config_file_cmd)

    declare_enable_drivers = DeclareLaunchArgument(
        'drivers',
        default_value='false',
        description='Start Husky VectorNav and Ouster nodes')
    ld.add_action(declare_enable_drivers)

    declare_enable_visualization = DeclareLaunchArgument(
        'viz',
        default_value='false',
        description='Start Rviz and Mapviz')
    ld.add_action(declare_enable_visualization)

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('viz')))
    ld.add_action(rviz_node)

    # Include other launch files
    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([current_pkg, 'launch', 'drivers.launch.py'])),
        condition=IfCondition(LaunchConfiguration('drivers')))
    ld.add_action(drivers_launch)

    localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'localization.launch.py'])))
    ld.add_action(localization_launch)

    mapviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'mapviz.launch.py'])),
            condition=IfCondition(LaunchConfiguration('viz')))
    ld.add_action(mapviz_launch)

    return ld
