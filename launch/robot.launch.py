from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('spraybot_bringup')

    # Declare the launch arguments
    declare_enable_drivers = DeclareLaunchArgument(
        'drivers',
        default_value='false',
        description='Start Husky, VectorNav and Ouster nodes')
    ld.add_action(declare_enable_drivers)

    declare_enable_visualization = DeclareLaunchArgument(
        'viz',
        default_value='false',
        description='Start Rviz and Mapviz')
    ld.add_action(declare_enable_visualization)

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

    visualization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'visualization.launch.py'])),
            condition=IfCondition(LaunchConfiguration('viz')))
    ld.add_action(visualization_launch)

    # Waypoint teleop
    node_waypoint_teleop = Node(
        package='spraybot_utils',
        executable='waypoint_teleop.py',
        output='screen',
        parameters=[{
            'waypoint_topic': '/goal_pose',
            'axis_scaling_factor': 0.25
        }],
    )
    ld.add_action(node_waypoint_teleop)

    return ld
