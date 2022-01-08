from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    husky_base_pkg = FindPackageShare('husky_base')
    husky_control_pkg = FindPackageShare('husky_control')
    vectornav_pkg = FindPackageShare('vectornav')
    # ouster_pkg = FindPackageShare('ros2_ouster')

    # Declare the launch arguments
    declare_enable_visualization = DeclareLaunchArgument(
        'sensors',
        default_value='true',
        description='Start VectorNav and Ouster nodes')
    ld.add_action(declare_enable_visualization)

    # Include other launch files
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_base_pkg, 'launch', 'base_launch.py'])))
    ld.add_action(base_launch)

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_control_pkg, 'launch', 'teleop_launch.py'])))
    ld.add_action(teleop_launch)

    vectornav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vectornav_pkg, 'launch', 'vectornav.launch.py'])),
            condition=IfCondition(LaunchConfiguration('sensors')))
    ld.add_action(vectornav_launch)

    # To be added after integrating Ouster hardware
    # ouster_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution([ouster_pkg, 'launch', 'driver_launch.py'])),
    #         condition=IfCondition(LaunchConfiguration('sensors')))
    # ld.add_action(ouster_launch)

    return ld
