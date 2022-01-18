import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare('spraybot_bringup')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'mapviz_config',
            default_value=PathJoinSubstitution([current_pkg, 'rviz', 'mapviz.mvc']),
            description='Full path to the Mapviz config file to use'),

        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output={'both': 'log'},
            parameters=[
                {
                    'config': LaunchConfiguration('mapviz_config'),
                    'autosave': False
                }
            ],
        ),

        Node(
            package='mapviz',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[
                {'local_xy_frame': 'map'},
                {'local_xy_navsatfix_topic': 'gps/fix/origin'},
                {'local_xy_origin': 'auto'},

                {'local_xy_origins': """[
                    {'name': 'pitt',
                        'latitude': 40.438889608527084,
                        'longitude': -79.95833630855975,
                        'altitude': 273.1324935602024,
                        'heading': 0.0}
                ]"""}
            ]
        ),

        Node(
            package='spraybot_utils',
            executable='vectornav_html_status.py',
            name='vectornav_html_status',
        )
    ])
