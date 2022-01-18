from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    current_pkg = FindPackageShare('spraybot_bringup')

    # Create temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
        }

    configured_params = RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution([current_pkg, 'params', 'localization.yaml']),
                description='Full path to the ROS2 parameters file to use'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node_odom',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node_map',
            output='screen',
            parameters=[configured_params],
            remappings=[('odometry/filtered', 'odometry/filtered_map')]),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[configured_params],
            remappings=[('odometry/filtered', 'odometry/filtered_map'),
                        ('imu', 'imu/data')]),

        # Sets the GPS origin
        Node(
            package='spraybot_utils',
            executable='gps_datum.py',
            name='gps_datum',
        )
    ])
