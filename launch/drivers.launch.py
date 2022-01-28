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
    husky_base_pkg = FindPackageShare('husky_base')
    husky_control_pkg = FindPackageShare('husky_control')
    vectornav_pkg = FindPackageShare('vectornav')
    ouster_pkg = FindPackageShare('ros2_ouster')

    lifecycle_nodes_sensors = ['vectornav', 'ouster_driver']

    # Declare the launch arguments
    declare_enable_visualization = DeclareLaunchArgument(
        'sensors', default_value='true', description='Start VectorNav and Ouster nodes'
    )
    ld.add_action(declare_enable_visualization)

    vectornav_params = RewrittenYaml(
        source_file=PathJoinSubstitution([vectornav_pkg, 'config', 'vectornav.yaml']),
        root_key='',
        param_rewrites={'autostart': 'false'},
        convert_types=True,
    )

    ouster_params = RewrittenYaml(
        source_file=PathJoinSubstitution([ouster_pkg, 'params', 'driver_config.yaml']),
        root_key='',
        param_rewrites={
            'autostart': 'false',
            'publish_tf': 'false'
            },
        convert_types=True,
    )

    vectornav_node = LifecycleNode(
        package='vectornav',
        executable='vectornav',
        name='vectornav',
        namespace='/',
        output='screen',
        parameters=[vectornav_params],
    )
    ld.add_action(vectornav_node)

    vectornav_sensor_msgs_node = Node(
        package='vectornav',
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[vectornav_params],
        remappings=[
            ('vectornav/imu', 'imu/data'),
            ('vectornav/gnss/ins', 'gps/fix'),
            ('vectornav/gnss/raw', 'gps/fix/raw'),
        ],
    )
    ld.add_action(vectornav_sensor_msgs_node)

    ouster_node = LifecycleNode(
        package='ros2_ouster',
        executable='ouster_driver',
        name='ouster_driver',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[ouster_params],
    )
    ld.add_action(ouster_node)

    # TODO: Create custom lifecycle manger with bond and auto-restart support
    lifecycle_manager_sensors = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sensors',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes_sensors, 'bond_timeout': 0.0},
        ],
        condition=IfCondition(LaunchConfiguration('sensors')),
    )
    ld.add_action(lifecycle_manager_sensors)

    # Include other launch files
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_base_pkg, 'launch', 'base_launch.py'])
        )
    )
    ld.add_action(base_launch)

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_control_pkg, 'launch', 'teleop_launch.py'])
        )
    )
    ld.add_action(teleop_launch)

    return ld
