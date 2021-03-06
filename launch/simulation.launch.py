from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('spraybot_bringup')
    spraybot_simulation_pkg = FindPackageShare('spraybot_simulation')
    husky_control_pkg = FindPackageShare('husky_control')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([current_pkg, 'rviz', 'simulation.rviz']),
        description='Full path to the Rviz config file to use')
    ld.add_action(declare_rviz_config_file)

    declare_enable_gzclient = DeclareLaunchArgument(
        'gzclient',
        default_value='true',
        description='Start Gazebo GUI')
    ld.add_action(declare_enable_gzclient)

    declare_enable_mapviz = DeclareLaunchArgument(
        'viz',
        default_value='true',
        description='Start visualization using Rviz and Mapviz')
    ld.add_action(declare_enable_mapviz)

    declare_enable_mapviz = DeclareLaunchArgument(
        'mapviz',
        default_value='false',
        description='Start Mapviz')
    ld.add_action(declare_enable_mapviz)

    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=PathJoinSubstitution([spraybot_simulation_pkg, 'worlds', 'spraybot.world']),
        description='The world path, by default is empty.world')
    ld.add_action(declare_world_path)

    # prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = RewrittenYaml(
            source_file=PathJoinSubstitution([FindPackageShare('husky_control'), 'config', 'control.yaml']),
            root_key='',
            param_rewrites={
              'use_sim_time': use_sim_time,  
            })

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('husky_description'), 'urdf', 'husky.urdf.xacro']
            ),
            ' ',
            'name:=husky',
            ' ',
            'prefix:=''',
            ' ',
            'is_sim:=true',
            ' ',
            'gazebo_controllers:=',
            config_husky_velocity_controller,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Nodes
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )
    ld.add_action(node_robot_state_publisher)

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    ld.add_action(spawn_joint_state_broadcaster)

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    ld.add_action(diffdrive_controller_spawn_callback)

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

    # Gazebo server
    gzserver_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gzserver.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world_path'),
                'verbose': 'true',
                'server_required': 'true',
                'minimal_comms': 'true',  # Disable if causes issues
                'extra_gazebo_args': [
                    '--ros-args --params-file ',
                    PathJoinSubstitution([current_pkg, 'params', 'gazebo.yaml'])
                ]
            }.items()
        )
    ])
    ld.add_action(gzserver_launch)

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gzclient')),
    )
    ld.add_action(gzclient)

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity',
                   'husky',
                   '-topic',
                   'robot_description',
                   '-x', '0.0', '-y', '0.0', '-Y', '3.14'],
        output='screen',
    )
    ld.add_action(spawn_robot)

    # Include other launch files
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_control_pkg, 'launch', 'teleop_launch.py'])))
    ld.add_action(teleop_launch)

    # Wrap in GroupAction for scoping arguments (https://answers.ros.org/question/377776)
    localization_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'localization.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'sim_params_file': PathJoinSubstitution([current_pkg, 'params', 'simulation.yaml'])
                }.items())
        ])
    ld.add_action(localization_launch)

    # behavior_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution([current_pkg, 'launch', 'behavior.launch.py'])),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time
    #             }.items())
    # ld.add_action(behavior_launch)

    visualization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([current_pkg, 'launch', 'visualization.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz_config': LaunchConfiguration('rviz_config'),
                'mapviz': LaunchConfiguration('mapviz')
                }.items(),
            condition=IfCondition(LaunchConfiguration('viz')))
    ld.add_action(visualization_launch)

    return ld
