from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('spraybot_bringup')
    husky_control_pkg = FindPackageShare('husky_control')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([current_pkg, 'rviz', 'simulation.rviz']),
        description='Full path to the RVIZ config file to use')
    ld.add_action(declare_rviz_config_file)

    declare_world_path = DeclareLaunchArgument(
        'world_path', default_value='',
        description='The world path, by default is empty.world')
    ld.add_action(declare_world_path)

    # prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('husky_control'), 'config', 'control.yaml']
    )

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

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose',
             LaunchConfiguration('world_path')],
        output='screen',
    )
    ld.add_action(gzserver)

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
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
                   'robot_description'],
        output='screen',
    )
    ld.add_action(spawn_robot)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')])

    ld.add_action(rviz_node)

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
            launch_arguments={'use_sim_time': use_sim_time}.items())
        ])
    ld.add_action(localization_launch)

    return ld
