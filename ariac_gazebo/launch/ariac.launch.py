import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='ariac_gazebo').find('ariac_gazebo')
    
    # Set the path to the world file
    world_file_name = 'ariac.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
            " "
        ]
    )

    trial_config_name = LaunchConfiguration("trial_config").perform(context)
    trial_config_path = os.path.join(pkg_share, 'config', 'trial_configuration', trial_config_name)

    user_config_name = LaunchConfiguration("user_config").perform(context)
    user_config_path = os.path.join(pkg_share, 'config', 'user_configuration', user_config_name)

    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            'world': world_path,
            }.items()
    )

    # Sensor TF
    sensor_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='sensor_tf_broadcaster.py',
        output='screen',
        arguments=[user_config_path]
    )

    # Objects TF
    object_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='object_tf_broadcaster.py',
        output='screen',
        arguments=[],
    )

    # Environment Startup
    environment_startup = Node(
        package='ariac_gazebo',
        executable='environment_startup_node.py',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'trial_config_path': trial_config_path},
            {'user_config_path': user_config_path},
            {"use_sim_time": True},
        ],
    )

    # Robot Controller Switcher
    robot_controller_switcher = Node(
        package='ariac_gazebo',
        executable='robot_controller_switcher_node.py',
        output='screen',
    )

    # Robot State Publisher
    robbot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'robot_description':robot_description_content},
            {"use_sim_time": True},
        ],
    )

    # Robot Controller Spawners
    controller_names = [
        'joint_state_broadcaster',
        'floor_robot_controller',
        'ceiling_robot_controller',
        'linear_rail_controller',
        'gantry_controller',
        'agv1_controller',
        'agv2_controller',
        'agv3_controller',
        'agv4_controller',
    ]

    controller_spawner_nodes = []
    for controller in controller_names:
        if controller == 'joint_state_broadcaster' or controller.count('agv') > 0:
            args = [controller]
        else:
            args = [controller, '--stopped']

        controller_spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                name=controller + "_spawner",
                arguments=args,
                parameters=[
                    {"use_sim_time": True},
                ],
            )
        )

    nodes_to_start = [
        gazebo,
        sensor_tf_broadcaster,
        object_tf_broadcaster,
        environment_startup,
        robot_controller_switcher,
        robbot_state_publisher,
        *controller_spawner_nodes,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("trial_config", default_value="sample.yaml", description="trial_configuration")
    )

    declared_arguments.append(
        DeclareLaunchArgument("user_config", default_value="sample.yaml", description="user_configuration")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])