import os
import yaml
import rclpy.logging
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
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


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

    trial_name = LaunchConfiguration("trial_name").perform(context)
    trial_config_path = os.path.join(pkg_share, 'config', 'trials', trial_name + ".yaml")

    if not os.path.exists(trial_config_path):
        rclpy.logging.get_logger('Launch File').fatal(
            f"Trial configuration '{trial_name}' not found in {pkg_share}/config/trials/")
        exit()

    try:
        competitor_pkg_share = get_package_share_directory(LaunchConfiguration("competitor_pkg").perform(context))
    except PackageNotFoundError:
        rclpy.logging.get_logger('Launch File').fatal("Competitor package not found")
        exit()

    sensor_config = LaunchConfiguration("sensor_config").perform(context)
    user_config_path = os.path.join(competitor_pkg_share, 'config', sensor_config + ".yaml")

    if not os.path.exists(user_config_path):
        rclpy.logging.get_logger('Launch File').fatal(
            f"Sensor configuration '{sensor_config}.yaml' not found in {competitor_pkg_share}/config/")
        exit()

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
        arguments=[user_config_path],
        parameters=[
            {"use_sim_time": True},
        ]
    )

    # Objects TF
    object_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='object_tf_broadcaster.py',
        output='screen',
        parameters=[
            {"use_sim_time": True},
        ]
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
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'robot_description': robot_description_content},
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

    # Human
    with open(trial_config_path, "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to read configuration file")
            config = None

    human_behavior = ""
    trial_has_human_challenge = 'false'
    if config is not None:
        try:
            challenges = config["challenges"]
            if not challenges:
                pass

            for challenge in challenges:
                for key, value in challenge.items():
                    if key == 'human':
                        human_behavior = value['behavior']
                        trial_has_human_challenge = 'true'
        except KeyError:
            pass

    human = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_human"), "/launch", "/human.launch.py"]
        ),
        launch_arguments={
            'human_behavior': human_behavior,
        }.items(),
        condition=IfCondition(trial_has_human_challenge)
    )
    nodes_to_start = [
        gazebo,
        sensor_tf_broadcaster,
        object_tf_broadcaster,
        environment_startup,
        robot_controller_switcher,
        robot_state_publisher,
        *controller_spawner_nodes,
        human
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("trial_name", default_value="kitting", description="name of trial")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "competitor_pkg", default_value="test_competitor",
            description="name of competitor package"))

    declared_arguments.append(
        DeclareLaunchArgument("sensor_config", default_value="sensors", description="name of user configuration file")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
