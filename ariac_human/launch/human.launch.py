import os
import subprocess
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):   
    pkg_share = get_package_share_directory('ariac_human')

    # Human Spawner
    urdf_path = os.path.join(get_package_share_directory('ariac_gazebo'), 'models', 'human', 'model.sdf')
    human_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name = 'human_spawner',
        arguments=[
            '-entity', 'ariac_operator',
            '-file', urdf_path,
            # '-robot_namespace', 'ariac_human',
            '-x', '-15',
            '-y', '-10',
            '-z', '0.05'
        ],
        output='screen',
    )

    # ROSbridge
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [FindPackageShare("rosbridge_server"), "/launch", "/rosbridge_websocket_launch.xml"]
        ),
    )

    # State publisher
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/human", "human.urdf.xacro"]), 
            " "
        ]
    )

    state_publisher = Node(
        package="robot_state_publisher",
        namespace="ariac_human",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'robot_description': robot_description_content},
            {"use_sim_time": True},
        ],
    )

    # Navigation Stack
    map_file = os.path.join(pkg_share, 'map', 'ariac_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'human_params.yaml')
    navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nav2_bringup"), "/launch", "/bringup_launch.py"]),
            launch_arguments={
                # 'namespace': 'ariac_human',
                # 'use_namespace': 'false',
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': params_file}.items())

    # Human Control ROS node
    human_control = Node(
        package="ariac_human",
        executable="human_control_node.py",
        parameters=[
            {"use_sim_time": True},
        ]
    )

    # Terminal command to launch agent
    cmd = pkg_share + "/agent/gradlew "

    behavior = LaunchConfiguration("human_behavior").perform(context)
    if behavior == "helpful":
        cmd += "runHelp "
    elif behavior == "antagonistic":
        cmd += "runAntag "
    elif behavior == "indifferent":
        cmd += "runIndif "
        
    cmd += "-p " + pkg_share + "/agent/"

    subprocess.Popen([cmd], shell=True, stdout=subprocess.DEVNULL)

    # RVIZ 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ariac_human"), "rviz", "navigation.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=['-d', rviz_config_file]
    )

    start_nav = RegisterEventHandler(
        OnProcessExit(
            target_action=human_spawner,
            on_exit=[
                navigation_bringup,
                human_control,
                # rviz,
                ]
        )
    )


    nodes_to_start = [
        state_publisher,
        human_spawner,
        start_nav,
        rosbridge,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("human_behavior", default_value="", description="human behavior type")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
