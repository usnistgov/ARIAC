import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("name")

    model_path = os.path.join(
        get_package_share_directory('ariac_mobile_robot'),
        'models',
        "mobile_robot",
        'model.sdf'
    )


    # Generate Robot Description parameter from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_mobile_robot"), "urdf", "mobile_robot.urdf.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        namespace="mobile_robot",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_mobile_robot",
        arguments=["-entity", "mobile_robot", "-robot_namespace", "mobile_robot", "-file", model_path, "-x", "-4.0", "-y", "3.5", "-z", "0.0", "-Y", "3.14"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        # gazebo_spawn_robot,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("name", default_value="diffbot", description="robot_name")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])