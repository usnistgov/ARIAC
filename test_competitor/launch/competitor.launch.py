import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
            " "
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    
    ## Moveit Parameters
    robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/ariac_robots.srdf")}

    robot_description_kinematics = {"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    test_competitor = Node(
        package="test_competitor",
        executable="competitor",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("test_competitor"), "rviz", "test_competitor.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
        condition=IfCondition(start_rviz)
    )

    nodes_to_start = [
        test_competitor,
        rviz_node
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="false", description="start rviz node?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
