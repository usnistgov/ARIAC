import os

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    gz_log_arg = LaunchConfiguration("gz_log_level").perform(context)

    world = os.path.join(get_package_share_directory('ariac_gz'), 'world', 'ariac.world')

    gui_config = os.path.join(get_package_share_directory('ariac_gz'), 'config', 'gui.config')

    gz_log_levels ={"error": 1, "warn": 2, "msg": 3, "dbg": 4}

    if (gz_log_arg in gz_log_levels):
        gz_log_level = gz_log_levels[gz_log_arg]
    else:
        gz_log_level = 3

    gz_args = f'-r --verbose {gz_log_level} --gui-config {gui_config} {world}'

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [gz_args]), ('on_exit_shutdown', 'true')]
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )

    return [
        gz,
        gz_sim_bridge,
    ]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("gz_log_level", default_value="msg", description="Log level for Gazebo Options: [dbg, msg, warn, error]")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])