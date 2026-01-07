import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo
)

from launch.event_handlers import (
    OnProcessExit,
)


from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    
    urdf = os.path.join(get_package_share_directory('ariac_description'), 'urdf', f'{robot_name}.urdf.xacro')
    
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')

    robot_description = {"robot_description": robot_description_content}

    # GZ nodes
    gz_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name", robot_name
        ]
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    return [
        gz_spawner_node,
        robot_state_publisher_node
    ]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("robot_name", default_value="inspection_robot_1", description="Robot to be spawned in")
    )

    declared_arguments.append(
        DeclareLaunchArgument("start_gripper_controller", default_value="true", description="Whether or not to start gripper controller")
    )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])