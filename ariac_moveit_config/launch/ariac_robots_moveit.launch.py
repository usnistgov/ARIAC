import os

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    urdf = os.path.join(get_package_share_directory("ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )    

    nodes_to_start = [move_group_node]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])