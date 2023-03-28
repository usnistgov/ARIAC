import os
import yaml

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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

def generate_parameters():
    parameters = []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
            " "
        ]
    )

    parameters.append({"robot_description": robot_description_content})
    
    parameters.append({"robot_description_semantic": load_file("ariac_moveit_config", "srdf/ariac_robots.srdf")})

    parameters.append({"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")})

    parameters.append({"use_sim_time": True})

    return parameters

