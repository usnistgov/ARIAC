import os
import tempfile
import xml.etree.ElementTree as ET

import asyncio

from jsonschema import ValidationError

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler
)

from launch.event_handlers import OnProcessExit

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from ariac_setup.yaml_validation import TrialConfigValidator, UserConfigValidator
from ariac_setup.utils import ROSAsyncAdapter
from ariac_setup.user_config_parser import UserConfigParser, ParsingError
from ariac_setup.structures import Cheats

def launch_setup(context, *args, **kwargs):

    trial_config = LaunchConfiguration("trial_config").perform(context)
    user_config = LaunchConfiguration("user_config").perform(context)
    db_path = LaunchConfiguration("db_path").perform(context)
    cheat_selection = int(LaunchConfiguration("cheat_selection").perform(context))
    gz_log_arg = LaunchConfiguration("gz_log_level").perform(context)

    team_config = validate_configs(trial_config, user_config)

    if team_config is None:
        return

    gz_args = get_gz_args(trial_config, team_config, db_path, cheat_selection, gz_log_arg)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [gz_args]), ('on_exit_shutdown', 'true')]
    )

    gz_sim_ready = Node(
        package="ariac_setup",
        executable="ready"
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )

    startup = Node(
        package="ariac_setup",
        executable="startup",
        output="screen",
        arguments=['--user-config-path',  LaunchConfiguration("user_config")],
    )

    inspection_robot_1 =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ariac_gz"), "launch", "robot.launch.py")]
        ),
        launch_arguments=[
            ("robot_name", "inspection_robot_1"),
        ]
    )

    inspection_robot_2 =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ariac_gz"), "launch", "robot.launch.py")]
        ),
        launch_arguments=[
            ("robot_name", "inspection_robot_2"),
        ]
    )

    assembly_robot_1 =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ariac_gz"), "launch", "robot.launch.py")]
        ),
        launch_arguments=[
            ("robot_name", "assembly_robot_1"),
        ]
    )

    assembly_robot_2 =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ariac_gz"), "launch", "robot.launch.py")]
        ),
        launch_arguments=[
            ("robot_name", "assembly_robot_2"),
            ("start_gripper_controller", "false")
        ]
    )

    gantry_welder =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ariac_gz"), "launch", "robot.launch.py")]
        ),
        launch_arguments=[
            ("robot_name", "gantry_welder"),
            ("start_gripper_controller", "false")
        ]
    )

    startup_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_sim_ready,
            on_exit=[
                gz_sim_bridge,
                startup,
                inspection_robot_1,
                inspection_robot_2,
                assembly_robot_1,
                assembly_robot_2,
                gantry_welder
            ]
        )
    )

    return [
        gz,
        gz_sim_ready,
        startup_when_ready,
    ]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("trial_config", default_value="", description="Path to a trial")
    )

    declared_arguments.append(
        DeclareLaunchArgument("user_config", default_value="", description="Path to a user config")
    )

    declared_arguments.append(
        DeclareLaunchArgument("db_path", default_value="", description="Path to the database")
    )

    declared_arguments.append(
        DeclareLaunchArgument("gz_log_level", default_value="msg", description="Log level for Gazebo Options: [dbg, msg, warn, error]")
    )

    declared_arguments.append(
        DeclareLaunchArgument("cheat_selection", default_value="0", description="Decides which cheat to load in")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def print_error(error: str) -> None:
    RED = "\033[91m"
    RESET = "\033[0m"
    print(f"{RED}{error}{RESET}")

def validate_configs(trial_config, user_config) -> UserConfigParser | None:
    try:
        TrialConfigValidator().validate_yaml(trial_config)
    except ValidationError as e:
        print_error(f'Error in trial config: {e.message}')
        return None
    
    try:
        UserConfigValidator().validate_yaml(user_config)
    except ValidationError as e:
        print_error(f'Error in user config: {e.message}')
        return None
    
    try:
        user_config = UserConfigParser(user_config)
    except ParsingError as e:
        print(e.message)
    
    return user_config

def get_gz_args(trial_config: str, team_config: UserConfigParser, db_path: str, cheat_selection: int, gz_log_arg: str) -> str:
    world = os.path.join(get_package_share_directory('ariac_gz'), 'world', 'ariac.world')
    tree = ET.parse(world)

    root = tree.getroot()

    competition_manager_plugin = root.find("./world/plugin[@name='ariac_plugins::CompetitionManagerPlugin']")
    cheat_tools_plugin = root.find("./world/plugin[@name='ariac_plugins::CheatToolsPlugin']")

    if competition_manager_plugin is not None and db_path != "":
        db_path_xml = ET.Element("db_path")
        db_path_xml.text = db_path
        competition_manager_plugin.append(db_path_xml)
    
    if cheat_tools_plugin is not None and cheat_selection in Cheats:
        elements: list[ET.Element]  = []
        match(cheat_selection):
            case Cheats.CELLS_IN_VOLTAGE_TESTERS:
                elements.append(ET.Element("cells_in_voltage_testers"))
            case Cheats.KIT_ON_AGV:
                elements.append(ET.Element("kit_on_agv1"))
            case Cheats.KITS_ON_AGVS:
                elements.append(ET.Element("kit_on_agv1"))
                elements.append(ET.Element("kit_on_agv2"))
            case Cheats.HIGH_PRIORITY_KIT:
                elements.append(ET.Element("kit_on_agv1"))
                elements.append(ET.Element("agv1_high_priority"))
            case Cheats.PARTIAL_MODULE:
                elements.append(ET.Element("partial_module"))
            case Cheats.MODULE:
                elements.append(ET.Element("module"))
            case Cheats.FLIPPED_MODULE:
                elements.append(ET.Element("flipped_module"))
            case Cheats.MODULE_WITH_WELDS:
                elements.append(ET.Element("module"))
                elements.append(ET.Element("module_has_welds"))
        for e in elements:
            e.text = "true"
        cheat_tools_plugin.extend(elements)
    
    for element in root.iter('trial_config_file'):
        element.text = trial_config
    
    for element in root.iter('db_path'):
        element.text = db_path

    for element in root.iter('competitor_name'):
        element.text = team_config.competitor_name

    for element in root.iter('sensor_cost'):
        element.text = str(team_config.sensor_cost)

    _, path = tempfile.mkstemp(suffix=".world")
    
    tree.write(path, xml_declaration=True, encoding="utf-8")

    gui_config = os.path.join(get_package_share_directory('ariac_gz'), 'config', 'gui.config')

    gz_log_levels ={"error": 1, "warn": 2, "msg": 3, "dbg": 4}

    if (gz_log_arg in gz_log_levels):
        gz_log_level = gz_log_levels[gz_log_arg]
    else:
        gz_log_level = 3

    return f'-r --verbose {gz_log_level} --gui-config {gui_config} {path}'
