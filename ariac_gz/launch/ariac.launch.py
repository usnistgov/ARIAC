import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ariac_db.manager import DatabaseManager, DatabaseError

import sqlite3
import re
from typing import Dict, List
from pathlib import Path

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
from ariac_setup.user_config_parser import UserConfigParser, ParsingError
from ariac_setup.structures import Cheats

RECORDER_POSITIONS = {
    "inspection_recorder": [0.43, 2.08, 1.03, 0.0, 0.45, -0.82],
    "assembly_recorder": [5.36, 6.59, 1.67, 0.0, 0.71, -2.42],
    "environment_recorder": [3.63, 0.54, 4.61, 0.0, 1.15, 1.57]
}

RECORDER_RESOLUTIONS = {
    "inspection_recorder": (1080, 540),
    "assembly_recorder": (1080, 540),
    "environment_recorder": (840, 1080)
}

class SchemaValidator:
    def __init__(self, db_path: str, expected_schema_path: str | None = None):
        self.db_path = db_path
        self.expected_schema_path = expected_schema_path
        
    def get_actual_schema(self) -> Dict[str, str]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT name, sql 
            FROM sqlite_schema 
            WHERE type = 'table' 
            AND name NOT LIKE 'sqlite_%'
            ORDER BY name
        """)
        
        tables = {name: sql for name, sql in cursor.fetchall()}
        conn.close()
        
        return tables
    
    def get_expected_schema(self) -> Dict[str, str]:
        if not self.expected_schema_path:
            return {}
            
        with open(self.expected_schema_path, 'r') as f:
            content = f.read()
        
        tables = {}
        pattern = r'CREATE\s+TABLE\s+(?:IF\s+NOT\s+EXISTS\s+)?([`"\[]?\w+[`"\]]?)\s*\((.*?)\);'
        
        matches = re.finditer(pattern, content, re.IGNORECASE | re.DOTALL)
        
        for match in matches:
            table_name = match.group(1).strip('`"[]')
            full_statement = match.group(0)
            tables[table_name] = full_statement
            
        return tables
    
    def normalize_sql(self, sql: str) -> str:
        if not sql:
            return ""
        sql = re.sub(r'\s+', ' ', sql)
        sql = sql.strip()
        return sql.lower()
    
    def get_table_columns(self, table_name: str) -> List[Dict]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute(f"PRAGMA table_info({table_name})")
        columns = []
        
        for row in cursor.fetchall():
            columns.append({
                'cid': row[0],
                'name': row[1],
                'type': row[2],
                'notnull': row[3],
                'default': row[4],
                'pk': row[5]
            })
        
        conn.close()
        return columns
    
    def validate(self, verbose: bool = True) -> bool:
        actual = self.get_actual_schema()
        expected = self.get_expected_schema() if self.expected_schema_path else {}
        
        all_valid = True
        
        print("=" * 80)
        print("SCHEMA VALIDATION REPORT")
        print("=" * 80)
        
        print(f"\n✓ Found {len(actual)} tables in database:")
        for table_name in sorted(actual.keys()):
            print(f"  - {table_name}")
        
        if expected:
            print(f"\n✓ Expected {len(expected)} tables from schema file:")
            for table_name in sorted(expected.keys()):
                print(f"  - {table_name}")
            
            actual_names = set(actual.keys())
            expected_names = set(expected.keys())
            
            missing_tables = expected_names - actual_names
            extra_tables = actual_names - expected_names
            
            if missing_tables:
                print(f"\n✗ MISSING TABLES ({len(missing_tables)}):")
                for table in sorted(missing_tables):
                    print(f"  - {table}")
                all_valid = False
            else:
                print("\n✓ All expected tables present")
            
            if extra_tables:
                print(f"\n⚠ EXTRA TABLES ({len(extra_tables)}):")
                all_valid = False
                for table in sorted(extra_tables):
                    print(f"  - {table}")

            
            common_tables = actual_names & expected_names
            if common_tables and verbose:
                print(f"\n{'=' * 80}")
                print("DETAILED TABLE COMPARISON")
                print("=" * 80)
                
                for table_name in sorted(common_tables):
                    print(f"\n--- Table: {table_name} ---")
                    
                    actual_sql = self.normalize_sql(actual[table_name]) + ";"
                    expected_sql = self.normalize_sql(expected[table_name])
                    
                    if actual_sql == expected_sql:
                        print("  ✓ Schema matches exactly")
                    else:
                        print("  ⚠ Schema differs:")
                        print(f"\n  Expected:\n  {expected[table_name]}")
                        print(f"\n  Actual:\n  {actual[table_name]}")
                        
                        columns = self.get_table_columns(table_name)
                        print(f"\n  Columns ({len(columns)}):")
                        for col in columns:
                            pk = " PRIMARY KEY" if col['pk'] else ""
                            notnull = " NOT NULL" if col['notnull'] else ""
                            default = f" DEFAULT {col['default']}" if col['default'] else ""
                            print(f"    - {col['name']}: {col['type']}{pk}{notnull}{default}")
        
        else:
            print("\n⚠ No expected schema file provided - showing actual schema only")
            print("\nDetailed Table Structures:")
            print("=" * 80)
            
            for table_name in sorted(actual.keys()):
                print(f"\n--- Table: {table_name} ---")
                print(f"CREATE statement:\n{actual[table_name]}\n")
                
                columns = self.get_table_columns(table_name)
                print(f"Columns ({len(columns)}):")
                for col in columns:
                    pk = " PRIMARY KEY" if col['pk'] else ""
                    notnull = " NOT NULL" if col['notnull'] else ""
                    default = f" DEFAULT {col['default']}" if col['default'] else ""
                    print(f"  - {col['name']}: {col['type']}{pk}{notnull}{default}")
        
        print("\n" + "=" * 80)
        if all_valid and expected:
            print("✓ VALIDATION PASSED")
        elif not expected:
            print("⚠ VALIDATION INCOMPLETE (no expected schema provided)")
        else:
            print("✗ VALIDATION FAILED")
        print("=" * 80)
        
        return all_valid

def launch_setup(context, *args, **kwargs):
    trial_config = LaunchConfiguration("trial_config").perform(context)
    
    headless_val = LaunchConfiguration("headless").perform(context)
    headless = str(headless_val).lower() == "true"
    record_val = LaunchConfiguration("record").perform(context)
    record = str(record_val).lower() == "true"

    user_config = LaunchConfiguration("user_config").perform(context)
    db_path = LaunchConfiguration("db_path").perform(context)
    cheat_selection = int(LaunchConfiguration("cheat_selection").perform(context))
    log_cell_info_val = LaunchConfiguration("log_cell_info").perform(context)
    log_cell_info = str(log_cell_info_val).lower() == "true"
    gz_log_arg = LaunchConfiguration("gz_log_level").perform(context)

    if db_path != "":
        check_db_validity(Path(db_path))
    else:
        ws_path = Path(get_package_share_directory('ariac_gz')).parent.parent.parent.parent
        src_path = ws_path / "src"
        db_src_paths: list[Path] = [path for path in src_path.rglob("ariac_db")]
        shortest_path = min(db_src_paths, key=lambda p: len(p.parts))
        database_dir = shortest_path / "database" / "ariac.db"
        if not database_dir.exists():
            try:
                _ = DatabaseManager(database_dir, create=True)
            except DatabaseError as e:
                print(f"Error creating db at {database_dir}: {e.message}")
                return
        db_path = str(database_dir)
        
    team_config = validate_configs(trial_config, user_config)

    if team_config is None:
        return

    gz_args = get_gz_args(trial_config, team_config, db_path, cheat_selection, gz_log_arg, record, log_cell_info)

    if headless:
        gz_args += " -s --headless-rendering"

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

    score_logger = Node(
        package="ariac_setup",
        executable="score_logger",
        output="screen",
        arguments=['--db-path',  db_path],
        parameters=[{'use_sim_time': True}]
    )

    return [
        gz,
        gz_sim_ready,
        startup_when_ready,
        score_logger
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
        DeclareLaunchArgument("headless", default_value="false", description="Run the competition without the GUI")
    )

    declared_arguments.append(
        DeclareLaunchArgument("record", default_value="false", description="Record the environment using the recording plugin")
    )

    declared_arguments.append(
        DeclareLaunchArgument("gz_log_level", default_value="msg", description="Log level for Gazebo Options: [dbg, msg, warn, error]")
    )

    declared_arguments.append(
        DeclareLaunchArgument("cheat_selection", default_value="0", description="Decides which cheat to load in")
    )

    declared_arguments.append(
        DeclareLaunchArgument("log_cell_info", default_value="false", description="Cheat tool for logging the info about the cell when spawned")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def check_db_validity(db_path: Path):
    if not db_path.exists():
        raise FileNotFoundError(f"Database at {db_path} does not exist")
    
    schema_path = Path(get_package_share_directory("ariac_db")) / "schema" / "ariac.sql"

    schema_validator = SchemaValidator(str(db_path), str(schema_path))
    
    if not schema_validator.validate(True):
        raise DatabaseError(f"Database at {db_path} corrupted. Please delete .db file and create a new one.")

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

def get_gz_args(trial_config: str, team_config: UserConfigParser, db_path: str, cheat_selection: int, gz_log_arg: str, record, log_cell_info: bool) -> str:
    world_file = os.path.join(get_package_share_directory('ariac_gz'), 'world', 'ariac.world')
    tree = ET.parse(world_file)

    root = tree.getroot()

    world_element = root.find("world")

    if world_element is None:
        return ""

    competition_manager_plugin = root.find("./world/plugin[@name='ariac_plugins::CompetitionManagerPlugin']")
    cheat_tools_plugin = root.find("./world/plugin[@name='ariac_plugins::CheatToolsPlugin']")

    if competition_manager_plugin is not None and db_path != "":
        db_path_xml = ET.Element("db_path")
        db_path_xml.text = db_path
        competition_manager_plugin.append(db_path_xml)

    if record:
        for recorder_name, recorder_position in RECORDER_POSITIONS.items():
            include = ET.Element("include")

            recorder_type = "vertical" if RECORDER_RESOLUTIONS[recorder_name][1] > RECORDER_RESOLUTIONS[recorder_name][0] else "horizontal"

            ET.SubElement(include, "name").text = recorder_name
            ET.SubElement(include, "pose").text = " ".join([str(i) for i in recorder_position])
            ET.SubElement(include, "uri").text = f"model://recorders/{recorder_type}_recorder"

            plugin = ET.SubElement(include, "plugin", {
                "filename": "libRecordingPlugin.so",
                "name": "ariac_plugins::RecordingPlugin"
            })

            ET.SubElement(plugin, "recording_width").text = str(RECORDER_RESOLUTIONS[recorder_name][0])
            ET.SubElement(plugin, "recording_height").text = str(RECORDER_RESOLUTIONS[recorder_name][1])

            world_element.append(include)
                
    if cheat_tools_plugin is not None and (cheat_selection in Cheats or log_cell_info):
        elements: list[ET.Element]  = []
        match(cheat_selection):
            case Cheats.CELLS_IN_VOLTAGE_TESTERS:
                elements.append(ET.Element("cells_in_voltage_testers"))
            case Cheats.KIT_ON_AGV:
                elements.append(ET.Element("kit_on_agv1"))
            case Cheats.KITS_ON_AGVS:
                elements.append(ET.Element("kit_on_agv1"))
                elements.append(ET.Element("kit_on_agv2"))
                elements.append(ET.Element("kit_on_agv3"))
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
        if log_cell_info:
            elements.append(ET.Element("log_cell_info"))
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