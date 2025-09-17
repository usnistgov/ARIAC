from dataclasses import dataclass
from pathlib import Path
from html.parser import HTMLParser
import re
import os

from typing import cast, Any
import yaml
from yaml.scanner import ScannerError
from yaml.parser import ParserError
from enum import IntEnum

from ament_index_python import get_package_share_directory

from ariac_interfaces.msg import VacuumTools

from ariac_setup.structures import (
    Sensor,
    Camera,
    Lidar
)

class ChallengeType(IntEnum):
    ConveyorMalfunction = 1
    VacuumToolMalfunction = 2
    VoltageTesterMalfunction = 3
    HighPriorityOrder = 4

    def __str__(self):
        match self:
            case ChallengeType.ConveyorMalfunction:
                return "ConveyorMalfunction"
            case ChallengeType.VacuumToolMalfunction:
                return "VacuumToolMalfunction"
            case ChallengeType.VoltageTesterMalfunction:
                return "VoltageTesterMalfunction"
            case ChallengeType.HighPriorityOrder:
                return "HighPriorityOrder"
            case _:
                return super().__str__()

@dataclass
class Challenge:
    challenge_type: ChallengeType

    def set_id(self, val):
        self.id = val

@dataclass
class ConveyorMalfunction(Challenge):
    start_time: int
    duration: int

@dataclass
class VacuumToolMalfunction(Challenge):
    tool: int
    grasp_occurrence: int


@dataclass
class VoltageTesterMalfunction(Challenge):
    start_time: int
    duration: int
    tester: int

@dataclass
class HighPriorityOrder(Challenge):
    order_id: str
    start_time: int

@dataclass
class TrialInfo:
    trial_id: str
    seed: int
    defect_rate: float
    time_limit: int
    num_kits: int
    num_modules: int
    possible_defects: list[int]
    challenges: list[Challenge]

class Trial:
    def __init__(self, yaml_path: Path|None = None):
        self.path = yaml_path
        self.info: TrialInfo

        if yaml_path is None:
            self.path = None
            self.info = TrialInfo(
                trial_id="",
                seed=0,
                defect_rate=0.15,
                time_limit=500,
                num_kits=0,
                num_modules=0,
                possible_defects=[],
                challenges=[]
            )
        else:
            self.path = yaml_path
            self.info = self.from_yaml(yaml_path)

    @staticmethod
    def from_yaml(file_path: Path) -> TrialInfo:
        with open(file_path) as f:
            yaml_dict: dict = yaml.load(f, Loader=yaml.SafeLoader)
        
        trial_id = yaml_dict["ID"]
        seed = int(float(yaml_dict["SEED"]))
        defect_rate = float(yaml_dict["DEFECT_RATE"])
        time_limit = int(float(yaml_dict["TIME_LIMIT"]))
        num_kits = int(float(yaml_dict["NUM_KITS"]))
        num_modules = int(float(yaml_dict["NUM_MODULES"]))
        possible_defects = yaml_dict["POSSIBLE_DEFECTS"] if "POSSIBLE_DEFECTS" in yaml_dict.keys() else []

        challenge_info: dict[str, list] = yaml_dict.get("CHALLENGES", {})
        challenges = []
        
        if "CONVEYOR_MALFUNCTIONS" in challenge_info:
            for conveyor_malfunction in challenge_info["CONVEYOR_MALFUNCTIONS"]:
                challenges.append(
                    ConveyorMalfunction(
                        ChallengeType.ConveyorMalfunction,
                        int(float(conveyor_malfunction["START_TIME"])),
                        int(float(conveyor_malfunction["DURATION"]))
                    )
                )

        if "VACUUM_TOOL_MALFUNCTIONS" in challenge_info:
            for vacuum_tool_malfunction in challenge_info["VACUUM_TOOL_MALFUNCTIONS"]:
                challenges.append(
                    VacuumToolMalfunction(
                        ChallengeType.VacuumToolMalfunction,
                        VacuumTools.VG_2 if int(vacuum_tool_malfunction["TOOL"])==1 else VacuumTools.VG_4,
                        int(float(vacuum_tool_malfunction["GRASP_OCCURRENCE"]))
                    )
                )

        if "VOLTAGE_TESTER_MALFUNCTIONS" in challenge_info:
            for voltage_tester_malfunction in challenge_info["VOLTAGE_TESTER_MALFUNCTIONS"]:
                challenges.append(
                    VoltageTesterMalfunction(
                        ChallengeType.VoltageTesterMalfunction,
                        int(float(voltage_tester_malfunction["START_TIME"])),
                        int(float(voltage_tester_malfunction["DURATION"])),
                        int(float(voltage_tester_malfunction["TESTER"]))
                    )
                )

        if "HIGH_PRIORITY_ORDERS" in challenge_info:
            for high_priority_order in challenge_info["HIGH_PRIORITY_ORDERS"]:
                challenges.append(
                    HighPriorityOrder(
                        ChallengeType.HighPriorityOrder,
                        high_priority_order["ID"],
                        int(float(high_priority_order["START_TIME"]))
                    )
                )
        
        return TrialInfo(
            trial_id,
            seed,
            defect_rate,
            time_limit,
            num_kits,
            num_modules,
            possible_defects,
            challenges
        )
    
    @staticmethod
    def contents(info: TrialInfo) -> str:
        trial_dict =  {
            "ID": info.trial_id,
            "SEED": int(info.seed),
            "TIME_LIMIT": int(info.time_limit),
            "NUM_KITS": int(info.num_kits),
            "NUM_MODULES": int(info.num_modules),
            "DEFECT_RATE": info.defect_rate
        }

        if len(info.possible_defects) > 0:
            trial_dict["POSSIBLE_DEFECTS"] = [int(v) for v in info.possible_defects]


        challenge_dict: dict[str, list] = {}

        for challenge in info.challenges:
            match(challenge.challenge_type):
                case ChallengeType.ConveyorMalfunction:
                    c = cast(ConveyorMalfunction, challenge)
                    challenge_dict.setdefault("CONVEYOR_MALFUNCTIONS", []).append({
                        "START_TIME": c.start_time,
                        "DURATION": c.duration
                    })
        
                case ChallengeType.VacuumToolMalfunction:
                    c = cast(VacuumToolMalfunction, challenge)
                    challenge_dict.setdefault("VACUUM_TOOL_MALFUNCTIONS", []).append({
                        "TOOL": int(c.tool),
                        "GRASP_OCCURRENCE": c.grasp_occurrence
                    })
                
                case ChallengeType.VoltageTesterMalfunction:
                    c = cast(VoltageTesterMalfunction, challenge)
                    challenge_dict.setdefault("VOLTAGE_TESTER_MALFUNCTIONS", []).append({
                        "START_TIME": c.start_time,
                        "DURATION": c.duration,
                        "TESTER": c.tester
                    })
                
                case ChallengeType.HighPriorityOrder:
                    c = cast (HighPriorityOrder, challenge)

                    challenge_dict.setdefault("HIGH_PRIORITY_ORDERS", []).append({
                        "ID": c.order_id,
                        "START_TIME": c.start_time
                    })
        
        if len(challenge_dict) > 0:
            trial_dict["CHALLENGES"] = challenge_dict
        
        return yaml.dump(trial_dict, sort_keys=False)
    
@dataclass
class UserInfo:
    name: str
    conveyor_speed: float
    cell_feed_rate: float
    sensors: list[Sensor]

    def contents(self):
        user_config_dict: dict[str, Any] = {
            "COMPETITOR_NAME": self.name,
            "CONVEYOR_SPEED": self.conveyor_speed,
            "CELL_FEED_RATE": self.cell_feed_rate,
        }

        user_config_dict["SENSORS"] = []

        for sensor in self.sensors:
            sensor_dict = {
                "NAME": sensor.name,
                "TYPE": str(sensor.sensor_type),
                "POSE": {
                    "XYZ": list(sensor.xyz),
                    "RPY": list(sensor.rpy)
                },
                "UPDATE_RATE": sensor.update_rate
            }

            if isinstance(sensor, Lidar):
                sensor_dict["HORIZONTAL"] = {
                    "SAMPLES": sensor.horizontal.samples,
                    "MIN_ANGLE": sensor.horizontal.min_angle,
                    "MAX_ANGLE": sensor.horizontal.max_angle
                }
                sensor_dict["VERTICAL"] = {
                    "SAMPLES": sensor.vertical.samples,
                    "MIN_ANGLE": sensor.vertical.min_angle,
                    "MAX_ANGLE": sensor.vertical.max_angle
                }

            if isinstance(sensor, Camera):
                sensor_dict["RESOLUTION"] = sensor.resolution
                sensor_dict["FOV"] = sensor.fov

            user_config_dict["SENSORS"].append(sensor_dict)
        
        return yaml.dump(user_config_dict, sort_keys=False)

class HighlightHTMLParser(HTMLParser):
    def __init__(self, search_term):
        super().__init__()
        self.search_term = search_term.lower()
        self.result = ""

    def handle_starttag(self, tag, attrs):
        attr_str = " ".join(f'{k}="{v}"' for k, v in attrs)
        self.result += f"<{tag} {attr_str}>" if attr_str else f"<{tag}>"

    def handle_endtag(self, tag):
        self.result += f"</{tag}>"

    def handle_data(self, data):
        highlighted = re.sub(
            f"({re.escape(self.search_term)})",
            r'<span style="font-weight: bolder;">\1</span>',
            data,
            flags=re.IGNORECASE
        )
        self.result += highlighted

    def handle_startendtag(self, tag, attrs):
        attr_str = " ".join(f'{k}="{v}"' for k, v in attrs)
        self.result += f"<{tag} {attr_str}/>" if attr_str else f"<{tag}/>"

    def get_html(self):
        return self.result