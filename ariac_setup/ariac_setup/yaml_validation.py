import os

from typing import Any

import json
from jsonschema import validate, ValidationError

import yaml
from yaml.scanner import ScannerError
from yaml.parser import ParserError

from ament_index_python import get_package_share_directory


class YAMLValidator:
    def __init__(self, schema_path: str):
        if not os.path.exists(schema_path):
            raise ValidationError(f"Schema: {schema_path} does not exist")
        with open(schema_path, 'r') as file:
            self._schema = json.load(file)

    def validate_yaml(self, yaml_path: str):
        if not yaml_path:
            raise ValidationError("yaml path is empty")
        if not os.path.exists(yaml_path):
            raise ValidationError(f"{yaml_path} does not exist")

        with open(yaml_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
            except (ScannerError, ParserError) as e:
                raise ValidationError(f"{yaml_path} is malformed {e.problem}")
        
        validate(data, self._schema)


class TrialConfigValidator(YAMLValidator):
    def __init__(self):
        share = get_package_share_directory("ariac_setup")

        super().__init__(os.path.join(share, "config", "trial_schema.json"))

        self.defects_path = os.path.join(share, "config", "defects.yaml")

    def validate_yaml(self, yaml_path: str):
        super().validate_yaml(yaml_path)


        self._validate_defects(yaml_path)
        self._validate_challenges(yaml_path)

    def _validate_defects(self, yaml_path:str):
        with open(self.defects_path, 'r') as file:
            try:
                defect_data = yaml.safe_load(file)
            except (ScannerError, ParserError) as e:
                raise ValidationError(f"{yaml_path} is malformed {e.problem}")

        defects = defect_data.get("DEFECT_TYPES", None)
        
        if defects is None:
            raise ValidationError(f"No defects found in {self.defects_path}")
        
        defect_types = [int(d) for d in defects.keys()]
        
        with open(yaml_path, 'r') as file:
            trial: dict= yaml.safe_load(file)

        possible_defects = trial.get("POSSIBLE_DEFECTS", None)
        
        if possible_defects is None:
            return
        
        if not all(d in defect_types for d in possible_defects):
            invalid = [d for d in possible_defects if d not in defect_types]
            raise ValidationError(f'Possible Defects: {invalid} are not in defects.yaml')

    def _validate_challenges(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            data: dict = yaml.safe_load(file)

        challenges = data.get("CHALLENGES")
        if not challenges:
            return

        for challenge_type, challenge_list in challenges.items():
            if challenge_type == "VACUUM_TOOL_MALFUNCTIONS":
                continue

            for challenge in challenge_list:
                start = challenge["START_TIME"]
                duration = challenge.get("DURATION", 0)
                time_limit = data["TIME_LIMIT"]

                if start > time_limit:
                    raise ValidationError(
                        f"{challenge_type} has a start time ({start}) "
                        f"which is greater than the time limit ({time_limit})"
                    )

                if challenge_type != "HIGH_PRIORITY_ORDERS":
                    if start + duration > time_limit:
                        raise ValidationError(
                            f"{challenge_type} has a start time ({start}) and "
                            f"duration ({duration}) which would outlast the "
                            f"time limit ({time_limit})"
                        )


class UserConfigValidator(YAMLValidator):
    def __init__(self):
        share = get_package_share_directory("ariac_setup")

        super().__init__(os.path.join(share, "config", "user_config_schema.json"))

        self.defects = os.path.join(share, "config", "defects.yaml")

    def validate_yaml(self, yaml_path: str):
        super().validate_yaml(yaml_path)
        self._validate_feed_rate(yaml_path)
        self._validate_sensors(yaml_path)

    def _validate_feed_rate(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            data: dict = yaml.safe_load(file)
        
        speed = data.get("CONVEYOR_SPEED", 0)

        feed_rate = data.get("CELL_FEED_RATE", 1)

        if (feed_rate > speed * 2):
            raise ValidationError("CELL_FEED_RATE must be less than double CONVEYOR_SPEED")
        
    def _validate_sensors(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            data: dict = yaml.safe_load(file)

        sensors: list[dict] = data.get("SENSORS", {})

        valid_update_rates = {
            'break_beam': [10, 30],
            'distance': [10, 30],
            'camera': [30],
            'lidar': [10, 20]
        }

        sensor_names: list[str] = []
        
        for sensor in sensors:
            sensor_type = sensor.get("TYPE", "")
            sensor_name = sensor.get("NAME", "")

            if sensor_name in sensor_names:
                raise ValidationError("sensor name {sensor_name} is not unique")
            sensor_names.append(sensor_name)

            update_rate = sensor.get("UPDATE_RATE", 1)
            valid_options = valid_update_rates.get(sensor_type, [])
            if update_rate not in valid_options:
                raise ValidationError(
                    f"Update rate of {update_rate} is not valid for sensor type: {sensor_type}."
                    f" Valid options are {', '.join(str(o) for o in valid_options)}"
                )
            
            if sensor_type == 'lidar':
                hor = sensor.get("HORIZONTAL")
                ver = sensor.get("VERTICAL")

                if not hor:
                    raise ValidationError(f'Missing HORIZONTAL tag for lidar: {sensor_name}')
                
                if not ver:
                    raise ValidationError(f'Missing VERTICAL tag for lidar: {sensor_name}')
                
            elif sensor_type == 'camera':
                if not sensor.get('RESOLUTION'):
                    raise ValidationError(f'Resolution not set for camera: {sensor_name}')
