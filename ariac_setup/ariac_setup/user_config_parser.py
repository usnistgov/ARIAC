import os

import yaml

from typing import Literal

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion


from ariac_setup.structures import (
    Sensor,
    SensorType,
    BreakBeam,
    DistanceSensor,
    Camera,
    Lidar,
    Scan,
    ParsingError
)

from ariac_setup.utils import evaluate_pi_expression


class UserConfigParser:
    def __init__(self, config_path: str):

        self._config_path = config_path
        self._competitor_name: str
        self._conveyor_speed: float
        self._cell_feed_rate: float 

        self._sensors: list[Sensor] = []

        self.parse_config()
        
    @property
    def sensors(self):
        return self._sensors

    @property
    def sensor_cost(self):
        return sum([s.cost for s in self.sensors])
    
    @property
    def competitor_name(self):
        return self._competitor_name

    @property
    def conveyor_speed(self):
        return self._conveyor_speed
    
    @property
    def cell_feed_rate(self):
        return self._cell_feed_rate
    
    def parse_config(self):
        with open(self._config_path, 'r') as file:
            data: dict = yaml.safe_load(file)

        self._competitor_name = data.get("COMPETITOR_NAME", "")
        self._conveyor_speed = data.get("CONVEYOR_SPEED", "")
        self._cell_feed_rate = data.get("CELL_FEED_RATE", "")

        self.parse_sensors(data.get("SENSORS", {}))

    def parse_sensors(self, sensors_config: list[dict]):
        self._sensors: list[Sensor] = []

        for config in sensors_config:
            name: str = config.get("NAME", "")
            update_rate: int = config.get("UPDATE_RATE", 1)
            pose = UserConfigParser._get_pose(config)

            if config.get("TYPE") == "break_beam":
                self._sensors.append(
                    BreakBeam(
                        name=name, 
                        sensor_type=SensorType.BREAKBEAM, 
                        pose=pose, 
                        update_rate=update_rate,
                    )
                )
            elif config.get("TYPE") == "distance":
                self._sensors.append(
                    DistanceSensor(
                        name=name,
                        sensor_type=SensorType.DISTANCE,
                        pose=pose, 
                        update_rate=update_rate,
                    )
                )
            elif config.get("TYPE") == "camera":
                resolution = UserConfigParser._get_resolution(config)
                fov = UserConfigParser._get_fov(config)
                self._sensors.append(
                    Camera(
                        name=name,
                        sensor_type=SensorType.CAMERA,
                        pose=pose, 
                        update_rate=update_rate,  
                        resolution=resolution, 
                        fov=fov
                    )
                )
            elif config.get("TYPE") == "lidar":
                h_scan = UserConfigParser._get_scan(config, "HORIZONTAL")
                v_scan = UserConfigParser._get_scan(config, "VERTICAL")
                lidar = Lidar(
                    name=name,
                    sensor_type=SensorType.LIDAR,
                    pose=pose,
                    update_rate=update_rate,
                    horizontal=h_scan,
                    vertical=v_scan
                )

                if not lidar.valid_location():
                    raise ParsingError(f"{lidar.name} is not inside accepted bounding boxes")
                
                if lidar.total_samples > Lidar.MAX_SAMPLES:
                    raise ParsingError(f"{lidar.name} has too man samples. Max is {Lidar.MAX_SAMPLES}")
                
                self._sensors.append(lidar)
        
    @staticmethod 
    def _get_pose(sensor_config: dict) -> Pose:
        pose: dict = sensor_config.get("POSE", {})

        x, y, z = pose.get("XYZ", (0, 0, 0))
        roll, pitch, yaw = [evaluate_pi_expression(s) for s in pose.get("RPY", (0, 0, 0))]

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    
        return Pose(position=Point(x=x, y=y, z=z),orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
    
    @staticmethod
    def _get_resolution(sensor_config: dict) -> Literal["720p", "1080p"]:
        return sensor_config.get("RESOLUTION", "720p")
    
    @staticmethod
    def _get_fov(sensor_config: dict) -> float:
        return sensor_config.get("FOV", 0.873)
    
    @staticmethod
    def _get_scan(sensor_config: dict, direction) -> Scan:
        scan_config: dict = sensor_config.get(direction, {})

        scan = Scan(
            samples=scan_config.get("SAMPLES", 1),
            min_angle=evaluate_pi_expression(scan_config.get("MIN_ANGLE", 0.1)),
            max_angle=evaluate_pi_expression(scan_config.get("MAX_ANGLE", 0.1))
        )

        if not scan.valid():
            raise ParsingError(f"{direction} scan is not valid. Min angle is larger than max angle")
        
        return scan
    
