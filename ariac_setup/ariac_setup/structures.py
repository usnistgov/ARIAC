from typing import Tuple, Literal
from dataclasses import dataclass
from enum import IntEnum

import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose, Point

from tf_transformations import euler_from_quaternion

class ParsingError(Exception):
    """Custom exception for sensors"""
    def __init__(self, message):
        super().__init__(message)
        self.message = message

@dataclass
class BoundingBox:
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]

    def contains(self, p: Point) -> bool:
        return (
            self.center[0] - self.size[0] / 2 <= p.x <= self.center[0] + self.size[0] / 2 and
            self.center[1] - self.size[1] / 2 <= p.y <= self.center[1] + self.size[1] / 2 and
            self.center[2] - self.size[2] / 2 <= p.z <= self.center[2] + self.size[2] / 2
        )

@dataclass
class Scan:
    samples: int
    min_angle: float
    max_angle: float

    def valid(self) -> bool:
        return self.max_angle >= self.min_angle
    
class SensorType(IntEnum):
    BREAKBEAM = 1
    DISTANCE = 2
    CAMERA = 3
    LIDAR = 4

    def __str__(self):
        match self:
            case SensorType.BREAKBEAM:
                return "break_beam"
            case SensorType.DISTANCE:
                return "distance"
            case SensorType.CAMERA:
                return "camera"
            case SensorType.LIDAR:
                return "lidar"
            case _:
                return super().__str__()
    
    @property
    def topics(self) -> list[str]:
        match self:
            case SensorType.BREAKBEAM:
                return ["status", "change"]
            case SensorType.DISTANCE:
                return ["distance"]
            case SensorType.CAMERA:
                return ["image", "info"]
            case SensorType.LIDAR:
                return ["scan"]


class SensorGrade(IntEnum):
    GRADE_A = 1
    GRADE_B = 2
    GRADE_C = 3
    UNKNOWN = -1

    def __str__(self):
        match self:
            case SensorGrade.GRADE_A:
                return "A"
            case SensorGrade.GRADE_B:
                return "B"
            case SensorGrade.GRADE_C:
                return "C"
            case SensorGrade.UNKNOWN:
                return "UNKNOWN"
            case _:
                return super().__str__()


@dataclass
class Sensor:
    name: str
    sensor_type: SensorType
    pose: Pose
    update_rate: int
    
    @property
    def cost(self) -> int:
        raise NotImplementedError
    
    @property
    def grade(self) -> SensorGrade:
        raise NotImplementedError

    @property
    def xyz(self) -> tuple[float, float, float]:
        return self.pose.position.x, self.pose.position.y, self.pose.position.z
    
    @property
    def rpy(self) -> tuple[float, float, float]:
        return euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
    
    @property 
    def topics(self):
        return [f'/{self.name}/{t}' for t in self.sensor_type.topics]
    
    def get_xml(self, _: str) -> str:
        raise NotImplementedError

    @staticmethod
    def _find_required(parent: ET.Element, tag: str):
        elem = parent.find(tag)
        if elem is None:
            raise ParsingError(f"Error finding tag {tag}")
        return elem
    
    def _modify_xml(self, sdf_path) -> ET.Element:        
        try:
            with open(sdf_path, 'r') as file:
                xml = file.read()
        except IOError:
            raise ParsingError("Unable to open sensor sdf")

        try:
            root = ET.fromstring(xml)
        except ET.ParseError:
            raise ParsingError("Error parsing sensor sdf as XML")

        # Traverse to <sensor> tag
        element = root
        for tag in ['model', 'link', 'sensor']:
            element = self._find_required(element, tag)

        # Set topic
        self._find_required(element, 'topic').text = f'{self.name}_gz_topic'

        # Set update rate
        self._find_required(element, 'update_rate').text = f'{self.update_rate}'

        # Plugin tag and nested sensor name
        plugin = self._find_required(element, 'plugin')
        self._find_required(plugin, 'sensor_name').text = self.name

        return root

@dataclass
class BreakBeam(Sensor):
    @property
    def grade(self) -> SensorGrade:
        match self.update_rate:
            case 10:
                return SensorGrade.GRADE_B
            case 30:
                return SensorGrade.GRADE_A
            case _:
                return SensorGrade.UNKNOWN
    
    @property
    def cost(self) -> int:
        match self.grade:
            case SensorGrade.GRADE_A:
                return 400
            case SensorGrade.GRADE_B:
                return 200
            case _:
                raise ValueError(f'{self.name} has an undefined grade')
        
    def get_xml(self, sdf_path):
        root = super()._modify_xml(sdf_path)

        # Traverse to <plugin> tag
        element = root
        for tag in ['model', 'link', 'sensor', 'plugin']:
            element = self._find_required(element, tag)

        # Set topic
        self._find_required(element, 'sensor_type').text = "break_beam"

        return ET.tostring(root, encoding="unicode")

@dataclass
class DistanceSensor(Sensor):
    @property
    def grade(self) -> SensorGrade:
        match self.update_rate:
            case 10:
                return SensorGrade.GRADE_B
            case 30:
                return SensorGrade.GRADE_A
            case _:
                return SensorGrade.UNKNOWN
    
    @property
    def cost(self) -> int:
        match self.grade:
            case SensorGrade.GRADE_A:
                return 600
            case SensorGrade.GRADE_B:
                return 300
            case _:
                raise ValueError(f'{self.name} has an undefined grade')
        
    def get_xml(self, sdf_path):
        root = super()._modify_xml(sdf_path)

        # Traverse to <plugin> tag
        element = root
        for tag in ['model', 'link', 'sensor', 'plugin']:
            element = self._find_required(element, tag)

        # Set topic
        self._find_required(element, 'sensor_type').text = "distance"

        return ET.tostring(root, encoding="unicode")

@dataclass
class Camera(Sensor):
    resolution: Literal["720p", "1080p"]
    fov: float

    @property
    def grade(self) -> SensorGrade:
        match self.resolution:
            case "720p":
                return SensorGrade.GRADE_B
            case "1080p":
                return SensorGrade.GRADE_A
            case _:
                return SensorGrade.UNKNOWN
    
    @property
    def cost(self) -> int:
        match self.grade:
            case SensorGrade.GRADE_A:
                return 800
            case SensorGrade.GRADE_B:
                return 500
            case _:
                raise ValueError(f'{self.name} has an undefined grade')
    
    def _width(self) -> str:
        return "1280" if self.resolution == "720p" else "1920"
    
    def _height(self) -> str:
        return self.resolution.removesuffix('p')
    
    def get_xml(self, sdf_path):
        root = super()._modify_xml(sdf_path)

        # Set scan params
        element = root
        for tag in ['model', 'link', 'sensor', 'camera']:
            element = self._find_required(element, tag)

        self._find_required(element, 'horizontal_fov').text = str(self.fov)
        self._find_required(element, 'camera_info_topic').text = f'{self.name}_gz_info_topic'
            
        # Set image parameters
        image = self._find_required(element, 'image')
        self._find_required(image, 'width').text = self._width()
        self._find_required(image, 'height').text = self._height()

        return ET.tostring(root, encoding="unicode")

@dataclass
class Lidar(Sensor):
    horizontal: Scan
    vertical: Scan
    LIDAR_BBOX1 = BoundingBox(center=(0.8, 0.825, 0.52), size=(0.6, 0.25, 0.2))
    LIDAR_BBOX2 = BoundingBox(center=(0.8, 1.175, 0.52), size=(0.6, 0.25, 0.2))
    LIDAR_BBOX3 = BoundingBox(center=(0.8, 1.0, 0.57), size=(0.6, 0.1, 0.1))
    MAX_SAMPLES = 400

    @property
    def grade(self) -> SensorGrade:
        if self.update_rate == 20:
            return SensorGrade.GRADE_A
        elif self.total_samples >= 200:
            return SensorGrade.GRADE_B
        else:
            return SensorGrade.GRADE_C
    
    @property
    def cost(self) -> int:
        match self.grade:
            case SensorGrade.GRADE_A:
                return 1500
            case SensorGrade.GRADE_B:
                return 1250
            case SensorGrade.GRADE_C:
                return 1000
            case _:
                raise ValueError(f'{self.name} has an undefined grade')

    @property
    def total_samples(self) -> int:
        return self.horizontal.samples * self.vertical.samples
    
    def valid_location(self):
        return (
            Lidar.LIDAR_BBOX1.contains(self.pose.position) or
            Lidar.LIDAR_BBOX2.contains(self.pose.position) or
            Lidar.LIDAR_BBOX3.contains(self.pose.position)
        )
        
    def get_xml(self, sdf_path):
        root = super()._modify_xml(sdf_path)

        # Traverse to <plugin> tag
        element = root
        for tag in ['model', 'link', 'sensor', 'plugin']:
            element = self._find_required(element, tag)

        # Set topic
        self._find_required(element, 'sensor_type').text = "lidar"

        # Set scan params
        element = root
        for tag in ['model', 'link', 'sensor', 'lidar', 'scan']:
            element = self._find_required(element, tag)
            
        # Set horizontal scan parameters
        horizontal = self._find_required(element, 'horizontal')
        self._find_required(horizontal, 'samples').text = str(self.horizontal.samples)
        self._find_required(horizontal, 'min_angle').text = str(self.horizontal.min_angle)
        self._find_required(horizontal, 'max_angle').text = str(self.horizontal.max_angle)

        # Set vertical scan parameters
        vertical = self._find_required(element, 'vertical')
        self._find_required(vertical, 'samples').text = str(self.vertical.samples)
        self._find_required(vertical, 'min_angle').text = str(self.vertical.min_angle)
        self._find_required(vertical, 'max_angle').text = str(self.vertical.max_angle)

        return ET.tostring(root, encoding="unicode")

class Cheats(IntEnum):
    CELLS_IN_VOLTAGE_TESTERS=1
    KIT_ON_AGV=2
    KITS_ON_AGVS=3
    HIGH_PRIORITY_KIT=4
    PARTIAL_MODULE=5
    MODULE=6
    FLIPPED_MODULE=7
    MODULE_WITH_WELDS=8