#!/usr/bin/env python3

import os

import xml.etree.ElementTree as ET

from ariac_gazebo.utilities import pose_info

from ament_index_python.packages import get_package_share_directory

class SpawnParams:
    def __init__(self, name, file_path=None, xyz=[0,0,0], rpy=[0,0,0], ns='', rf=''):
        self.name = name
        self.xml = ""
        self.file_path = file_path
        self.initial_pose = pose_info(xyz, rpy)
        self.robot_namespace = ns
        self.reference_frame = rf
    
    def get_sdf(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError:
            return ''
        
        return entity_xml


class RobotSpawnParams(SpawnParams):
    def __init__(self, name, urdf, xyz=[0,0,0], rpy=[0,0,0]):
        super().__init__(name, xyz=xyz, rpy=rpy)

        self.xml = urdf


class SensorSpawnParams(SpawnParams):
    def __init__(self, name, sensor_type, visualize, xyz=[0,0,0], rpy=[0,0,0]):
        file_path = os.path.join(get_package_share_directory('ariac_sensors'), 
            'models', sensor_type, 'model.sdf')
        
        super().__init__(name=name, file_path=file_path, xyz=xyz, rpy=rpy)

        self.visualize = visualize
        self.sensor_type = sensor_type

        self.modify_sdf()
    
    def modify_sdf(self):
        xml = ET.fromstring(self.get_sdf(self.file_path))

        xml.find('model').find('link').find('sensor').find("visualize").text = str(self.visualize)

        ray_sensors = ["break_beam", "proximity", "laser_profiler", "lidar"]
        if self.sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(self.name + "_ros_plugin"))
            plugin.find('sensor_name').text = self.name
            plugin.find('frame_name').text = self.name + "_frame"
        
        cameras = ['rgb_camera', 'rgbd_camera', 'basic_logical_camera', 'advanced_logical_camera']
        if self.sensor_type in cameras:
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(self.name + "_ros_plugin"))
            plugin.find('camera_name').text = self.name
            plugin.find('frame_name').text = self.name + "_frame"

        # scoring_sensors = ['agv_tray_sensor', 'assembly_station_sensor']
        # if self.sensor_type in scoring_sensors:
        #     plugin = xml.find('model').find('link').find('sensor').find('plugin')
        #     plugin.set('name', str(self.name + "_ros_plugin"))
        #     plugin.find('sensor_num').text = self.name[-1]

        self.xml = ET.tostring(xml, encoding="unicode")


class PartSpawnParams(SpawnParams):
    part_types = ['battery', 'pump', 'sensor', 'regulator']

    colors = {
        'blue': (0, 0, 168),
        'green': (0, 100, 0),
        'red': (139, 0, 0),
        'purple': (138, 0, 226),
        'orange': (255, 140, 0)   
    }

    def __init__(self, name, part_type, color, xyz=[0,0,0], rpy=[0,0,0], rf=''):
        file_path = os.path.join(get_package_share_directory('ariac_gazebo'), 
            'models', part_type, 'model.sdf')

        super().__init__(name=name, file_path=file_path, xyz=xyz, rpy=rpy, rf=rf)

        self.part_type = part_type
        self.color = color

        self.modify_xml()
    
    def modify_xml(self):
        xml = ET.fromstring(self.get_sdf(self.file_path))

        r, g, b = self.colors[self.color]
        color_string = str(r/255) + " " + str(g/255) + " " + str(b/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'):
            if elem.attrib['name'] == "base":
                elem.find("material").find("ambient").text = color_string
                elem.find("material").find("diffuse").text = color_string

        self.xml = ET.tostring(xml, encoding="unicode")


class TraySpawnParams(SpawnParams):
    def __init__(self, name, marker_id, xyz=[0,0,0], rpy=[0,0,0], rf=''):
        file_path = os.path.join(get_package_share_directory('ariac_gazebo'), 
            'models', 'kit_tray', 'model.sdf')

        super().__init__(name=name, file_path=file_path, xyz=xyz, rpy=rpy, rf=rf)

        self.marker_id = marker_id

        self.modify_xml()
    
    def modify_xml(self):
        xml = ET.fromstring(self.get_sdf(self.file_path))

        marker_string = "model://kit_tray/meshes/markers/marker_" + self.marker_id + ".dae"
        for elem in xml.find('model').find('link').findall('visual'):
            if elem.attrib['name'] == "marker":
                elem.find("geometry").find("mesh").find("uri").text = marker_string

        self.xml =  ET.tostring(xml, encoding="unicode")
