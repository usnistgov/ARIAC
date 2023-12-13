import math
from typing import List, Tuple


from math import pi

from numpy import negative

from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
import PyKDL
from ariac_msgs.msg import (
    Part as PartMsg,
    PartLot as PartLotMsg,
    OrderCondition as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    KittingPart as KittingPartMsg,
    Challenge as ChallengeMsg
)

_part_color_ints = {"RED":0,
                    "GREEN":1,
                    "BLUE":2,
                    "ORANGE":3,
                    "PURPLE":4}
    
_part_type_ints = {"BATTERY":10,
                    "PUMP":11,
                    "SENSOR":12,
                    "REGULATOR":13}

_assembly_part_poses = {}

_assembly_part_install_directions = {}

SLIDER_VALUES = [-pi,-5*pi/6,-4*pi/5,-3*pi/4,-2*pi/3,-3*pi/5,-pi/2,-2*pi/5,-pi/3,-pi/4,-pi/5,-pi/6,0,pi/6,pi/5,pi/4,pi/3,2*pi/5,pi/2,3*pi/5,2*pi/3,3*pi/4,4*pi/5,5*pi/6,pi]
SLIDER_STR = ["-pi","-5*pi/6","-4pi/5","-3pi/4","-2*pi/3","-3pi/5","-pi/2","-2pi/5","-pi/3","-pi/4","-pi/5","-pi/6","0","pi/6","pi/5","pi/4","pi/3","2pi/5","pi/2","3pi/5","2*pi/3","3pi/4","4pi/5","5*pi/6","pi"]
ORDER_TYPES=["kitting", "assembly", "combined"]
ACCEPTED_NUMBERS = "0123456789."  # for requiring number input

SENSORS = ["break_beam", "proximity", "laser_profiler", "lidar", "camera", "logical_camera"]
ROBOTS = ["floor_robot", "ceiling_robot"]
BEHAVIORS = ["antagonistic","indifferent","helpful"]
CHALLENGE_TYPES = ["faulty_part", "dropped_part", "sensor_blackout", "robot_malfunction","human"]

class BinPart():
    def __init__(self,color = "green", pType = "battery", rotation = "", flipped = ""):
        self.part= PartMsg()
        self.part.color = _part_color_ints[color.upper()]
        self.part.type = _part_type_ints[pType.upper()]
        self.rotation = rotation
        self.flipped = flipped

class ConveyorPart():
    def __init__(self,color, pType, num_parts, offset, rotation, flipped):
        part = PartMsg()
        part.color = _part_color_ints[color.upper()]
        part.type = _part_type_ints[pType.upper()]
        self.part_lot = PartLotMsg()
        self.part_lot.part = part
        self.part_lot.quantity = num_parts
        self.rotation = rotation
        self.offset = offset
        self.flipped = flipped

class CompetitionClass():
    def __init__(self, time_limit, tray_ids, slots, assembly_insert_rotations, bin_dict, current_bin_parts, conveyor_active, spawn_rate, conveyor_order,
                 conveyor_parts_to_spawn, current_conveyor_parts, orders, challenges):
        self.competition = {}

        self.competition["time_limit"] = time_limit

        self.competition["kitting_trays"] = {}
        self.competition["kitting_trays"]["tray_ids"] = tray_ids
        self.competition["kitting_trays"]["slots"] = slots

        self.competition["assembly_insert_rotations"] = assembly_insert_rotations

        self.competition["current_bin_parts"] = current_bin_parts
        self.competition["bin_parts"] = bin_dict
        
        self.competition["conveyor_belt"] = {}
        self.competition["conveyor_belt"]["active"] = conveyor_active
        self.competition["conveyor_belt"]["spawn_rate"] = spawn_rate
        self.competition["conveyor_belt"]["order"] = conveyor_order
        self.competition["conveyor_belt"]["parts_to_spawn"] = conveyor_parts_to_spawn
        self.competition["conveyor_belt"]["current_conveyor_parts"] = current_conveyor_parts

        self.competition["orders"] = orders
        
        self.competition["challenges"] = challenges

def _build_assembly_parts_pose_direction():
        regulator_pose = PoseStamped()
        regulator_pose.pose.position.x = 0.175
        regulator_pose.pose.position.y = -0.233
        regulator_pose.pose.position.z = 0.215
        regulator_pose.pose.orientation = quaternion_from_euler(pi/2,0,-pi/2)
        _assembly_part_poses["REGULATOR"] = regulator_pose
        regulator_install_direction = Vector3()
        regulator_install_direction.x = 0
        regulator_install_direction.y = 0
        regulator_install_direction.z = -1
        _assembly_part_install_directions["REGULATOR"] = regulator_install_direction

        battery_pose = PoseStamped()
        battery_pose.pose.position.x = -0.15
        battery_pose.pose.position.y = 0.035
        battery_pose.pose.position.z = 0.043
        battery_pose.pose.orientation = quaternion_from_euler(0,0,pi/2)
        _assembly_part_poses["BATTERY"] = battery_pose
        battery_install_direction = Vector3()
        battery_install_direction.x = 0
        battery_install_direction.y = 1
        battery_install_direction.z = 0
        _assembly_part_install_directions["BATTERY"] = battery_install_direction

        pump_pose = PoseStamped()
        pump_pose.pose.position.x = 0.14
        pump_pose.pose.position.y = 0.0
        pump_pose.pose.position.z = 0.02
        pump_pose.pose.orientation = quaternion_from_euler(0,0,-pi/2)
        _assembly_part_poses["PUMP"] = pump_pose
        pump_install_direction = Vector3()
        pump_install_direction.x = 0
        pump_install_direction.y = 0
        pump_install_direction.z = -1
        _assembly_part_install_directions["PUMP"] = pump_install_direction

        sensor_pose = PoseStamped()
        sensor_pose.pose.position.x = -0.1
        sensor_pose.pose.position.y = 0.395
        sensor_pose.pose.position.z = 0.045
        sensor_pose.pose.orientation = quaternion_from_euler(0,0,-pi/2)
        _assembly_part_poses["SENSOR"] = sensor_pose
        sensor_install_direction = Vector3()
        sensor_install_direction.x = 0
        sensor_install_direction.y = -1
        sensor_install_direction.z = 0
        _assembly_part_install_directions["SENSOR"] = sensor_install_direction

def build_competition_from_file(yaml_dict : dict) -> CompetitionClass:
    _build_assembly_parts_pose_direction()
    time_limit = int(float(yaml_dict["time_limit"]))
    tray_ids = yaml_dict["kitting_trays"]["tray_ids"]
    slots = yaml_dict["kitting_trays"]["slots"]
    
    assembly_insert_rotations = [0.0 for _ in range(4)]
    if "assembly_inserts" in yaml_dict.keys():
        assembly_insert_rotations[0] = SLIDER_VALUES[SLIDER_STR.index(yaml_dict["assembly_inserts"]["as1"])]
        assembly_insert_rotations[1] = SLIDER_VALUES[SLIDER_STR.index(yaml_dict["assembly_inserts"]["as2"])]
        assembly_insert_rotations[2] = SLIDER_VALUES[SLIDER_STR.index(yaml_dict["assembly_inserts"]["as3"])]
        assembly_insert_rotations[3] = SLIDER_VALUES[SLIDER_STR.index(yaml_dict["assembly_inserts"]["as4"])]
    
    bin_parts = {f"bin{i}":[BinPart() for _ in range(9)] for i in range(1,9)}
    current_bin_parts = {f"bin{i}":["" for _ in range(9)] for i in range(1,9)}
    try:
        for bin in yaml_dict["parts"]["bins"].keys():
            for part in yaml_dict["parts"]["bins"][bin]:
                try:
                    part_rotation = SLIDER_VALUES[SLIDER_STR.index(part["rotation"])]
                except:
                    part_rotation = 0.0
                try:
                    part_flipped = "1" if part["flipped"] else "0"
                except:
                    part_flipped = "0"
                for slot in part["slots"]:
                    bin_parts[bin][slot-1] = BinPart(part["color"], part["type"], part_rotation, part_flipped)
                    current_bin_parts[bin][slot-1] = part["color"]+part["type"]
    except:
        pass
    
    try:
        conveyor_active = "1" if yaml_dict["parts"]["conveyor_belt"]["active"] else "0"
        spawn_rate = str(yaml_dict["parts"]["conveyor_belt"]["spawn_rate"])
        conveyor_order = yaml_dict["parts"]["conveyor_belt"]["order"]
        conveyor_parts = []
        current_conveyor_parts = []
        for part in yaml_dict["parts"]["conveyor_belt"]["parts_to_spawn"]:
            num_parts = part["number"]
            try:
                part_offset = part["offset"]
            except:
                part_offset = 0.0
            try:
                part_rotation = SLIDER_VALUES[SLIDER_STR.index(part["rotation"])]
            except:
                part_rotation = 0.0
            try:
                part_flipped = "1" if part["flipped"] else "0"
            except:
                part_flipped = "0"
            conveyor_parts.append(ConveyorPart(part["color"], part["type"],
                                            num_parts, part_offset,
                                            part_rotation, part_flipped))
            current_conveyor_parts.append(part["color"]+part["type"])
    except:
        conveyor_active="1"
        spawn_rate = 1
        conveyor_order = "random"
        conveyor_parts = []
        current_conveyor_parts = []
    
    orders = []
    if "orders" in yaml_dict.keys():
        for order in yaml_dict["orders"]:
            new_order = OrderMsg()
            new_order.id = order["id"]
            new_order.type = ORDER_TYPES.index(order["type"])
            new_order.priority = "1" if order["priority"] else "0"
            if order["type"]=="kitting":
                new_order.kitting_task.agv_number = order["kitting_task"]["agv_number"]
                new_order.kitting_task.tray_id = order["kitting_task"]["tray_id"]
                new_order.kitting_task.destination = 3
                kitting_parts = []
                for part in order["kitting_task"]["products"]:
                    kitting_part = KittingPartMsg()
                    kitting_part.part.color = _part_color_ints[part["color"].upper()]
                    kitting_part.part.type = _part_type_ints[part["type"].upper()]
                    kitting_part.quadrant = part["quadrant"]
                    kitting_parts.append(kitting_part)
                new_order.kitting_task.parts = kitting_parts
            
            if order["type"]=="assembly":
                new_order.assembly_task.agv_numbers = order["assembly_task"]["agv_number"]
                new_order.assembly_task.station = order["assembly_task"]["station"]
                assembly_parts = []
                for part in order["assembly_task"]["products"]:
                    assembly_part = AssemblyPartMsg()
                    assembly_part.part.color = _part_color_ints[part["color"].upper()]
                    assembly_part.part.type = _part_type_ints[part["type"].upper()]
                    assembly_part.assembled_pose = _assembly_part_poses[part["type"].upper()]
                    assembly_part.install_direction = _assembly_part_install_directions[part["type"].upper()]
                    assembly_parts.append(assembly_part)
                new_order.assembly_task.parts = assembly_parts
            
            if order["type"] == "combined":
                new_order.combined_task.station = order["combined_task"]["station"]
                combined_parts = []
                for part in order["combined_task"]["products"]:
                    combined_part = AssemblyPartMsg()
                    combined_part.part.color = _part_color_ints[part["color"].upper()]
                    combined_part.part.type = _part_type_ints[part["type"].upper()]
                    combined_part.assembled_pose = _assembly_part_poses[part["type"].upper()]
                    combined_part.install_direction = _assembly_part_install_directions[part["type"].upper()]
                    combined_parts.append(combined_part)
                new_order.combined_task.parts = combined_parts
            
            current_keys = [key for key in order["announcement"].keys()]
            if "time_condition" in current_keys:
                new_order.condition.type = 0
                new_order.condition.time_condition.seconds = order["announcement"]["time_condition"]
            elif "part_place_condition" in current_keys:
                new_order.condition.type = 1
                new_order.condition.part_place_condition.part.color = _part_color_ints[order["announcement"]["part_place_condition"]["color"].upper()]
                new_order.condition.part_place_condition.part.type = _part_type_ints[order["announcement"]["part_place_condition"]["type"].upper()]
                new_order.condition.part_place_condition.agv = order["announcement"]["part_place_condition"]["agv"]
            else:
                new_order.condition.type = 2
                new_order.condition.submission_condition.order_id = order["announcement"]["submission_condition"]["order_id"]
            orders.append(new_order)

    challenges = []
    if "challenges" in yaml_dict.keys():
        for challenge in yaml_dict["challenges"]:
            new_challenge = ChallengeMsg()
            if "dropped_part" in challenge.keys():
                new_challenge.type = 1
                new_challenge.dropped_part_challenge.robot = challenge["dropped_part"]["robot"]
                new_challenge.dropped_part_challenge.part_to_drop.type = _part_type_ints[challenge["dropped_part"]["type"].upper()]
                new_challenge.dropped_part_challenge.part_to_drop.color = _part_color_ints[challenge["dropped_part"]["color"].upper()]
                new_challenge.dropped_part_challenge.drop_after_num = challenge["dropped_part"]["drop_after"]
                new_challenge.dropped_part_challenge.drop_after_time = challenge["dropped_part"]["delay"]
            elif "human" in challenge.keys():
                new_challenge.type = 4
                new_challenge.human_challenge.behavior = BEHAVIORS.index(challenge["human"]["behavior"])
                if "time_condition" in challenge["human"].keys():
                    new_challenge.human_challenge.condition.type = 0
                    new_challenge.human_challenge.condition.time_condition.seconds = challenge["human"]["time_condition"]
                elif "part_place_condition" in challenge["human"].keys():
                    new_challenge.human_challenge.condition.type = 1
                    new_challenge.human_challenge.condition.part_place_condition.part.color = _part_color_ints[challenge["human"]["part_place_condition"]["color"].upper()]
                    new_challenge.human_challenge.condition.part_place_condition.part.type = _part_type_ints[challenge["human"]["part_place_condition"]["type"].upper()]
                    new_challenge.human_challenge.condition.part_place_condition.agv = challenge["human"]["part_place_condition"]["agv"]
                else:
                    new_challenge.human_challenge.condition.type = 2
                    new_challenge.human_challenge.condition.submission_condition.order_id = challenge["human"]["submission_condition"]["order_id"]
            elif "robot_malfunction" in challenge.keys():
                new_challenge.type = 3
                new_challenge.robot_malfunction_challenge.duration = challenge["robot_malfunction"]["duration"]
                if "floor_robot" in challenge["robot_malfunction"]["robots_to_disable"]:
                    new_challenge.robot_malfunction_challenge.robots_to_disable.floor_robot = True
                if "ceiling_robot" in challenge["robot_malfunction"]["robots_to_disable"]:
                    new_challenge.robot_malfunction_challenge.robots_to_disable.ceiling_robot = True
                if "time_condition" in challenge["robot_malfunction"].keys():
                    new_challenge.robot_malfunction_challenge.condition.type = 0
                    new_challenge.robot_malfunction_challenge.condition.time_condition.seconds = challenge["robot_malfunction"]["time_condition"]
                elif "part_place_condition" in challenge["robot_malfunction"].keys():
                    new_challenge.robot_malfunction_challenge.condition.type = 1
                    new_challenge.robot_malfunction_challenge.condition.part_place_condition.part.color = _part_color_ints[challenge["robot_malfunction"]["part_place_condition"]["color"].upper()]
                    new_challenge.robot_malfunction_challenge.condition.part_place_condition.part.type = _part_type_ints[challenge["robot_malfunction"]["part_place_condition"]["type"].upper()]
                    new_challenge.robot_malfunction_challenge.condition.part_place_condition.agv = challenge["robot_malfunction"]["part_place_condition"]["agv"]
                else:
                    new_challenge.robot_malfunction_challenge.condition.type = 2
                    new_challenge.robot_malfunction_challenge.condition.submission_condition.order_id = challenge["robot_malfunction"]["submission_condition"]["order_id"]
            elif "sensor_blackout" in challenge.keys():
                new_challenge.type = 2
                new_challenge.sensor_blackout_challenge.duration = challenge["sensor_blackout"]["duration"]
                if "break_beam" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.break_beam = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.break_beam = False
                if "proximity" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.proximity = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.proximity = False
                if "laser_profiler" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.laser_profiler = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.laser_profiler = False
                if "lidar" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.lidar = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.lidar = False
                if "camera" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.camera = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.camera = False
                if "logical_camera" in challenge["sensor_blackout"]["sensors_to_disable"]:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.logical_camera = True
                else:
                    new_challenge.sensor_blackout_challenge.sensors_to_disable.logical_camera = False
                if "time_condition" in challenge["sensor_blackout"].keys():
                    new_challenge.sensor_blackout_challenge.condition.type = 0
                    new_challenge.sensor_blackout_challenge.condition.time_condition.seconds = challenge["sensor_blackout"]["time_condition"]
                elif "part_place_condition" in challenge["sensor_blackout"].keys():
                    new_challenge.sensor_blackout_challenge.condition.type = 1
                    new_challenge.sensor_blackout_challenge.condition.part_place_condition.part.color = _part_color_ints[challenge["sensor_blackout"]["part_place_condition"]["color"].upper()]
                    new_challenge.sensor_blackout_challenge.condition.part_place_condition.part.type = _part_type_ints[challenge["sensor_blackout"]["part_place_condition"]["type"].upper()]
                    new_challenge.sensor_blackout_challenge.condition.part_place_condition.agv = challenge["sensor_blackout"]["part_place_condition"]["agv"]
                else:
                    new_challenge.sensor_blackout_challenge.condition.type = 2
                    new_challenge.sensor_blackout_challenge.condition.submission_condition.order_id = challenge["sensor_blackout"]["submission_condition"]["order_id"]
            else:
                new_challenge.type = 0
                new_challenge.faulty_part_challenge.order_id = challenge["faulty_part"]["order_id"]
                try:
                    new_challenge.faulty_part_challenge.quadrant1 = True if challenge["faulty_part"]["quadrant1"] else False
                except:
                    new_challenge.faulty_part_challenge.quadrant1 = False
                try:
                    new_challenge.faulty_part_challenge.quadrant2 = True if challenge["faulty_part"]["quadrant2"] else False
                except:
                    new_challenge.faulty_part_challenge.quadrant2 = False
                try:
                    new_challenge.faulty_part_challenge.quadrant3 = True if challenge["faulty_part"]["quadrant3"] else False
                except:
                    new_challenge.faulty_part_challenge.quadrant3 = False
                try:
                    new_challenge.faulty_part_challenge.quadrant4 = True if challenge["faulty_part"]["quadrant4"] else False
                except:
                    new_challenge.faulty_part_challenge.quadrant4 = False
            challenges.append(new_challenge)


    return CompetitionClass(time_limit, tray_ids, slots, assembly_insert_rotations, bin_parts, current_bin_parts, 
                            conveyor_active, spawn_rate, conveyor_order,
                            conveyor_parts, current_conveyor_parts, orders, challenges)



def rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    q_msg = Quaternion()
    q_msg.w = q[0]
    q_msg.x = q[1]
    q_msg.y = q[2]
    q_msg.z = q[3]

    return q_msg

def build_pose(x,y,z,q : Quaternion)->Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = q
    return p

def require_num(val, _, __, ___):
    """Makes sure a tkinter stringvar is numerical and has no more than one decimal point"""
    perFlag=0
    tempStr=val.get()
    for i in tempStr:
        if i not in ACCEPTED_NUMBERS:
            tempStr=tempStr.replace(i, "")
    if tempStr.count('.')>0:
        for i in range(len(tempStr)):
            if tempStr[i]=='.' and perFlag==0:
                perFlag=1
            elif tempStr[i]=='.':
                tempStr=tempStr[:i]+tempStr[i+1:]
                break
    val.set(tempStr)

def require_int(val, _, __, ___):
    """Makes sure a tkinter stringvar is numerical and has no decimal places"""
    perFlag=0
    tempStr=val.get()
    for i in tempStr:
        if not i.isnumeric():
            tempStr=tempStr.replace(i, "")
    val.set(tempStr)

def validate_time_limit(val, _, __, ___):
    """Makes sure a tkinter stringvar is numerical and the only negative value is -1"""
    tempStr=val.get()
    negative_flag = False
    if len(tempStr)>0:
        negative_flag = tempStr[0]=="-"
    for i in range(len(tempStr)-1,-1,-1):
        if not tempStr[i].isnumeric():
            tempStr=tempStr.replace(tempStr[i], "")
    if negative_flag:
        tempStr="-"+tempStr
    if len(tempStr)>0:
        if tempStr[0]=="-" and tempStr!="-":
                tempStr="-1"
        try:
            if int(tempStr)>400:
                tempStr="400"
        except:
            pass
    val.set(tempStr)