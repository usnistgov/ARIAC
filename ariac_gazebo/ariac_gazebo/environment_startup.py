#!/usr/bin/env python3

import math
import yaml
import xml.etree.ElementTree as ET
from random import randint

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.qos import QoSProfile, DurabilityPolicy

from ariac_gazebo.tf2_geometry_msgs import do_transform_pose
from ariac_gazebo.utilities import quaternion_from_euler, euler_from_quaternion, convert_pi_string_to_float

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Vector3
)

from ariac_msgs.msg import (
    AssemblyPart,
    AssemblyTask,
    BinInfo,
    BinParts,
    Challenge,
    CombinedTask,
    Condition,
    ConveyorParts,
    ConveyorBeltState,
    DroppedPartChallenge,
    FaultyPartChallenge,
    HumanChallenge,
    KittingPart,
    KittingTask,
    OrderCondition,
    Order,
    Part,
    PartLot,
    RobotMalfunctionChallenge,
    SensorBlackoutChallenge,
    Trial,
)

from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Empty
from ariac_gazebo.spawn_params import (
    SpawnParams,
    RobotSpawnParams,
    SensorSpawnParams,
    PartSpawnParams,
    TraySpawnParams)


class PartInfo:
    part_heights = {
        'battery': 0.04,
        'sensor': 0.07,
        'pump': 0.12,
        'regulator': 0.07,
    }

    def __init__(self):
        self.type = None
        self.color = None
        self.rotation = '0'
        self.flipped = False
        self.height = None


class EnvironmentStartup(Node):
    def __init__(self):
        super().__init__('environment_startup_node')

        self.declare_parameter('robot_description', '',
                               ParameterDescriptor(description='Ariac Robots description'))
        self.declare_parameter('trial_config_path', '',
                               ParameterDescriptor(description='Path of the current trial\'s configuration yaml file'))
        self.declare_parameter('user_config_path', '',
                               ParameterDescriptor(description='Path of the user\'s configuration yaml file'))

        self.trial_config = self.read_yaml(
            self.get_parameter('trial_config_path').get_parameter_value().string_value)
        
        self.user_config = self.read_yaml(
            self.get_parameter('user_config_path').get_parameter_value().string_value)
        


        # Conveyor
        self.conveyor_spawn_rate = None
        self.conveyor_parts_to_spawn = []
        self.conveyor_transform = None
        self.conveyor_enabled = False
        self.conveyor_spawn_order_types = ['sequential', 'random']
        self.conveyor_width = 0.28

        self.conveyor_status_sub = self.create_subscription(ConveyorBeltState,
                                                            '/ariac/conveyor_state', self.conveyor_status, 10)

        # Create publishers for bin and conveyor parts
        self.bin_parts = BinParts()
        self.bin_parts_publisher = self.create_publisher(BinParts,
                                                         '/ariac/bin_parts', 10)

        self.conveyor_parts = ConveyorParts()
        self.conveyor_parts_publisher = self.create_publisher(ConveyorParts,
                                                              '/ariac/conveyor_parts', 10)

        self.bin_parts_pub_timer = self.create_timer(
            1.0, self.publish_bin_parts)
        self.conveyor_parts_pub_timer = self.create_timer(
            1.0, self.publish_conveyor_parts)

        # Create publisher for the trial config file
        # This is used by the task manager
        latching_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trial_info_pub = self.create_publisher(
            Trial, '/ariac/trial_config', latching_qos)

        # Create a subscriber for debugging purposes
        # self.trial_config_sub = self.create_subscription(
        #     Trial,
        #     '/ariac/trial_config',
        #     self.trial_config_callback,
        #     10)

        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Create pause and unpause clients
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.unpause_client = self.create_client(Empty, "/unpause_physics")

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    # def trial_config_callback(self, msg: Trial):
    #     """Simple callback to print the trial name

    #     Args:
    #         msg (Trial): The trial config message
    #     """
    #     pass
    #     # self.get_logger().info('Trial name: "%s"' % msg.trial_name)
        
        
    def parse_trial_file(self):
        '''
        Parse the trial configuration file and publish it
        '''        
        config_file_name = self.get_parameter(
            'trial_config_path').get_parameter_value().string_value

        config_file_name = config_file_name.rsplit('/', 1)[1]
        
        message = Trial()

        # If time limit is not specified, use default of -1
        try:
            message.time_limit = float(self.trial_config["time_limit"])
        except KeyError:
            self.get_logger().info(
                "Trial configuration file is missing time_limit field...default to -1")

        # Check we have at least one order
        try:
            orders = self.trial_config["orders"]
        except KeyError:
            self.get_logger().fatal("No orders found in trial configuration file...exiting")
            return

        # Retrieve challenges
        try:
            challenges = self.trial_config["challenges"]
            message.challenges = self.create_challenge_list(challenges)
        except KeyError:
            self.get_logger().info("No challenges found in trial configuration file")

        message.order_conditions = self.create_order_list(orders)
        
        message.trial_name = config_file_name
        self.trial_info_pub.publish(message)


    def convert_order_type_to_int(self, order_type):
        """Converts an order type to an integer.
        - kitting: Order.KITTING
        - assembly: Order.ASSEMBLY
        - combined: Order.COMBINED

        Args:
            order_type (str): The type of an order in string format.

        Returns:
            int: The type of an order in integer format.
        """
        options = {
            'kitting': Order.KITTING,
            'assembly': Order.ASSEMBLY,
            'combined': Order.COMBINED
        }

        found_type = options.get(order_type)

        if found_type in [Order.KITTING, Order.ASSEMBLY, Order.COMBINED]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Order type '{order_type}' is not correct. Check spelling.")
            return None


    def convert_behavior_to_int(self, behavior):
        """Converts a behavior to an integer.
        - antagonistic: HumanChallenge.ANTAGONISTIC
        - indifferent: HumanChallenge.INDIFFERENT
        - helpful: HumanChallenge.HELPFUL

        Args:
            behavior (str): The behavior of the human in string format.

        Returns:
            int: The behavior of a human in integer format.
        """
        options = {
            'antagonistic': HumanChallenge.ANTAGONISTIC,
            'indifferent': HumanChallenge.INDIFFERENT,
            'helpful': HumanChallenge.HELPFUL
        }

        found_behavior = options.get(behavior)

        if found_behavior in [HumanChallenge.ANTAGONISTIC, HumanChallenge.INDIFFERENT, HumanChallenge.HELPFUL]:
            return found_behavior
        else:
            self.get_logger().fatal(
                f"Behavior '{behavior}' is not correct. Check spelling.")
            return None
        
        
    def convert_part_type_to_int(self, part_type):
        """Converts a part type to an integer.
        - battery: Part.BATTERY
        - pump: Part.PUMP
        - sensor: Part.SENSOR
        - regulator: Part.REGULATOR

        Args:
            part_type (str): The type of a part in string format.

        Returns:
            int: The type of a part in integer format.
        """

        options = {
            'battery': Part.BATTERY,
            'pump': Part.PUMP,
            'sensor': Part.SENSOR,
            'regulator': Part.REGULATOR
        }

        found_type = options.get(part_type)

        if found_type in [Part.BATTERY, Part.PUMP, Part.SENSOR, Part.REGULATOR]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Part type '{part_type}' is not correct. Check spelling.")
            return None


    def convert_part_color_to_int(self, part_color):
        """Converts a part color to an integer.
        - red: Part.RED
        - green: Part.GREEN
        - blue: Part.BLUE
        - orange: Part.ORANGE
        - purple: Part.PURPLE

        Args:
            part_color (str): The color of a part in string format.

        Returns:
            int: The color of a part in integer format.
        """

        options = {
            'red': Part.RED,
            'green': Part.GREEN,
            'blue': Part.BLUE,
            'orange': Part.ORANGE,
            'purple': Part.PURPLE
        }

        found_type = options.get(part_color)

        if found_type in [Part.RED, Part.GREEN, Part.BLUE, Part.ORANGE, Part.PURPLE]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Part color '{part_color}' is not correct. Check spelling.")
            return None


    def convert_destination_to_int(self, destination):
        """Converts a destination to an integer.
        - as1: KittingTask.ASSEMBLY_FRONT,
        - as2: KittingTask.ASSEMBLY_BACK,
        - as3: KittingTask.ASSEMBLY_FRONT,
        - as4: KittingTask.ASSEMBLY_BACK,
        - kitting: KittingTask.KITTING,
        - warehouse: KittingTask.WAREHOUSE,

        Args:
            destination (str): The destination in string format.

        Returns:
            int: The destination in integer format.
        """

        options = {
            'as1': KittingTask.ASSEMBLY_FRONT,
            'as2': KittingTask.ASSEMBLY_BACK,
            'as3': KittingTask.ASSEMBLY_FRONT,
            'as4': KittingTask.ASSEMBLY_BACK,
            'kitting': KittingTask.KITTING,
            'warehouse': KittingTask.WAREHOUSE,
        }

        found_type = options.get(destination)

        if found_type in [KittingTask.ASSEMBLY_FRONT,
                          KittingTask.ASSEMBLY_BACK,
                          KittingTask.ASSEMBLY_FRONT,
                          KittingTask.ASSEMBLY_BACK,
                          KittingTask.KITTING,
                          KittingTask.WAREHOUSE]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Destination '{destination}' is not correct. Check spelling.")
            return None


    def convert_condition_to_int(self, condition):
        """Converts a condition to an integer.
        - 'time_condition': Condition.TIME,
        - 'part_place_condition': Condition.PART_PLACE,
        - 'submission_condition': Condition.SUBMISSION,

        Args:
            condition (str): The condition in string format.

        Returns:
            int: The condition in integer format.
        """
        options = {
            'time_condition': Condition.TIME,
            'part_place_condition': Condition.PART_PLACE,
            'submission_condition': Condition.SUBMISSION,
        }
        
        found_type = options.get(condition)

        if found_type in [Condition.TIME, Condition.PART_PLACE, Condition.SUBMISSION]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Condition '{condition}' is not correct. Check spelling.")
            return None
        
        
        
    def convert_assembly_station_to_int(self, assembly_station):
        """Converts an assembly station to an integer.
        - as1: AssemblyTask.AS1,
        - as2: AssemblyTask.AS2,
        - as3: AssemblyTask.AS3,
        - as4: AssemblyTask.AS4,

        Args:
            assembly_station (str): The assembly station in string format.

        Returns:
            int: The assembly station in integer format.
        """

        options = {
            'as1': AssemblyTask.AS1,
            'as2': AssemblyTask.AS2,
            'as3': AssemblyTask.AS3,
            'as4': AssemblyTask.AS4
        }

        found_type = options.get(assembly_station)

        if found_type in [AssemblyTask.AS1, AssemblyTask.AS2, AssemblyTask.AS3, AssemblyTask.AS4]:
            return found_type
        else:
            self.get_logger().fatal(
                f"Assembly station '{assembly_station}' is not correct. Check spelling.")
            return None


    def create_robot_malfunction(self, challenge_dict):
        """Method to build and return a RobotMalfunctionChallenge object from a dictionary

        Args:
            challenge_dict (dict): robot_malfunction dictionary from config file

        Returns:
            RobotMalfunctionChallenge: Object containing robot malfunction information
        """
        msg = RobotMalfunctionChallenge()
        msg.duration = float(challenge_dict['duration'])

        if "floor_robot" in challenge_dict['robots_to_disable']:
            msg.robots_to_disable.floor_robot = True
        if "ceiling_robot" in challenge_dict['robots_to_disable']:
            msg.robots_to_disable.ceiling_robot = True

        if challenge_dict.get('part_place_condition'):
            msg.condition.type = Condition.PART_PLACE
            msg.condition.part_place_condition.part.type = self.convert_part_type_to_int(
                challenge_dict['part_place_condition']['type'])
            msg.condition.part_place_condition.part.color = self.convert_part_color_to_int(
                challenge_dict['part_place_condition']['color'])

            for key in challenge_dict['part_place_condition'].keys():
                if key == 'agv':
                    msg.condition.part_place_condition.agv = challenge_dict[
                        'part_place_condition']['agv']
                # elif key == 'as':
                #     msg.condition.part_place_condition.station = self.convert_assembly_station_to_int(
                #         challenge_dict['part_place_condition']['as'])
        elif challenge_dict.get('time_condition'):
            msg.condition.type = Condition.TIME
            msg.condition.time_condition.seconds = float(challenge_dict['time_condition'])
        elif challenge_dict.get('submission_condition'):
            msg.condition.type = Condition.SUBMISSION
            msg.condition.submission_condition.order_id = challenge_dict[
                'submission_condition']['order_id']

        challenge_msg = Challenge()
        challenge_msg.type = Challenge.ROBOT_MALFUNCTION
        challenge_msg.robot_malfunction_challenge = msg
        return challenge_msg
    
    def create_human_challenge(self, challenge_dict):
        """Method to build and return a HumanChallenge object from a dictionary
        
        Args:
            challenge_dict (dict): human dictionary from config file

        Returns:
            HumanChallenge: Object containing human challenge information
        
        """
        msg = HumanChallenge()
        
        # Set the behavior
        msg.behavior = self.convert_behavior_to_int(challenge_dict['behavior'])

        # Set the condition
        if challenge_dict.get('part_place_condition'):
            msg.condition.type = Condition.PART_PLACE
            msg.condition.part_place_condition.part.type = self.convert_part_type_to_int(
                challenge_dict['part_place_condition']['type'])
            msg.condition.part_place_condition.part.color = self.convert_part_color_to_int(
                challenge_dict['part_place_condition']['color'])

            for key in challenge_dict['part_place_condition'].keys():
                if key == 'agv':
                    msg.condition.part_place_condition.agv = challenge_dict[
                        'part_place_condition']['agv']
                # elif key == 'as':
                #     msg.condition.part_place_condition.station = self.convert_assembly_station_to_int(
                #         challenge_dict['part_place_condition']['as'])
        elif challenge_dict.get('time_condition'):
            msg.condition.type = Condition.TIME
            msg.condition.time_condition.seconds = float(
                challenge_dict['time_condition'])
        elif challenge_dict.get('submission_condition'):
            msg.condition.type = Condition.SUBMISSION
            msg.condition.submission_condition.order_id = challenge_dict[
                'submission_condition']['order_id']

        challenge_msg = Challenge()
        challenge_msg.type = Challenge.HUMAN
        challenge_msg.human_challenge = msg
        return challenge_msg
    
    def create_dropped_part(self, challenge_dict):
        """Method to build and return a DroppedPartChallenge object from a dictionary
        
        Args:
            challenge_dict (dict): dropped_part dictionary from config file

        Returns:
            DroppedPartChallenge: Object containing dropped part information
        
        """
        msg = DroppedPartChallenge()
        msg.robot = challenge_dict['robot']
        # print("ROBOT", challenge_dict['robot'])
        
        part_msg = Part()
        part_msg.type = self.convert_part_type_to_int(challenge_dict['type'])
        part_msg.color = self.convert_part_color_to_int(challenge_dict['color'])
        msg.part_to_drop = part_msg
        msg.drop_after_num = int(challenge_dict['drop_after'])
        msg.drop_after_time = float(challenge_dict['delay'])

        challenge_msg = Challenge()
        challenge_msg.type = Challenge.DROPPED_PART
        challenge_msg.dropped_part_challenge = msg
        return challenge_msg
        

    def create_faulty_part(self, challenge_dict):
        """Method to build and return a FaultyPartChallenge object from a dictionary

        Args:
            challenge_dict (dict): faulty_part dictionary from config file

        Returns:
            FaultyPartChallenge: Object containing faulty part information
        """
        msg = FaultyPartChallenge()
        msg.order_id = challenge_dict['order_id']
        msg.quadrant1 = False
        msg.quadrant2 = False
        msg.quadrant3 = False
        msg.quadrant4 = False

        if challenge_dict.get('quadrant1'):
            msg.quadrant1 = challenge_dict['quadrant1']
        if challenge_dict.get('quadrant2'):
            msg.quadrant2 = challenge_dict['quadrant2']
        if challenge_dict.get('quadrant3'):
            msg.quadrant3 = challenge_dict['quadrant3']
        if challenge_dict.get('quadrant4'):
            msg.quadrant4 = challenge_dict['quadrant4']

        challenge_msg = Challenge()
        challenge_msg.type = Challenge.FAULTY_PART
        challenge_msg.faulty_part_challenge = msg
        return challenge_msg

    def create_sensor_blackout(self, challenge_dict):
        """Method to build and return a SensorBlackoutChallenge object from a dictionary
        Args:
            challenge_dict (dict): sensor_blackout dictionary from config file
        Returns:
            SensorBlackoutChallenge: Object containing sensor blackout information
        """
        msg = SensorBlackoutChallenge()
        msg.duration = float(challenge_dict['duration'])

        if "break_beam" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.break_beam = True
        if "proximity" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.proximity = True
        if "laser_profiler" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.laser_profiler = True
        if "lidar" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.lidar = True
        if "camera" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.camera = True
        if "logical_camera" in challenge_dict['sensors_to_disable']:
            msg.sensors_to_disable.logical_camera = True

        if challenge_dict.get('part_place_condition'):
            msg.condition.type = Condition.PART_PLACE
            msg.condition.part_place_condition.part.type = self.convert_part_type_to_int(
                challenge_dict['part_place_condition']['type'])
            msg.condition.part_place_condition.part.color = self.convert_part_color_to_int(
                challenge_dict['part_place_condition']['color'])

            for key in challenge_dict['part_place_condition'].keys():
                if key == 'agv':
                    agv = challenge_dict['part_place_condition']['agv']
                    msg.condition.part_place_condition.agv = agv
                # elif key == 'as':
                #     station = challenge_dict['part_place_condition']['as']
                #     msg.condition.part_place_condition.station = self.convert_assembly_station_to_int(
                #         station)

        elif challenge_dict.get('time_condition'):
            msg.condition.type = Condition.TIME
            msg.condition.time_condition.seconds = float(
                challenge_dict['time_condition'])
        elif challenge_dict.get('submission_condition'):
            msg.condition.type = Condition.SUBMISSION
            msg.condition.submission_condition.order_id = challenge_dict[
                'submission_condition']['order_id']

        challenge_msg = Challenge()
        challenge_msg.type = Challenge.SENSOR_BLACKOUT
        challenge_msg.sensor_blackout_challenge = msg
        return challenge_msg


    def create_challenge_list(self, challenges):
        """Create a list of challenges from a list of challenge dictionaries

        Args:
            challenges (dict): List of challenge dictionaries

        Returns:
            [ariac_msgs.Challenge]: List of challenges
        """

        challenge_list = []

        for challenge in challenges:
            for key, value in challenge.items():
                # self.get_logger().fatal(f"KEY: {key}")
                if key == 'robot_malfunction':
                    challenge_list.append(self.create_robot_malfunction(value))
                elif key == 'sensor_blackout':
                    challenge_list.append(self.create_sensor_blackout(value))
                elif key == 'faulty_part':
                    challenge_list.append(self.create_faulty_part(value))
                elif key == 'dropped_part':
                    challenge_list.append(self.create_dropped_part(value))
                elif key == 'human':
                    challenge_list.append(self.create_human_challenge(value))

        return challenge_list


    def create_assembly_task(self, assembly_task_dict):
        msg = AssemblyTask()
        products = assembly_task_dict['products']
        assembly_product_list = []
        for product in products:
            assembly_part_msg = AssemblyPart()
            assembly_part_msg.part.type = self.convert_part_type_to_int(
                product['type'])
            assembly_part_msg.part.color = self.convert_part_color_to_int(
                product['color'])

            assembly_part_msg.assembled_pose = PoseStamped()
            assembly_part_msg.assembled_pose.pose.position.x = product['assembled_pose']['xyz'][0]
            assembly_part_msg.assembled_pose.pose.position.y = product['assembled_pose']['xyz'][1]
            assembly_part_msg.assembled_pose.pose.position.z = product['assembled_pose']['xyz'][2]

            roll = product['assembled_pose']['rpy'][0]
            if isinstance(roll, str):
                roll = convert_pi_string_to_float(roll)

            pitch = product['assembled_pose']['rpy'][1]
            if isinstance(pitch, str):
                pitch = convert_pi_string_to_float(pitch)

            yaw = product['assembled_pose']['rpy'][2]
            if isinstance(yaw, str):
                yaw = convert_pi_string_to_float(yaw)

            orientation = quaternion_from_euler(roll, pitch, yaw)
            assembly_part_msg.assembled_pose.pose.orientation.w = orientation[0]
            assembly_part_msg.assembled_pose.pose.orientation.x = orientation[1]
            assembly_part_msg.assembled_pose.pose.orientation.y = orientation[2]
            assembly_part_msg.assembled_pose.pose.orientation.z = orientation[3]

            vector3 = Vector3()
            vector3.x = float(product['assembly_direction'][0])
            vector3.y = float(product['assembly_direction'][1])
            vector3.z = float(product['assembly_direction'][2])
            assembly_part_msg.install_direction = vector3
            # add the product to the list
            assembly_product_list.append(assembly_part_msg)

        agv_number_list = []
        for item in assembly_task_dict['agv_number']:
            agv_number_list.append(int(item))
        msg.agv_numbers = agv_number_list
        msg.station = self.convert_assembly_station_to_int(
            assembly_task_dict['station'])
        msg.parts = assembly_product_list
        return msg


    def create_combined_task(self, combined_task_dict):
        msg = CombinedTask()
        products = combined_task_dict['products']
        product_list = []
        for product in products:
            assembly_part_msg = AssemblyPart()
            assembly_part_msg.part.type = self.convert_part_type_to_int(
                product['type'])
            assembly_part_msg.part.color = self.convert_part_color_to_int(
                product['color'])

            assembly_part_msg.assembled_pose = PoseStamped()
            assembly_part_msg.assembled_pose.pose.position.x = product['assembled_pose']['xyz'][0]
            assembly_part_msg.assembled_pose.pose.position.y = product['assembled_pose']['xyz'][1]
            assembly_part_msg.assembled_pose.pose.position.z = product['assembled_pose']['xyz'][2]

            roll = product['assembled_pose']['rpy'][0]
            if isinstance(roll, str):
                roll = convert_pi_string_to_float(roll)

            pitch = product['assembled_pose']['rpy'][1]
            if isinstance(pitch, str):
                pitch = convert_pi_string_to_float(pitch)

            yaw = product['assembled_pose']['rpy'][2]
            if isinstance(yaw, str):
                yaw = convert_pi_string_to_float(yaw)

            orientation = quaternion_from_euler(roll, pitch, yaw)
            assembly_part_msg.assembled_pose.pose.orientation.w = orientation[0]
            assembly_part_msg.assembled_pose.pose.orientation.x = orientation[1]
            assembly_part_msg.assembled_pose.pose.orientation.y = orientation[2]
            assembly_part_msg.assembled_pose.pose.orientation.z = orientation[3]

            vector3 = Vector3()
            vector3.x = float(product['assembly_direction'][0])
            vector3.y = float(product['assembly_direction'][1])
            vector3.z = float(product['assembly_direction'][2])
            assembly_part_msg.install_direction = vector3
            # add the product to the list
            product_list.append(assembly_part_msg)

        msg.station = self.convert_assembly_station_to_int(
            combined_task_dict['station'])
        msg.parts = product_list
        return msg


    def create_kitting_task(self, kitting_task_dict):
        msg = KittingTask()
        products = kitting_task_dict['products']
        kitting_product_list = []
        for product in products:
            kitting_part_msg = KittingPart()
            kitting_part_msg.part.type = self.convert_part_type_to_int(
                product['type'])
            kitting_part_msg.part.color = self.convert_part_color_to_int(
                product['color'])
            kitting_part_msg.quadrant = product['quadrant']
            kitting_product_list.append(kitting_part_msg)

        msg.agv_number = kitting_task_dict['agv_number']
        msg.tray_id = kitting_task_dict['tray_id']
        msg.destination = self.convert_destination_to_int(
            kitting_task_dict['destination'])
        msg.parts = kitting_product_list
        return msg


    def create_order_list(self, orders):
        order_condition_list = []
        for order in orders:
            order_condition = OrderCondition()
            order_condition.id = order['id']
            order_condition.type = self.convert_order_type_to_int(
                order['type'])
            order_condition.priority = order['priority']
            # Announcement
            announcement = order['announcement']
            condition = Condition()
            for announcement_key, announcement_value in announcement.items():
                if announcement_key == 'time_condition':
                    condition.type = Condition.TIME
                    condition.time_condition.seconds = float(
                        announcement_value)
                elif announcement_key == 'part_place_condition':
                    condition.type = Condition.PART_PLACE
                    part_color = order['announcement']['part_place_condition']['color']
                    condition.part_place_condition.part.color = self.convert_part_color_to_int(
                        part_color)
                    part_type = order['announcement']['part_place_condition']['type']
                    condition.part_place_condition.part.type = self.convert_part_type_to_int(
                        part_type)

                    for key in order['announcement']['part_place_condition'].keys():
                        if key == 'agv':
                            agv = order['announcement']['part_place_condition']['agv']
                            condition.part_place_condition.agv = agv
                        # elif key == 'as':
                        #     station = order['announcement']['part_place_condition']['as']
                        #     condition.part_place_condition.station = self.convert_assembly_station_to_int(
                        #         station)

                elif announcement_key == 'submission_condition':
                    condition.type = Condition.SUBMISSION
                    # print("Condition type", condition.type)
                    order_id = order['announcement']['submission_condition']['order_id']
                    condition.submission_condition.order_id = order_id
            # Set condition for order
            order_condition.condition = condition
            # Set task for order
            if order['type'] == "kitting":
                order_condition.kitting_task = self.create_kitting_task(
                    order['kitting_task'])
            elif order['type'] == "assembly":
                order_condition.assembly_task = self.create_assembly_task(
                    order['assembly_task'])
            elif order['type'] == "combined":
                order_condition.combined_task = self.create_combined_task(
                    order['combined_task'])

            order_condition_list.append(order_condition)
        return order_condition_list


    def spawn_sensors(self):
        try:
            user_sensors = self.user_config['sensors']
        except (TypeError, KeyError):
            self.get_logger().warn("No sensors found in config")
            user_sensors = []

        if not user_sensors:
            user_sensors = []

        # Spawn user sensors
        for sensor_name in user_sensors:
            sensor_type = user_sensors[sensor_name]['type']
            xyz = user_sensors[sensor_name]['pose']['xyz']
            rpy = user_sensors[sensor_name]['pose']['rpy']

            if 'visualize_fov' in user_sensors[sensor_name].keys():
                vis = user_sensors[sensor_name]['visualize_fov']
            else:
                vis = False

            params = SensorSpawnParams(
                sensor_name, sensor_type, visualize=vis, xyz=xyz, rpy=rpy)
            self.spawn_entity(params)

        # Spawn agv tray sensors
        for i in range(1, 5):
            sensor_name = "agv_tray_sensor_" + str(i)
            sensor_type = "agv_tray_sensor"
            xyz = [0, 0, 1]
            vis = False

            params = SensorSpawnParams(
                sensor_name, sensor_type, visualize=vis, xyz=xyz)
            params.reference_frame = "agv" + str(i) + "_tray"
            self.spawn_entity(params)
        
        # Spawn assembly station sensors
        for i in range(1, 5):
            sensor_name = "assembly_station_sensor_" + str(i)
            sensor_type = "assembly_station_sensor"
            vis = False

            try:
                t = self.tf_buffer.lookup_transform('world', "as" + str(i) + "_insert_frame", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform assembly station {i} to world: {ex}')
                return
            
            xyz = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z + 0.989]

            params = SensorSpawnParams(sensor_name, sensor_type, visualize=vis, xyz=xyz)

            self.spawn_entity(params)

        # Spawn assembly station sensors
        for i in range(1, 5):
            sensor_name = "assembly_station_sensor_" + str(i)
            sensor_type = "assembly_station_sensor"
            vis = False

            try:
                t = self.tf_buffer.lookup_transform(
                    'world', "as" + str(i) + "_insert_frame", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform assembly station {i} to world: {ex}')
                return

            xyz = [t.transform.translation.x, t.transform.translation.y,
                   t.transform.translation.z + 0.989]

            params = SensorSpawnParams(
                sensor_name, sensor_type, visualize=vis, xyz=xyz)

            self.spawn_entity(params)


    def spawn_kit_trays(self):
        possible_slots = [1, 2, 3, 4, 5, 6]
        possible_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

        slot_info = {
            1: {"table": "table_1", "offset": -0.43},
            2: {"table": "table_1", "offset": 0.0},
            3: {"table": "table_1", "offset": 0.43},
            4: {"table": "table_2", "offset": -0.43},
            5: {"table": "table_2", "offset": 0.0},
            6: {"table": "table_2", "offset": 0.43},
        }

        # Check that input is valid
        try:
            ids = self.trial_config["kitting_trays"]["tray_ids"]
            slots = self.trial_config["kitting_trays"]["slots"]
        except KeyError:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if len(ids) == 0 or len(slots) == 0:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if not (len(ids) == len(slots)):
            self.get_logger().warn("Number of trays does not equal number of slots")
            return

        for id, slot in zip(ids, slots):
            if not type(id) == int or not type(slot) == int:
                self.get_logger().warn("Tray ids and slots must be integers")
                return
            elif id not in possible_ids:
                self.get_logger().warn("Tray id must be between 0 and 9")
                return
            elif slot not in possible_slots:
                self.get_logger().warn("Tray slot must be between 1 and 6")
                return

        # Calculate location of tables using tf
        transforms = {}
        try:
            transforms['table_1'] = self.tf_buffer.lookup_transform(
                'world', "kts1_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform kts1_table_frame to world: {ex}')
            return

        try:
            transforms['table_2'] = self.tf_buffer.lookup_transform(
                'world', "kts2_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform kts1_table_frame to world: {ex}')
            return

        # Spawn trays
        num_trays = 0
        kit_tray_thickness = 0.01
        for id, slot in zip(ids, slots):
            rel_pose = Pose()
            rel_pose.position.x = slot_info[slot]["offset"]
            rel_pose.position.z = kit_tray_thickness

            world_pose = do_transform_pose(
                rel_pose, transforms[slot_info[slot]["table"]])

            # Create unique name for each tray that includes the id
            marker_id = str(id).zfill(2)
            name = "kit_tray_" + marker_id + "_" + str(num_trays)
            num_trays += 1

            xyz = [world_pose.position.x,
                   world_pose.position.y, world_pose.position.z]
            rpy = euler_from_quaternion(world_pose.orientation)

            params = TraySpawnParams(name, marker_id, xyz=xyz, rpy=rpy)

            self.spawn_entity(params, wait=False)


    def spawn_bin_parts(self):
        possible_bins = ['bin1', 'bin2', 'bin3',
                         'bin4', 'bin5', 'bin6', 'bin7', 'bin8']

        slot_info = {
            1: {"x_offset": 0.18, "y_offset": 0.18},
            2: {"x_offset": 0.18, "y_offset": 0.0},
            3: {"x_offset": 0.18, "y_offset": -0.18},
            4: {"x_offset": 0.0, "y_offset": 0.18},
            5: {"x_offset": 0.0, "y_offset": 0.0},
            6: {"x_offset": 0.0, "y_offset": -0.18},
            7: {"x_offset": -0.18, "y_offset": 0.18},
            8: {"x_offset": -0.18, "y_offset": 0.0},
            9: {"x_offset": -0.18, "y_offset": -0.18},
        }

        # Validate input
        try:
            bin_parts_config = self.trial_config["parts"]["bins"]
        except KeyError:
            self.get_logger().warn("No bin parts found in configuration")
            return

        if not bin_parts_config:
            return

        part_count = 0
        for bin_name in bin_parts_config.keys():
            if not bin_name in possible_bins:
                self.get_logger().warn(f"{bin_name} is not a valid bin name")
                continue

            try:
                bin_transform = self.tf_buffer.lookup_transform(
                    'world', bin_name + "_frame", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {bin_name}_frame to world: {ex}')
                return

            # Fill bin info msg for each bin that has parts
            bin_info = BinInfo()
            bin_info.bin_number = int(bin_name[-1])

            available_slots = list(range(1, 10))
            for part_info in bin_parts_config[bin_name]:
                ret, part = self.parse_part_info(part_info)
                if not ret:
                    continue

                try:
                    slots = part_info['slots']
                    if not type(slots) == list:
                        self.get_logger().warn("slots parameter should be a list of integers")
                        continue
                except KeyError:
                    self.get_logger().warn("Part slots are not specified")
                    continue

                # Spawn parts into slots
                num_parts_in_bin = 0
                for slot in slots:
                    if not slot in slot_info.keys():
                        self.get_logger().warn(
                            f"Slot {slot} is not a valid option")
                        continue
                    elif not slot in available_slots:
                        self.get_logger().warn(
                            f"Slot {slot} is already occupied")
                        continue

                    num_parts_in_bin += 1

                    available_slots.remove(slot)

                    part_name = part.type + "_" + part.color + \
                        "_b" + str(part_count).zfill(2)
                    part_count += 1

                    if part.flipped:
                        roll = math.pi
                    else:
                        roll = 0

                    yaw = convert_pi_string_to_float(part.rotation)

                    q = quaternion_from_euler(roll, 0, yaw)
                    rel_pose = Pose()
                    rel_pose.position.x = slot_info[slot]["x_offset"]
                    rel_pose.position.y = slot_info[slot]["y_offset"]

                    if part.flipped:
                        rel_pose.position.z = part.height

                    rel_pose.orientation.w = q[0]
                    rel_pose.orientation.x = q[1]
                    rel_pose.orientation.y = q[2]
                    rel_pose.orientation.z = q[3]

                    world_pose = do_transform_pose(rel_pose, bin_transform)

                    xyz = [world_pose.position.x,
                           world_pose.position.y, world_pose.position.z]
                    rpy = euler_from_quaternion(world_pose.orientation)

                    params = PartSpawnParams(
                        part_name, part.type, part.color, xyz=xyz, rpy=rpy)

                    self.spawn_entity(params, wait=False)

                bin_info.parts.append(
                    self.fill_part_lot_msg(part, num_parts_in_bin))

            self.bin_parts.bins.append(bin_info)

    def fill_part_lot_msg(self, part: PartInfo, quantity: int):
        part_colors = {
            'red': Part.RED,
            'green': Part.GREEN,
            'blue': Part.BLUE,
            'orange': Part.ORANGE,
            'purple': Part.PURPLE,
        }

        part_types = {
            'battery': Part.BATTERY,
            'pump': Part.PUMP,
            'sensor': Part.SENSOR,
            'regulator': Part.REGULATOR,
        }

        lot = PartLot()
        lot.part.type = part_types[part.type]
        lot.part.color = part_colors[part.color]
        lot.quantity = quantity

        return lot

    def spawn_conveyor_part(self):
        if self.conveyor_enabled:
            if self.conveyor_spawn_order == 'sequential':
                part_params = self.conveyor_parts_to_spawn.pop(0)
            elif self.conveyor_spawn_order == 'random':
                part_params = self.conveyor_parts_to_spawn.pop(
                    randint(0, len(self.conveyor_parts_to_spawn) - 1))

            self.spawn_entity(part_params, wait=False)

        if not self.conveyor_parts_to_spawn:
            self.conveyor_spawn_timer.cancel()

    def spawn_robots(self):
        urdf = ET.fromstring(self.get_parameter('robot_description').value)

        params = RobotSpawnParams(
            "ariac_robots", ET.tostring(urdf, encoding="unicode"))

        self.spawn_entity(params)

    def spawn_parts_on_agvs(self):
        quadrant_info = {
            1: {"x_offset": -0.0925, "y_offset": 0.1275},
            2: {"x_offset": 0.0925, "y_offset": 0.1275},
            3: {"x_offset": -0.0925, "y_offset": -0.1275},
            4: {"x_offset": 0.0925, "y_offset": -0.1275},
        }

        possible_agvs = ['agv1', 'agv2', 'agv3', 'agv4']

        # Validate input
        try:
            agv_parts = self.trial_config["parts"]["agvs"]
        except KeyError:
            # self.get_logger().warn("No agv parts found in configuration")
            return

        part_count = 0
        for agv in agv_parts:
            if not agv in possible_agvs:
                self.get_logger().warn(f"{agv} is not a valid agv name")
                continue

            # Spawn a kit tray onto the AGV
            try:
                tray_id = agv_parts[agv]['tray_id']
            except KeyError:
                tray_id = 0

            marker_id = str(tray_id).zfill(2)
            name = "kit_tray_" + marker_id + "_a" + agv[-1]

            xyz = [0, 0, 0.01]
            reference_frame = agv + "_tray"
            params = TraySpawnParams(
                name, marker_id, xyz=xyz, rf=reference_frame)
            self.spawn_entity(params)

            # Spawn parts onto kit tray
            available_quadrants = list(range(1, 5))
            for part_info in agv_parts[agv]['parts']:
                ret, part = self.parse_part_info(part_info)
                if not ret:
                    continue

                try:
                    quadrant = part_info['quadrant']
                    if not quadrant in quadrant_info.keys():
                        self.get_logger().warn(
                            f"Quadrant {quadrant} is not an option")
                        continue
                except KeyError:
                    self.get_logger().warn("Quadrant is not specified")
                    continue

                available_quadrants.remove(quadrant)

                if part.flipped:
                    roll = math.pi
                    z_height = part.height + 0.01
                else:
                    roll = 0
                    z_height = 0.01

                part_name = part.type + "_" + part.color + \
                    "_a" + str(part_count).zfill(2)
                part_count += 1

                xyz = [quadrant_info[quadrant]["x_offset"],
                       quadrant_info[quadrant]["y_offset"], z_height]
                rpy = [roll, 0, part.rotation]

                params = PartSpawnParams(
                    part_name, part.type, part.color, xyz=xyz, rpy=rpy, rf=reference_frame)

                self.spawn_entity(params, wait=False)

    def parse_conveyor_config(self):
        # Parse Conveyor Configuration
        try:
            conveyor_config = self.trial_config['parts']['conveyor_belt']
        except KeyError:
            self.get_logger().info("Conveyor belt not in use in this trial")
            return False

        try:
            active = conveyor_config['active']
            if not active:
                return False
        except KeyError:
            self.get_logger().error("Active paramater not set in conveyor belt configuration")
            return False

        try:
            spawn_rate = conveyor_config['spawn_rate']
        except KeyError:
            self.get_logger().error("Spawn rate paramater not set in conveyor belt configuration")
            return False

        try:
            self.spawn_rate = float(spawn_rate)
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        # Get conveyor transform
        try:
            conveyor_transform = self.tf_buffer.lookup_transform('world',
                                                                 "conveyor_belt_part_spawn_frame", rclpy.time.Time())
        except TransformException:
            return

        try:
            if conveyor_config['order'] in self.conveyor_spawn_order_types:
                self.conveyor_spawn_order = conveyor_config['order']
            else:
                self.get_logger().error(
                    f"Order paramater must be of type: {self.conveyor_spawn_order_types}")
                return False
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        try:
            parts = conveyor_config['parts_to_spawn']
        except KeyError:
            self.get_logger().error("Parts to spawn not found in configuration")
            return False

        part_count = 0
        for part_info in parts:
            ret, part = self.parse_part_info(part_info)

            if not ret:
                continue

            try:
                amount = part_info['number']
            except KeyError:
                continue

            try:
                offset = part_info['offset']
                if not -1 <= offset <= 1:
                    self.get_logger().error("Offset must be in range [-1, 1]")
                    offset = 0
            except KeyError:
                offset = 0

            self.conveyor_parts.parts.append(
                self.fill_part_lot_msg(part, amount))

            for i in range(amount):
                part_name = part.type + "_" + part.color + \
                    "_c" + str(part_count).zfill(2)
                part_count += 1

                if part.flipped:
                    roll = math.pi
                else:
                    roll = 0

                yaw = convert_pi_string_to_float(part.rotation)
                q = quaternion_from_euler(roll, 0, yaw)
                rel_pose = Pose()

                rel_pose.position.y = offset * (self.conveyor_width/2)

                if part.flipped:
                    rel_pose.position.z = part.height

                rel_pose.orientation.w = q[0]
                rel_pose.orientation.x = q[1]
                rel_pose.orientation.y = q[2]
                rel_pose.orientation.z = q[3]

                world_pose = do_transform_pose(rel_pose, conveyor_transform)

                xyz = [world_pose.position.x,
                       world_pose.position.y, world_pose.position.z]
                rpy = euler_from_quaternion(world_pose.orientation)

                self.conveyor_parts_to_spawn.append(PartSpawnParams(
                    part_name, part.type, part.color, xyz=xyz, rpy=rpy))

        if len(self.conveyor_parts_to_spawn) > 0:
            # Create Spawn Timer
            self.conveyor_spawn_timer = self.create_timer(
                self.spawn_rate, self.spawn_conveyor_part)
            return True

        return False

    def conveyor_status(self, msg: ConveyorBeltState):
        self.conveyor_enabled = msg.enabled

    def spawn_entity(self, params: SpawnParams, wait=True) -> bool:
        self.spawn_client.wait_for_service()

        # self.get_logger().info(f'Spawning: {params.name}')

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        future = self.spawn_client.call_async(req)

        if wait:
            rclpy.spin_until_future_complete(self, future)
            return future.result().success
        else:
            return True

    def parse_part_info(self, part_info):
        part = PartInfo()

        try:
            part.type = part_info['type']
            part.height = PartInfo.part_heights[part.type]
        except KeyError:
            self.get_logger().warn("Part type is not specified")
            return (False, part)

        try:
            part.color = part_info['color']
        except KeyError:
            self.get_logger().warn("Part color is not specified")
            return (False, part)

        try:
            part.rotation = str(part_info['rotation'])
        except KeyError:
            pass

        try:
            part.flipped = part_info['flipped']
            if not type(part.flipped) == bool:
                self.get_logger().warn("flipped parameter should be either true or false")
                part.flipped = False
        except KeyError:
            pass

        if not part.type in PartSpawnParams.part_types:
            self.get_logger().warn(
                f"{part_info['type']} is not a valid part type")
            return (False, part)

        if not part.color in PartSpawnParams.colors:
            self.get_logger().warn(
                f"{part_info['color']} is not a valid part color")
            return (False, part)

        return (True, part)

    def read_yaml(self, path):
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}

    def pause_physics(self):
        self.pause_client.wait_for_service()

        request = Empty.Request()

        self.pause_client.call_async(request)

    def unpause_physics(self):
        self.unpause_client.wait_for_service()

        request = Empty.Request()

        self.unpause_client.call_async(request)

    def publish_bin_parts(self):
        self.bin_parts_publisher.publish(self.bin_parts)

    def publish_conveyor_parts(self):
        self.conveyor_parts_publisher.publish(self.conveyor_parts)
