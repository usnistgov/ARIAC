try:
    import customtkinter as ctk
    from customtkinter import *
except:
    print("ERROR: customtkinter not installed")
    quit()    
from tkinter import *
import tkinter as tk
from tkinter import ttk, filedialog
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi
import random
import string
import yaml
from copy import copy
from datetime import datetime

from ariac_msgs.msg import (
    Part as PartMsg,
    OrderCondition as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    KittingPart as KittingPartMsg,
    KittingTask as KittingTaskMsg,
    AssemblyTask as AssemblyTaskMsg,
    CombinedTask as CombinedTaskMsg,
    Condition as ConditionMsg,
    FaultyPartChallenge as FaultyPartChallengeMsg,
    DroppedPartChallenge as DroppedPartChallengeMsg,
    SensorBlackoutChallenge as SensorBlackoutChallengeMsg,
    RobotMalfunctionChallenge as RobotMalfunctionChallengeMsg,
    HumanChallenge as HumanChallengeMsg,
    Challenge as ChallengeMsg
)
from geometry_msgs.msg import PoseStamped, Vector3
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ariac_gui.utils import (build_competition_from_file, quaternion_from_euler, require_int, 
                             require_num, 
                             BinPart, 
                             ConveyorPart,
                             CompetitionClass,
                             SLIDER_STR, 
                             SLIDER_VALUES, 
                             ORDER_TYPES)

FRAMEWIDTH=700
FRAMEHEIGHT=900
LEFT_COLUMN=1
MIDDLE_COLUMN = 2
RIGHT_COLUMN = 3

PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']

#Options for kitting trays
KITTING_TRAY_OPTIONS = [""]+[str(i) for i in range(10)]

# Bin menu items
ALL_BINS=['bin'+str(i) for i in range(1,9)]

# Conveyor order types
CONVEYOR_ORDERS = ["random", "sequential"]

# Menu images
GUI_PACKAGE = get_package_share_directory('ariac_gui')
MENU_IMAGES = {part_label:Image.open(GUI_PACKAGE + f"/resource/{part_label}.png") for part_label in ["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]}

QUADRANTS=["1","2","3","4"]
AGV_OPTIONS=["1","2","3","4"]
AGV_TITLES=[f"agv{i}" for i in range(1,5)]
ASSEMBLY_STATIONS=["as1","as2","as3","as4"]
CONDITION_TYPE=['time','part_place']
TRAY_IDS=[str(i) for i in range(10)]

SENSORS = ["break_beam", "proximity", "laser_profiler", "lidar", "camera", "logical_camera"]
ROBOTS = ["floor_robot", "ceiling_robot"]
BEHAVIORS = ["antagonistic","indifferent","helpful"]
CHALLENGE_TYPES = ["faulty_part", "dropped_part", "sensor_blackout", "robot_malfunction","human"]


_part_color_ints = {"RED":0,
                    "GREEN":1,
                    "BLUE":2,
                    "ORANGE":3,
                    "PURPLE":4}
    
_part_type_ints = {"BATTERY":10,
                    "PUMP":11,
                    "SENSOR":12,
                    "REGULATOR":13}

_part_color_str = {_part_color_ints[key]:key.lower() for key in _part_color_ints.keys()}
    
_part_type_str = {_part_type_ints[key]:key.lower() for key in _part_type_ints.keys()}


_assembly_part_poses = {}

_assembly_part_install_directions = {}

_assembly_part_pose_and_direction_dicts = {"SENSOR":{"assembled_pose":{"xyz":[-0.1,0.395,0.045],
                                                                       "rpy":[0,0,'-pi/2']
                                                                       },
                                                                       "assembly_direction":[0,-1,0]},
                                           "BATTERY":{"assembled_pose":{"xyz":[-0.15,0.035,0.043],
                                                                       "rpy":[0,0,'pi/2']
                                                                       },
                                                                       "assembly_direction":[0,1,0]},
                                            "REGULATOR":{"assembled_pose":{"xyz":[0.175,-0.223,0.215],
                                                                       "rpy":['pi/2',0,'-pi/2']
                                                                       },
                                                                       "assembly_direction":[0,0,-1]},
                                            "PUMP":{"assembled_pose":{"xyz":[0.14,0.0,0.02],
                                                                       "rpy":[0,0,'-pi/2']
                                                                       },
                                                                       "assembly_direction":[0,0,-1]}}

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

class GUI_CLASS(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
        ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
        
        self.title("NIST ARIAC CONFIGURATION GUI")

        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(100, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(4, weight=1)

        s = ttk.Style()
        s.theme_use('clam')
        s.configure('TNotebook', font='Arial Bold')

        self.notebook = ttk.Notebook(self)

        # Loaded file information
        self.load_through_file_flag = False
        self.file_name = ""
        self.save_flag = True

        # Trial files location

        self.trials_file_location = ''
        try:
            self.ws = ''.join(str(item) + '/' for item in get_package_prefix("ariac_gazebo").split("/")[:-2])
            self.pkgs = [ f.name for f in os.scandir(self.ws + '/src/') if f.is_dir() ]
            for pkg in self.pkgs:
                if pkg.lower().count('ariac') >= 1:
                    temp_folder = self.ws + 'src/' + pkg + '/ariac_gazebo/config/trials/'
                    if os.path.exists(temp_folder):
                        self.trials_file_location = temp_folder
                        break
        except:
            pass

        # Setup info
        self.time_limit = ctk.StringVar()
        self.trial_name = ctk.StringVar()
        self.author = ctk.StringVar()
        self.time_limit.set('0')
        self.trial_name.set('')
        self.author.set('')
        self.time_limit.trace('w', partial(require_int, self.time_limit))

        # AGV parts info
        self.available_quadrants = {f"agv_{i}":[j for j in range(1,5)] for i in range(1,5)}
        self.all_present_parts = []
        self.needed_kitting_trays = [0 for _ in range(4)]

        # Kitting tray info
        self.kitting_tray_selections = [ctk.StringVar() for _ in range(6)]

        # Assembly insert info
        self.assembly_insert_rotations = [ctk.DoubleVar() for _ in range(4)]

        # Bin parts info
        self.current_bin_parts = {f"bin{i}":["" for _ in range(9)] for i in range(1,9)}
        self.bin_parts = {f"bin{i}":[BinPart() for _ in range(9)] for i in range(1,9)}
        self.bin_parts_counter = ctk.StringVar()
        self.bin_parts_counter.set('0')
        self.current_bin_canvas_elements = []

        # Conveyor parts info
        self.current_conveyor_parts = []
        self.conveyor_parts = []
        self.conveyor_parts_counter = ctk.StringVar()
        self.conveyor_parts_counter.set('0')
        self.present_conveyor_widgets = []
        self.current_conveyor_canvas_elements = []
        self.has_parts = ctk.StringVar()
        self.has_parts.set('0')

        # Order widgets
        self.current_left_order_widgets = []
        self.current_right_order_widgets = []
        self.current_main_order_widgets = []
        self.current_order_part_widgets = []

        # Order row indeces
        self.left_row_index = 1
        self.right_row_index = 3

        # Order structure
        self.order_counter = ctk.StringVar()
        self.order_counter.set('0')
        self.used_ids = []
        self.order_info = {}
        self.order_info["order_type"] = ctk.StringVar()
        self.order_info["priority"] = ctk.StringVar()
        self.order_info["announcement_type"] = ctk.StringVar()
        
        self.order_info["announcement"] = {}
        self.order_info["announcement"]["time_condition"] = ctk.StringVar()
        self.order_info["announcement"]["color"] = ctk.StringVar()
        self.order_info["announcement"]["type"] = ctk.StringVar()
        self.order_info["announcement"]["agv"] = ctk.StringVar()
        self.order_info["announcement"]["submission_id"] = ctk.StringVar()

        self.order_info["kitting_task"] = {}
        self.order_info["kitting_task"]["agv_number"] = ctk.StringVar()
        self.order_info["kitting_task"]["tray_id"] = ctk.StringVar()
        self.order_info["kitting_task"]["parts"] = []

        self.order_info["assembly_task"] = {}
        self.order_info["assembly_task"]["agv_numbers"] = [ctk.StringVar() for _ in range(4)]
        self.order_info["assembly_task"]["station"] = ctk.StringVar()
        self.order_info["assembly_task"]["parts"] = []

        self.order_info["combined_task"] = {}
        self.order_info["combined_task"]["station"] = ctk.StringVar()
        self.order_info["combined_task"]["parts"] = []

        self.reset_order()
        
        for var in self.order_info["assembly_task"]["agv_numbers"]:
            var.trace('w', self.activate_assembly_save)

        self.current_orders = []

        # Challenges widgets
        self.current_challenges_widgets = []
        self.current_challenges_condition_widgets = []
        self.challenges_counter = ctk.StringVar()
        self.challenges_counter.set("0")

        # Dropped part challenge variables
        self.dropped_part_info = {}
        self.dropped_part_info["robot"] = ctk.StringVar()
        self.dropped_part_info["type"] = ctk.StringVar()
        self.dropped_part_info["color"] = ctk.StringVar()
        self.dropped_part_info["drop_after"] = ctk.StringVar()
        self.dropped_part_info["drop_after"].trace('w', partial(require_int, self.dropped_part_info["drop_after"]))
        self.dropped_part_info["delay"] = ctk.StringVar()
        self.dropped_part_info["delay"].trace('w', partial(require_num, self.dropped_part_info["delay"]))

        # Robot malfunction challenge variables
        self.robot_malfunction_info = {}
        self.robot_malfunction_info["duration"] = ctk.StringVar()
        self.robot_malfunction_info["duration"].trace('w', partial(require_num, self.robot_malfunction_info["duration"]))
        self.robot_malfunction_info["floor_robot"] = ctk.StringVar()
        self.robot_malfunction_info["ceiling_robot"] = ctk.StringVar()

        # Sensor blackout challenge variables
        self.sensor_blackout_info = {}
        self.sensor_blackout_info["duration"] = ctk.StringVar()
        self.sensor_blackout_info["duration"].trace('w', partial(require_num, self.sensor_blackout_info["duration"]))
        self.sensor_blackout_info["sensors_to_disable"] = {SENSORS[i]:ctk.StringVar() for i in range(len(SENSORS))}

        # Faulty part challenge variables
        self.faulty_part_info = {}
        self.faulty_part_info["order_id"] = ctk.StringVar()
        self.faulty_part_info["quadrants"] = [ctk.StringVar() for _ in range(4)]

        # Human challenge variables
        self.human_info = {}
        self.human_info["behavior"] = ctk.StringVar()

        # Challenge condition variables
        self.challenge_condition_type = ctk.StringVar()
        self.challenge_condition_info = {}
        self.challenge_condition_info["time_condition"] = ctk.StringVar()
        self.challenge_condition_info["color"] = ctk.StringVar()
        self.challenge_condition_info["type"] = ctk.StringVar()
        self.challenge_condition_info["agv"] = ctk.StringVar()
        self.challenge_condition_info["submission_id"] = ctk.StringVar()
        self.challenge_condition_info["time_condition"].trace('w',partial(require_num,self.challenge_condition_info["time_condition"]))

        # List to hold saved challenges
        self.current_challenges = []
        self.current_challenges_row = 1

        # Menu tabs
        self.setup_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.setup_frame.pack(fill='both',expand=True)
        self.notebook.add(self.setup_frame,text="Setup")
        self.add_setup_widgets_to_frame()

        self.kitting_tray_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.kitting_tray_frame.pack(fill='both',expand=True)
        self.notebook.add(self.kitting_tray_frame,text="Kitting Trays")
        self.add_kitting_trays_widgets_to_frame()

        self.assembly_inserts_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.assembly_inserts_frame.pack(fill='both',expand=True)
        self.notebook.add(self.assembly_inserts_frame,text="Assembly Inserts")
        self.add_assembly_inserts_widgets_to_frame()

        self.bin_parts_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.bin_parts_frame.pack(fill='both',expand=True)
        self.notebook.add(self.bin_parts_frame,text="Bin Parts")
        self.add_bin_parts_widgets_to_frame()

        self.conveyor_parts_frame = ttk.Frame(self.notebook,width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.conveyor_parts_frame.pack(fill='both',expand=True)
        self.notebook.add(self.conveyor_parts_frame, text="Conveyor Parts")
        self.add_conveyor_parts_widgets_to_frame()

        self.orders_frame = ttk.Frame(self.notebook,width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.orders_frame.pack(fill='both',expand=True)
        self.notebook.add(self.orders_frame, text="Orders")
        self.add_order_widgets_to_frame()

        self.challenges_frame = ttk.Frame(self.notebook,width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.challenges_frame.pack(fill='both',expand=True)
        self.notebook.add(self.challenges_frame, text="Challenges")
        self.add_challenges_widgets_to_frame()

        self.save_file_button = ctk.CTkButton(self, text="Save file", command=self.choose_save_location)        

        self._build_assembly_parts_pose_direction()

        # File dicts
        self.kitting_trays_dict = {}
        self.assembly_inserts_dict = {}
        self.agv_parts_dict = {}
        self.bin_parts_dict = {}
        self.conveyor_parts_dict = {}
        self.orders_dict = {}
        self.challenges_dict = {}

        self.open_initial_window()
    
    # =======================================================
    #            Load gui from a previous file
    # =======================================================
    def open_main_window(self):
        self.initial_label.grid_forget()
        self.load_file_button.grid_forget()
        self.new_file_button.grid_forget()
        self.notebook.grid(pady=10,column=MIDDLE_COLUMN,sticky=tk.E+tk.W+tk.N+tk.S)
        self.save_file_button.grid(pady=10,column=MIDDLE_COLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

    def open_initial_window(self):
        self.initial_label = ctk.CTkLabel(self, text="Would you like to open an existing file or create a new one?")
        self.initial_label.grid(column = LEFT_COLUMN, row = 1, columnspan = 3, pady = 85, padx=75)
        self.load_file_button = ctk.CTkButton(self, text="Load file", command = self._load_file)
        self.load_file_button.grid(column = LEFT_COLUMN, row = 2, pady = 85)
        self.new_file_button = ctk.CTkButton(self, text="New file", command = self.open_main_window)
        self.new_file_button.grid(column = RIGHT_COLUMN, row = 2, pady = 85)

    def _load_file(self):
        file_to_open=filedialog.askopenfile("r", filetypes =[('Yaml Files', '*.yaml')], initialdir=self.trials_file_location,title='Open ARIAC configuration',)
        try:
            with open(file_to_open.name) as f:
                yaml_dict = yaml.load(f, Loader=yaml.SafeLoader)
            self.trial_name.set(file_to_open.name.split("/")[-1].replace(".yaml",""))
            self._load_options_from_competition_class(build_competition_from_file(yaml_dict))
            self.load_through_file_flag = True
            self.file_name = file_to_open.name
            self.open_main_window()
        except:
            try:
                if file_to_open.name != None:
                    print("Unable to open or parse file")
            except:
                pass

    def _load_options_from_competition_class(self, competition: CompetitionClass):
        self.save_file_button.configure(command=self.run_overwrite_window)
        self.time_limit.set(competition.competition["time_limit"])

        for i in range(len(competition.competition["kitting_trays"]["slots"])):
            self.kitting_tray_selections[competition.competition["kitting_trays"]["slots"][i]-1].set(str(competition.competition["kitting_trays"]["tray_ids"][i]))
        
        for i in range(len(competition.competition["assembly_insert_rotations"])):
            self.assembly_insert_rotations[i].set(competition.competition["assembly_insert_rotations"][i])

        self.bin_parts = competition.competition["bin_parts"]
        self.current_bin_parts = competition.competition["current_bin_parts"]
        for bin_number in range(1,9):
            for slot in range(9):
                if self.current_bin_parts[f"bin{bin_number}"][slot]!="":
                    self.all_present_parts.append(_part_color_str[self.bin_parts[f"bin{bin_number}"][slot].part.color]+" "+_part_type_str[self.bin_parts[f"bin{bin_number}"][slot].part.type])
        self.bin_parts_counter.set(str(len([1 for part in self.current_bin_parts if part !=""])))

        if len(competition.competition["conveyor_belt"]["parts_to_spawn"])>0:
            self.has_parts.set("1")
        self.conveyor_setup_vals["active"].set(competition.competition["conveyor_belt"]["active"])
        self.conveyor_setup_vals["spawn_rate"].set(competition.competition["conveyor_belt"]["spawn_rate"])
        self.conveyor_setup_vals["order"].set(competition.competition["conveyor_belt"]["order"])
        self.conveyor_parts = competition.competition["conveyor_belt"]["parts_to_spawn"]
        self.current_conveyor_parts = competition.competition["conveyor_belt"]["current_conveyor_parts"]
        self.conveyor_parts_counter.set(str(len(self.conveyor_parts)))
        for part in self.conveyor_parts:
            for _ in range(part.part_lot.quantity):
                self.all_present_parts.append(_part_color_str[part.part_lot.part.color]+" "+_part_type_str[part.part_lot.part.type])

        self.current_orders = competition.competition["orders"]
        self.order_counter.set(str(len(self.current_orders)))
        if len(self.current_orders)>0:
            if 'submission' not in CONDITION_TYPE:
                CONDITION_TYPE.append('submission')
        for order in self.current_orders:
            self.used_ids.append(order.id)
        
        self.current_challenges = competition.competition["challenges"]
        self.challenges_counter.set(str(len(self.current_challenges)))

        self.show_main_order_menu()
    
    # =======================================================
    #           Assembly Part Orientation Setup
    # =======================================================
    def _build_assembly_parts_pose_direction(self):
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

    # =======================================================
    #            Configuration Setup Functions
    # =======================================================
    def add_setup_widgets_to_frame(self):
        time_limit_label = ctk.CTkLabel(self.setup_frame, text="Enter the time limit (-1 for no time limit):")
        time_limit_label.pack()
        time_limit_entry = ctk.CTkEntry(self.setup_frame, textvariable=self.time_limit)
        time_limit_entry.pack()
        trial_name_label = ctk.CTkLabel(self.setup_frame, text="Enter the trial name:")
        trial_name_label.pack()
        trial_name_entry = ctk.CTkEntry(self.setup_frame, textvariable=self.trial_name)
        trial_name_entry.pack()
        author_label = ctk.CTkLabel(self.setup_frame, text="Enter the name of the author:")
        author_label.pack()
        author_entry = ctk.CTkEntry(self.setup_frame, textvariable=self.author)
        author_entry.pack()
    # =======================================================
    #               Kitting Tray Functions
    # =======================================================
    def add_kitting_trays_widgets_to_frame(self):
        self.kitting_tray_frame.grid_rowconfigure(0, weight=1)
        self.kitting_tray_frame.grid_rowconfigure(100, weight=1)
        self.kitting_tray_frame.grid_columnconfigure(0, weight=1)
        self.kitting_tray_frame.grid_columnconfigure(4, weight=1)
        tray_label = ctk.CTkLabel(self.kitting_tray_frame,text="Select the tray ids for each slot")
        tray_label.grid(row=1, column = MIDDLE_COLUMN)
        kitting_tray_canvas = Canvas(self.kitting_tray_frame, height=400)
        kitting_tray_canvas.create_rectangle(10, 10, 170, 310, 
                                outline = "black", fill = "#f6f6f6",
                                width = 2)
        kitting_tray_canvas.create_rectangle(200, 10, 360, 310, 
                                outline = "black", fill = "#f6f6f6",
                                width = 2)
        menu_coordinates = [(95,60),(95,160),(95,260),(285,60),(285,160),(285,260)]
        label_coordinates = [(20,60),(20,160),(20,260),(210,60),(210,160),(210,260)]
        
        for i in self.kitting_tray_selections:i.set(KITTING_TRAY_OPTIONS[0])
        tray_menus = [ctk.CTkOptionMenu(self.kitting_tray_frame,
                                        variable=self.kitting_tray_selections[i],
                                        values=KITTING_TRAY_OPTIONS,
                                        fg_color = "#e2e2e2",
                                        text_color="black",
                                        button_color="#d3d3d3",
                                        button_hover_color="#9e9e9e",
                                        anchor='center') for i in range(6)]
        for i in range(6):
            kitting_tray_canvas.create_window(label_coordinates[i], window=ctk.CTkLabel(self.kitting_tray_frame, text=f"{i+1}:"))
            kitting_tray_canvas.create_window(menu_coordinates[i], window = tray_menus[i])
        kitting_tray_canvas.create_window((90,325),window=ctk.CTkLabel(self.kitting_tray_frame,text="kts_1"))
        kitting_tray_canvas.create_window((280,325),window=ctk.CTkLabel(self.kitting_tray_frame,text="kts_2"))
        kitting_tray_canvas.grid(row = 3,column = MIDDLE_COLUMN, sticky = "we")

    def kitting_trays_to_dict(self):
        slots = []
        trays = []
        for i in range(len(self.kitting_tray_selections)):
            tray = self.kitting_tray_selections[i].get()
            if tray != "":
                slots.append(i+1)
                trays.append(int(tray))
        self.kitting_trays_dict["kitting_trays"] = {"tray_ids":trays, "slots":slots}
    
    # =======================================================
    #               Assembly Inserts Functions
    # =======================================================
    def add_assembly_inserts_widgets_to_frame(self):
        self.assembly_inserts_frame.grid_rowconfigure(0, weight=1)
        self.assembly_inserts_frame.grid_rowconfigure(100, weight=1)
        self.assembly_inserts_frame.grid_columnconfigure(0, weight=1)
        self.assembly_inserts_frame.grid_columnconfigure(4, weight=1)
        for i in range(len(ASSEMBLY_STATIONS)):
            label_text = f"{ASSEMBLY_STATIONS[i]} rotation value: {SLIDER_STR[SLIDER_VALUES.index(self.assembly_insert_rotations[i].get())]}"
            station_label = ctk.CTkLabel(self.assembly_inserts_frame, text=label_text)
            station_label.grid(row = i+1, column = LEFT_COLUMN, padx = 10, pady = 15)
            rotation_slider = ctk.CTkSlider(self.assembly_inserts_frame, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=self.assembly_insert_rotations[i], orientation="horizontal")
            rotation_slider.grid(row = i+1, column = RIGHT_COLUMN, padx = 10, pady = 15)
            self.assembly_insert_rotations[i].trace('w', partial(self.nearest_slider_value_assembly_inserts,self.assembly_insert_rotations[i], rotation_slider,station_label, ASSEMBLY_STATIONS[i]))
        filler_label = ctk.CTkLabel(self.assembly_inserts_frame, text = " "*60) # So the menu does not move when the labels change
        filler_label.grid(row = 10, column = LEFT_COLUMN)
    
    def assembly_inserts_to_dict(self):
        self.assembly_inserts_dict["assembly_inserts"] = {ASSEMBLY_STATIONS[i]:SLIDER_STR[SLIDER_VALUES.index(self.assembly_insert_rotations[i].get())] for i in range(len(ASSEMBLY_STATIONS))}
    
    # =======================================================
    #                 AGV Parts Functions
    # =======================================================
    def udpate_avalable_quadrants(self):
        for order in self.current_orders:
            order : OrderMsg
            if order.type == 0:
                self.needed_kitting_trays[order.kitting_task.agv_number-1] = order.kitting_task.tray_id
                for part in order.kitting_task.parts:
                    part : KittingPartMsg
                    self.available_quadrants[f"agv_{order.kitting_task.agv_number}"].remove(part.quadrant)
    
    def agv_parts_to_dict(self):
        parts_on_agv = {f"agv{i}":[] for i in range(1,5)}
        needed_parts = self.get_needed_parts()
        num_parts_needed = len(needed_parts)
        if len(needed_parts) == 0:
            return
        for index in range(len(needed_parts)-1,-1,-1):
            for agv_num in needed_parts[index][1]:
                for quadrant in self.available_quadrants[f"agv_{agv_num}"]:
                    parts_on_agv[f"agv{agv_num}"].append((needed_parts[index][0],quadrant))
                    self.available_quadrants[f"agv_{agv_num}"].remove(quadrant)
                    del needed_parts[index]
                    break
                if len(needed_parts)==index:
                    break
        if num_parts_needed>0:
            for agv_key in parts_on_agv.keys():
                if len(parts_on_agv[agv_key])>0:
                    self.agv_parts_dict[agv_key] = {}
                    self.agv_parts_dict[agv_key]["tray_id"] = self.needed_kitting_trays[AGV_TITLES.index(agv_key)]
                    self.agv_parts_dict[agv_key]["parts"] = []
                    for part in parts_on_agv[agv_key]:
                        temp_dict = {}
                        temp_dict["type"] = part[0].split(" ")[0]
                        temp_dict["color"] = part[0].split(" ")[1]
                        temp_dict["quadrant"] = part[1]
                        self.agv_parts_dict[agv_key]["parts"].append(temp_dict)

    def get_needed_parts(self):
        needed_parts = []
        for order in self.current_orders:
            order : OrderMsg
            if order.type == 1:
                for part in order.assembly_task.parts:
                    part : AssemblyPartMsg
                    part_str = _part_color_str[part.part.color]+" "+_part_type_str[part.part.type]
                    if part_str in self.all_present_parts:
                        self.all_present_parts.remove(part_str)
                    else:
                        needed_parts.append((part_str, order.assembly_task.agv_numbers))
        return needed_parts

    # =======================================================
    #                 Bin Parts Functions
    # =======================================================
    def add_bin_parts_widgets_to_frame(self):
        self.bin_parts_frame.grid_rowconfigure(0, weight=1)
        self.bin_parts_frame.grid_rowconfigure(100, weight=1)
        self.bin_parts_frame.grid_columnconfigure(0, weight=1)
        self.bin_parts_frame.grid_columnconfigure(4, weight=1)
        bin_selection = ctk.StringVar()
        bin_selection.set(ALL_BINS[0])
        bin_label = ctk.CTkLabel(self.bin_parts_frame,text="Select the bin you would like to add parts to:")
        bin_label.grid(column = MIDDLE_COLUMN)
        bin_menu = ctk.CTkOptionMenu(self.bin_parts_frame,
                                        variable=bin_selection,
                                        values=ALL_BINS,
                                        fg_color = "#e2e2e2",
                                        text_color="black",
                                        button_color="#d3d3d3",
                                        button_hover_color="#9e9e9e",
                                        anchor='center',
                                        )
        bin_menu.grid(column = MIDDLE_COLUMN)

        bin_parts_canvas = Canvas(self.bin_parts_frame, height=375)
        
        bin_parts_canvas.create_rectangle(75, 10, 375, 310, 
                                outline = "black", fill = "#60c6f1",
                                width = 2)
        self.show_grid(bin_selection,bin_parts_canvas,self.bin_parts_frame)
        bin_parts_canvas.grid(column = MIDDLE_COLUMN, sticky = "we")
        add_multiple_parts_button = ctk.CTkButton(self.bin_parts_frame,text="Add multiple parts",command=partial(self.add_multiple_parts,bin_selection.get()))
        add_multiple_parts_button.grid(column = MIDDLE_COLUMN, pady = 15)
        flipped_meaning_label = ctk.CTkLabel(self.bin_parts_frame, text="When a part is flipped, an \"F\" will show up in the bottom right of the part image.")
        flipped_meaning_label.grid(column = MIDDLE_COLUMN, pady = 10)
        bin_selection.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))
        self.bin_parts_counter.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))

    def show_grid(self,bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk):
        button_coordinates = [(125,60),(225,60),(325,60),
                            (125,160),(225,160),(325,160),
                            (125,260),(225,260),(325,260)]
        flipped_label_coordinates = [(coord[0]+30, coord[1]+30) for coord in button_coordinates]
        current_bin_slot_widgets = []
        current_flipped_labels = ["" for _ in range(len(flipped_label_coordinates))]
        for i in range(len(button_coordinates)):
            if self.current_bin_parts[bin_selection.get()][i]=="":
                current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(self.add_bin_part, bin_selection.get(), i),
                                                            image=ctk.CTkImage(MENU_IMAGES["plus"],size=(75,75)),
                                                            fg_color="transparent",bg_color="#4FA2C6",hover_color="#458DAC",width=1))
            else:
                current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(self.add_bin_part, bin_selection.get(), i),
                                                            image=ctk.CTkImage(MENU_IMAGES[self.current_bin_parts[bin_selection.get()][i]].rotate(self.bin_parts[bin_selection.get()][i].rotation*180/pi),size=(75,75)),
                                                            fg_color="transparent",bg_color="#60c6f1",hover_color="#60c6f1",width=1))
            if self.bin_parts[bin_selection.get()][i].flipped == "1":
                current_flipped_labels[i]=(ctk.CTkLabel(main_wind, text="F",bg_color="#60c6f1"))
        for i in range(len(current_bin_slot_widgets)):
            self.current_bin_canvas_elements.append(canvas.create_window(button_coordinates[i], window = current_bin_slot_widgets[i]))
            if current_flipped_labels[i]!="":
                self.current_bin_canvas_elements.append(canvas.create_window(flipped_label_coordinates[i], window=current_flipped_labels[i]))
            
    def update_bin_grid(self,bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk,_,__,___):
        for i in self.current_bin_canvas_elements:
            canvas.delete(i)
        self.current_bin_canvas_elements.clear()
        self.show_grid(bin_selection,canvas,main_wind)

    def add_bin_part(self,bin, index):
        bin_vals = {}
        add_part_bin_window = ctk.CTkToplevel()
        add_part_bin_window.geometry("400x400 + 700 + 300")
        bin_vals["color"] = ctk.StringVar()
        
        bin_vals["pType"] = ctk.StringVar()
        
        bin_vals["rotation"] = ctk.DoubleVar()
        
        bin_vals["flipped"] = ctk.StringVar()
        if self.current_bin_parts[bin][index] == "":
            bin_vals["color"].set(PART_COLORS[0])
            bin_vals["pType"].set(PART_TYPES[0])
            bin_vals["rotation"].set(0.0)
            bin_vals["flipped"].set("0")
        else:
            bin_vals["color"].set(_part_color_str[self.bin_parts[bin][index].part.color])
            bin_vals["pType"].set(_part_type_str[self.bin_parts[bin][index].part.type])
            bin_vals["rotation"].set(self.bin_parts[bin][index].rotation)
            bin_vals["flipped"].set(self.bin_parts[bin][index].flipped)
        color_menu = ctk.CTkOptionMenu(add_part_bin_window, variable=bin_vals["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_part_bin_window, variable=bin_vals["pType"],values=PART_TYPES)
        type_menu.pack()
        rotation_label = ctk.CTkLabel(add_part_bin_window, text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(bin_vals['rotation'].get())]}")
        rotation_label.pack()
        rotation_slider = ctk.CTkSlider(add_part_bin_window, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=bin_vals["rotation"], orientation="horizontal")
        rotation_slider.pack(pady=5)
        bin_vals["rotation"].trace('w', partial(self.nearest_slider_value, bin_vals["rotation"], rotation_slider,rotation_label))
        flipped_cb = ctk.CTkCheckBox(add_part_bin_window,text="Flipped",variable=bin_vals["flipped"], onvalue="1", offvalue="0", height=1, width=20)
        flipped_cb.pack(pady=5)
        if self.current_bin_parts[bin][index]!="":
            remove_part_button = ctk.CTkButton(add_part_bin_window, text="Remove part", command = partial(self.remove_part_from_bin,bin,index,add_part_bin_window))
            remove_part_button.pack()
        back_button = ctk.CTkButton(add_part_bin_window,text="Back",command=add_part_bin_window.destroy)
        back_button.pack()
        save_button = ctk.CTkButton(add_part_bin_window,text="Save part",command=partial(self.save_bin_part,bin,index,add_part_bin_window,bin_vals))
        save_button.pack()

    def remove_part_from_bin(self, bin, index, window):
        self.current_bin_parts[bin][index] = ""
        self.bin_parts[bin][index] = BinPart()
        self.bin_parts_counter.set(str(int(self.bin_parts_counter.get())-1))
        window.destroy()

    def save_bin_part(self,bin, index, window:ctk.CTkToplevel, bin_vals):
        color = bin_vals["color"].get()
        pType = bin_vals["pType"].get()
        self.all_present_parts.append(f"{color} {pType}")
        self.current_bin_parts[bin][index]=color+pType
        temp_part = PartMsg()
        temp_part.color = _part_color_ints[color.upper()]
        temp_part.type = _part_type_ints[pType.upper()]
        self.bin_parts[bin][index].part = temp_part
        self.bin_parts[bin][index].rotation = bin_vals["rotation"].get() 
        self.bin_parts[bin][index].flipped = bin_vals["flipped"].get()
        self.bin_parts_counter.set(str(int(self.bin_parts_counter.get())+1))
        window.destroy()

    def add_multiple_parts(self,bin):
        slot_widgets = []
        slot_values = [ctk.StringVar() for _ in range(9)]
        for val in slot_values: val.set('-1')
        add_parts_bin_window = ctk.CTkToplevel()
        add_parts_bin_window.geometry("400x450 + 700 + 300")
        for i in range(9):
            slot_widgets.append(ctk.CTkCheckBox(add_parts_bin_window,text=f"Slot {i+1}", variable=slot_values[i], onvalue=str(i), offvalue="-1", height=1, width=20))
            slot_widgets[-1].pack()
        bin_vals = {}
        
        bin_vals["color"] = ctk.StringVar()
        bin_vals["color"].set(PART_COLORS[0])
        bin_vals["pType"] = ctk.StringVar()
        bin_vals["pType"].set(PART_TYPES[0])
        bin_vals["rotation"] = ctk.DoubleVar()
        bin_vals["rotation"].set(0.0)
        bin_vals["flipped"] = ctk.StringVar()
        bin_vals["flipped"].set("0")
        color_menu = ctk.CTkOptionMenu(add_parts_bin_window, variable=bin_vals["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_parts_bin_window, variable=bin_vals["pType"],values=PART_TYPES)
        type_menu.pack()
        rotation_label = ctk.CTkLabel(add_parts_bin_window, text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(bin_vals['rotation'].get())]}")
        rotation_label.pack()
        rotation_slider = ctk.CTkSlider(add_parts_bin_window, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=bin_vals["rotation"], orientation="horizontal")
        rotation_slider.pack(pady=5)
        bin_vals["rotation"].trace('w', partial(self.nearest_slider_value, bin_vals["rotation"], rotation_slider,rotation_label))
        flipped_cb = ctk.CTkCheckBox(add_parts_bin_window,text="Flipped",variable=bin_vals["flipped"], onvalue="1", offvalue="0", height=1, width=20)
        flipped_cb.pack(pady=5)
        back_button = ctk.CTkButton(add_parts_bin_window,text="Back",command=add_parts_bin_window.destroy)
        back_button.pack()
        save_button = ctk.CTkButton(add_parts_bin_window,text="Save part",command=partial(self.save_bin_parts,bin,slot_values,add_parts_bin_window,bin_vals))
        save_button.pack()

    def save_bin_parts(self,bin, slot_values, window:ctk.CTkToplevel, bin_vals):
        color = bin_vals["color"].get()
        pType = bin_vals["pType"].get()
        slot_indices = [int(val.get()) for val in slot_values if val.get()!="-1"]
        for index in slot_indices:
            self.all_present_parts.append(f"{color} {pType}")
            self.current_bin_parts[bin][index]=color+pType
            temp_part = PartMsg()
            temp_part.color = _part_color_ints[color.upper()]
            temp_part.type = _part_type_ints[pType.upper()]
            self.bin_parts[bin][index].part = temp_part
            self.bin_parts[bin][index].rotation = bin_vals["rotation"].get() 
            self.bin_parts[bin][index].flipped = bin_vals["flipped"].get()
        self.bin_parts_counter.set(str(int(self.bin_parts_counter.get())+len(slot_indices)))
        window.destroy()

    def bin_part_equal(self, part_1 : BinPart, part_2 : BinPart)->bool:
        if part_1.part.color == part_2.part.color:
            if part_1.part.type == part_2.part.type:
                if part_1.rotation == part_2.rotation:
                    if part_1.flipped == part_2.flipped:
                        return True
        return False
    
    def bin_parts_to_dict(self):
        for bin in self.current_bin_parts.keys():
            used_slots = []
            for slot in range(9):
                if self.current_bin_parts[bin][slot]!="" and slot not in used_slots:
                    temp_slots = []
                    for i in range(slot,9):
                        if self.bin_part_equal(self.bin_parts[bin][slot],self.bin_parts[bin][i]):
                            temp_slots.append(i+1)
                            used_slots.append(i)
                    temp_bin_part_dict = {}
                    temp_bin_part_dict["type"] = _part_type_str[self.bin_parts[bin][slot].part.type]
                    temp_bin_part_dict["color"] = _part_color_str[self.bin_parts[bin][slot].part.color]
                    temp_bin_part_dict["rotation"] = SLIDER_STR[SLIDER_VALUES.index(self.bin_parts[bin][slot].rotation)]
                    temp_bin_part_dict["flipped"] = True if self.bin_parts[bin][slot].flipped == "1" else False
                    temp_bin_part_dict["slots"] = temp_slots
                    try:
                        self.bin_parts_dict[bin].append(temp_bin_part_dict)
                    except:
                        self.bin_parts_dict[bin] = []
                        self.bin_parts_dict[bin].append(temp_bin_part_dict)
                    
    # =======================================================
    #               Conveyor Parts Functions
    # =======================================================
    def add_conveyor_parts_widgets_to_frame(self):
        trial_has_parts_cb = ctk.CTkCheckBox(self.conveyor_parts_frame,text="Trial has conveyor parts",variable=self.has_parts, onvalue="1", offvalue="0", height=1, width=20)
        trial_has_parts_cb.pack(pady=5)
        self.conveyor_setup_vals = {"active":ctk.StringVar(),"spawn_rate":ctk.IntVar(),"order":ctk.StringVar()}
        self.conveyor_setup_vals["active"].set('1')
        self.conveyor_setup_vals['spawn_rate'].set(1)
        self.conveyor_setup_vals["order"].set(CONVEYOR_ORDERS[0])
        conveyor_active_cb = ctk.CTkCheckBox(self.conveyor_parts_frame,text="Conveyor active",variable=self.conveyor_setup_vals["active"], onvalue="1", offvalue="0", height=1, width=20, state=tk.DISABLED)
        conveyor_active_cb.pack(pady=5)
        self.present_conveyor_widgets.append(conveyor_active_cb)
        spawn_rate_label = ctk.CTkLabel(self.conveyor_parts_frame,text=f"current spawn rate (seconds): {self.conveyor_setup_vals['spawn_rate'].get()}")
        spawn_rate_label.pack()
        spawn_rate_slider = ctk.CTkSlider(self.conveyor_parts_frame, state=tk.DISABLED,variable=self.conveyor_setup_vals["spawn_rate"],from_=1, to=10, number_of_steps=9, orientation="horizontal")
        spawn_rate_slider.pack()
        self.present_conveyor_widgets.append(spawn_rate_slider)
        conveyor_order_label = ctk.CTkLabel(self.conveyor_parts_frame,text=f"Select the conveyor order:")
        conveyor_order_label.pack()
        conveyor_order_menu = ctk.CTkOptionMenu(self.conveyor_parts_frame, variable=self.conveyor_setup_vals["order"],values=CONVEYOR_ORDERS,state=tk.DISABLED)
        conveyor_order_menu.pack()
        self.present_conveyor_widgets.append(conveyor_order_menu)
        add_parts_button = ctk.CTkButton(self.conveyor_parts_frame,text="Add parts", command=partial(self.add_conveyor_parts), state=tk.DISABLED)
        add_parts_button.pack(pady=10)
        current_parts_label = ctk.CTkLabel(self.conveyor_parts_frame, text="Current parts on conveyor belt:")
        current_parts_label.pack()
        conveyor_canvas = Canvas(self.conveyor_parts_frame)
        conveyor_canvas.pack(fill = BOTH, expand = 1)
        flipped_meaning_label = ctk.CTkLabel(self.conveyor_parts_frame, text="When a part is flipped, an \"F\" will show up in the bottom right of the part image.")
        flipped_meaning_label.pack(pady=10)
        self.present_conveyor_widgets.append(add_parts_button)
        self.conveyor_setup_vals["spawn_rate"].trace('w',partial(self.update_spawn_rate_slider,self.conveyor_setup_vals["spawn_rate"],spawn_rate_label))
        self.has_parts.trace('w', self.activate_deactivate_menu)
        self.conveyor_parts_counter.trace('w',partial(self.show_current_parts,conveyor_canvas))
    
    def show_current_parts(self,canvas : tk.Canvas,_,__,___):
        for e in self.current_conveyor_canvas_elements:
            canvas.delete(e)
        self.current_conveyor_canvas_elements.clear()
        image_coordinates = [(25,10+(75*i)) for i in range(10)]
        num_parts_coordinates = [(65,10+(75*i)) for i in range(10)]
        remove_button_coordinates = [(200,30+(75*i)) for i in range(10)]
        edit_button_coordinates = [(350,30+(75*i)) for i in range(10)]
        flipped_label_coordinates = [(coord[0]+30, coord[1]+30) for coord in image_coordinates]
        image_labels = []
        num_parts_labels = []
        remove_part_buttons = []
        edit_part_buttons = []
        current_flipped_labels = ["" for _ in range(len(flipped_label_coordinates))]
        for i in range(len(self.current_conveyor_parts)):
            part = _part_color_str[self.conveyor_parts[i].part_lot.part.color]+_part_type_str[self.conveyor_parts[i].part_lot.part.type]
            image_labels.append(ctk.CTkLabel(self.conveyor_parts_frame,text="",
            image=ctk.CTkImage(MENU_IMAGES[part].rotate(self.conveyor_parts[i].rotation*180/pi),size=(75,75))))
            num_parts_labels.append(ctk.CTkLabel(self.conveyor_parts_frame,text=f"X {self.conveyor_parts[i].part_lot.quantity}"))
            remove_part_buttons.append(ctk.CTkButton(self.conveyor_parts_frame,
                                                     text="Remove part"+("s" if self.conveyor_parts[i].part_lot.quantity>1 else ""),
                                                     command = partial(self.remove_conveyor_part,i)))
            edit_part_buttons.append(ctk.CTkButton(self.conveyor_parts_frame, 
                                                   text="Edit part"+("s" if self.conveyor_parts[i].part_lot.quantity>1 else ""), 
                                                   command=partial(self.add_conveyor_parts,i)))
            if self.conveyor_parts[i].flipped=="1":
                current_flipped_labels[i]=(ctk.CTkLabel(self.conveyor_parts_frame, text="F"))
        for i in range(len(image_labels)):
            self.current_conveyor_canvas_elements.append(canvas.create_window(image_coordinates[i], window = image_labels[i]))
            self.current_conveyor_canvas_elements.append(canvas.create_window(num_parts_coordinates[i], window = num_parts_labels[i]))
            self.current_conveyor_canvas_elements.append(canvas.create_window(remove_button_coordinates[i], window = remove_part_buttons[i]))
            self.current_conveyor_canvas_elements.append(canvas.create_window(edit_button_coordinates[i], window = edit_part_buttons[i]))
            if current_flipped_labels[i]!="":
                self.current_conveyor_canvas_elements.append(canvas.create_window(flipped_label_coordinates[i], window=current_flipped_labels[i]))

    def remove_conveyor_part(self, index):
        del self.current_conveyor_parts[index]
        self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))

    def update_spawn_rate_slider(self,value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
        label.configure(text=f"current spawn rate (seconds): {value.get()}")

    def activate_deactivate_menu(self,_,__,___):
        if self.has_parts.get()=="1":
            for widget in self.present_conveyor_widgets:
                widget.configure(state=tk.NORMAL)
        else:
            for widget in self.present_conveyor_widgets:
                widget.configure(state=tk.DISABLED)
    
    def add_conveyor_parts(self, index = -1):
        add_parts_conveyor_window = ctk.CTkToplevel()
        add_parts_conveyor_window.geometry("400x450 + 700 + 300")
        conveyor_part_vals = {}
        conveyor_part_vals["color"] = ctk.StringVar()
        conveyor_part_vals["pType"] = ctk.StringVar()
        conveyor_part_vals["num_parts"] = ctk.IntVar()
        conveyor_part_vals["offset"] = ctk.DoubleVar()
        conveyor_part_vals["rotation"] = ctk.DoubleVar()
        conveyor_part_vals["flipped"] = ctk.StringVar()
        if index ==-1:
            conveyor_part_vals["color"].set(PART_COLORS[0])
            conveyor_part_vals["pType"].set(PART_TYPES[0])
            conveyor_part_vals["num_parts"].set(1)
            conveyor_part_vals["offset"].set(0.0)
            conveyor_part_vals["rotation"].set(0.0)
            conveyor_part_vals["flipped"].set("0")
        else:
            conveyor_part_vals["color"].set(_part_color_str[self.conveyor_parts[index].part_lot.part.color])
            conveyor_part_vals["pType"].set(_part_type_str[self.conveyor_parts[index].part_lot.part.type])
            conveyor_part_vals["num_parts"].set(self.conveyor_parts[index].part_lot.quantity)
            conveyor_part_vals["offset"].set(self.conveyor_parts[index].offset)
            conveyor_part_vals["rotation"].set(self.conveyor_parts[index].rotation)
            conveyor_part_vals["flipped"].set(self.conveyor_parts[index].flipped)

        color_menu = ctk.CTkOptionMenu(add_parts_conveyor_window, variable=conveyor_part_vals["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_parts_conveyor_window, variable=conveyor_part_vals["pType"],values=PART_TYPES)
        type_menu.pack()
        num_parts_label = ctk.CTkLabel(add_parts_conveyor_window,text=f"Current number of parts: {conveyor_part_vals['num_parts'].get()}")
        num_parts_label.pack()
        num_parts_slider = ctk.CTkSlider(add_parts_conveyor_window,variable=conveyor_part_vals["num_parts"],from_=1, to=10, number_of_steps=9, orientation="horizontal")
        num_parts_slider.pack()
        conveyor_part_vals["num_parts"].trace('w', partial(self.update_num_parts_slider, conveyor_part_vals["num_parts"], num_parts_label))
        offset_label = ctk.CTkLabel(add_parts_conveyor_window,text=f"Current offset: {conveyor_part_vals['offset'].get()}")
        offset_label.pack()
        offset_slider = ctk.CTkSlider(add_parts_conveyor_window,variable=conveyor_part_vals["offset"],from_=-1, to=1, number_of_steps=40, orientation="horizontal")
        offset_slider.pack()
        conveyor_part_vals["offset"].trace('w', partial(self.update_offset_slider, conveyor_part_vals["offset"], offset_label))
        rotation_label = ctk.CTkLabel(add_parts_conveyor_window, text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(conveyor_part_vals['rotation'].get())]}")
        rotation_label.pack()
        rotation_slider = ctk.CTkSlider(add_parts_conveyor_window, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=conveyor_part_vals["rotation"], orientation="horizontal")
        rotation_slider.pack(pady=5)
        conveyor_part_vals["rotation"].trace('w', partial(self.nearest_slider_value, conveyor_part_vals["rotation"], rotation_slider,rotation_label))
        flipped_cb = ctk.CTkCheckBox(add_parts_conveyor_window,text="Flipped",variable=conveyor_part_vals["flipped"], onvalue="1", offvalue="0", height=1, width=20)
        flipped_cb.pack(pady=5)
        back_button = ctk.CTkButton(add_parts_conveyor_window,text="Back",command=add_parts_conveyor_window.destroy)
        back_button.pack()
        save_button = ctk.CTkButton(add_parts_conveyor_window,text="Save part",command=partial(self.save_conveyor_parts,add_parts_conveyor_window,conveyor_part_vals, index))
        save_button.pack()
    
    def update_num_parts_slider(self,value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
        label.configure(text=f"Current number of parts: {value.get()}")

    def update_offset_slider(self, value : ctk.DoubleVar, label : ctk.CTkLabel,_,__,___):
        value.set(round(value.get(),3))
        label.configure(text=f"Current offset: {value.get()}")
    
    def save_conveyor_parts(self, window:ctk.CTkToplevel, conveyor_part_vals, index):
        color = conveyor_part_vals["color"].get()
        pType = conveyor_part_vals["pType"].get()
        for _ in range(int(conveyor_part_vals["num_parts"].get())):
            self.all_present_parts.append(f"{color} {pType}")
        if index == -1:    
            self.conveyor_parts.append(ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get(), conveyor_part_vals["flipped"].get()))
            self.current_conveyor_parts.append(color+pType)
            self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))
        else:
            self.conveyor_parts[index] = ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get(),conveyor_part_vals["flipped"].get())
            self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))
        window.destroy()
    
    def conveyor_parts_to_dict(self):
        if len(self.conveyor_parts)>0:
            self.conveyor_parts_dict["active"] = True if self.conveyor_setup_vals["active"].get() == "1" else False
            self.conveyor_parts_dict["spawn_rate"] = float(self.conveyor_setup_vals["spawn_rate"].get())
            self.conveyor_parts_dict["order"] = self.conveyor_setup_vals["order"].get()
            self.conveyor_parts_dict["parts_to_spawn"] = []
            for part in self.conveyor_parts:
                part : ConveyorPart
                temp_conveyor_part_dict = {}
                temp_conveyor_part_dict["type"]=_part_type_str[part.part_lot.part.type].lower()
                temp_conveyor_part_dict["color"]=_part_color_str[part.part_lot.part.color].lower()
                temp_conveyor_part_dict["number"] = part.part_lot.quantity
                temp_conveyor_part_dict["offset"] = part.offset
                temp_conveyor_part_dict["flipped"] = True if part.flipped == "1" else False
                temp_conveyor_part_dict["rotation"] = SLIDER_STR[SLIDER_VALUES.index(part.rotation)]
                self.conveyor_parts_dict["parts_to_spawn"].append(temp_conveyor_part_dict)

    # =======================================================
    #                 Order Functions
    # =======================================================
    def add_order_widgets_to_frame(self):
        self.orders_frame.grid_rowconfigure(0, weight=1)
        self.orders_frame.grid_rowconfigure(100, weight=1)
        self.orders_frame.grid_columnconfigure(0, weight=1)
        self.orders_frame.grid_columnconfigure(4, weight=1)

        self.add_kitting_order_button = ctk.CTkButton(self.orders_frame, text="Add kitting order", command=self.add_kitting_order)
        self.add_assembly_order_button = ctk.CTkButton(self.orders_frame, text="Add assembly order", command=self.add_assembly_order)
        self.add_combined_order_button = ctk.CTkButton(self.orders_frame, text="Add combined order", command=self.add_combined_order)

        self.show_main_order_menu()

        self.save_error_message = ctk.CTkLabel(self.orders_frame, text="")
        self.save_order_button = ctk.CTkButton(self.orders_frame,text="Save order", command=self.save_order)
        self.cancel_order_button = ctk.CTkButton(self.orders_frame,text="Cancel order", command=self.show_main_order_menu)

        # Trace functions
        self.order_info["announcement_type"].trace('w', self.show_correct_announcement_menu)
    
    def show_main_order_menu(self):
        self.clear_order_menu()

        self.add_kitting_order_button.grid(column=MIDDLE_COLUMN, row=1, pady=10)
        self.add_assembly_order_button.grid(column=MIDDLE_COLUMN, row=2, pady=10)
        self.add_combined_order_button.grid(column=MIDDLE_COLUMN, row=3, pady=10)

        self.current_main_order_widgets.append(self.add_kitting_order_button)
        self.current_main_order_widgets.append(self.add_assembly_order_button)
        self.current_main_order_widgets.append(self.add_combined_order_button)

        current_row = 4
        for i in range(len(self.current_orders)):
            temp_order_label = ctk.CTkLabel(self.orders_frame,
                                            text=f"Order {i}. ID: {self.current_orders[i].id}\nType {ORDER_TYPES[self.current_orders[i].type]}")
            temp_order_label.grid(column = LEFT_COLUMN, row = current_row+i)
            self.current_left_order_widgets.append(temp_order_label)

            edit_order_button = ctk.CTkButton(self.orders_frame, text="Edit order", command=partial(self.edit_order, i))
            edit_order_button.grid(column=MIDDLE_COLUMN, row=current_row+i, padx=10)
            self.current_main_order_widgets.append(edit_order_button)

            delete_order_button = ctk.CTkButton(self.orders_frame, text = "Delete order", command=partial(self.delete_order, i))
            delete_order_button.grid(column = RIGHT_COLUMN, row = current_row+i)
            self.current_right_order_widgets.append(delete_order_button)
    
    def edit_order(self, index : int):
        self.set_order_variables_to_current_order(self.current_orders[index])
        self.left_row_index = 1
        self.clear_order_menu()
        if self.order_info["order_type"].get() == "kitting":
            self.show_kitting_menu()
        elif self.order_info["order_type"].get() == "assembly":
            self.show_assembly_menu()
        else:
            self.show_combined_menu()
        self.add_order_main_widgets(index)
    
    def set_order_variables_to_current_order(self, order : OrderMsg):
        self.order_info["order_type"].set(ORDER_TYPES[order.type])
        self.order_info["priority"].set(str(order.priority))
        self.order_info["announcement_type"].set(CONDITION_TYPE[order.condition.type])
        if order.condition.type == 0:
            self.order_info["announcement"]["time_condition"].set(str(order.condition.time_condition.seconds))
        elif order.condition.type == 1:
            self.order_info["announcement"]["color"].set(_part_color_str[order.condition.part_place_condition.part.color])
            self.order_info["announcement"]["type"].set(_part_type_str[order.condition.part_place_condition.part.type])
            self.order_info["announcement"]["agv"].set(str(order.condition.part_place_condition.agv))
        else:
            self.order_info["announcement"]["submission_id"].set(str(order.condition.submission_condition.order_id))
        
        if order.type == 0:
            self.order_info["kitting_task"]["agv_number"].set(str(order.kitting_task.agv_number))
            self.order_info["kitting_task"]["tray_id"].set(str(order.kitting_task.tray_id))
            self.order_info["kitting_task"]["parts"] = order.kitting_task.parts

        elif order.type == 1:
            for i in range(len(self.order_info["assembly_task"]["agv_numbers"])):
                self.order_info["assembly_task"]["agv_numbers"][i].set("1" if i+1 in order.assembly_task.agv_numbers else "0")
            self.order_info["assembly_task"]["station"].set(order.assembly_task.station)
            self.order_info["assembly_task"]["parts"] = order.assembly_task.parts
        else:
            self.order_info["combined_task"]["station"].set(order.combined_task.station)
            self.order_info["combined_task"]["parts"] = order.combined_task.parts

    def delete_order(self, index):
        del self.current_orders[index]
        del self.used_ids[index]
        self.order_counter.set(str(len(self.current_orders)))
        self.show_main_order_menu()

    def add_kitting_order(self):
        self.clear_order_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("kitting")
        self.show_kitting_menu()
        self.add_order_main_widgets()
    
    def add_assembly_order(self):
        self.clear_order_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("assembly")
        self.show_assembly_menu()
        self.add_order_main_widgets()

    def add_combined_order(self):
        self.clear_order_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("combined")
        self.show_combined_menu()
        self.add_order_main_widgets()

    def add_order_main_widgets(self, index = -1):
        self.priority_cb = ctk.CTkCheckBox(self.orders_frame,text="Priority",variable=self.order_info["priority"], onvalue="1", offvalue="0", height=1, width=20)
        self.priority_cb.grid(column=MIDDLE_COLUMN, row=1)
        self.current_main_order_widgets.append(self.priority_cb)
        self.save_error_message.grid(column=LEFT_COLUMN, columnspan = 3, row=max(self.left_row_index,self.right_row_index)+1)
        self.current_main_order_widgets.append(self.save_error_message)
        self.save_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+2)
        self.current_main_order_widgets.append(self.save_order_button)
        self.cancel_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+3)
        self.current_main_order_widgets.append(self.cancel_order_button)

        self.announcement_type_label = ctk.CTkLabel(self.orders_frame, text = "Select the type of announcement:")
        self.announcement_type_label.grid(column = RIGHT_COLUMN , row = 1)
        self.current_main_order_widgets.append(self.announcement_type_label)
        self.announcement_type_menu = ctk.CTkOptionMenu(self.orders_frame, variable=self.order_info["announcement_type"],values=CONDITION_TYPE)
        self.announcement_type_menu.grid(column = RIGHT_COLUMN, row = 2)
        self.current_main_order_widgets.append(self.announcement_type_menu)

        self.save_order_button.configure(command = partial(self.save_order, index))

        self.order_info["announcement_type"].set(CONDITION_TYPE[0])
    
    def grid_left_column(self, widget, pady=7):
        widget.grid(column = LEFT_COLUMN, row = self.left_row_index, pady=pady)
        self.left_row_index+=1
        
    def grid_right_column(self, widget, pady=7):
        widget.grid(column = RIGHT_COLUMN, row = self.right_row_index, pady=pady)
        self.right_row_index+=1

    def generate_order_id(self):
        '''Generates a unique id for each order'''
        newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
        if newId in self.used_ids:
            while newId in self.used_ids:
                newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
        return newId    

    def reset_order(self):
        self.order_info["order_type"].set(ORDER_TYPES[0])
        self.order_info["priority"].set('0')
        self.order_info["announcement_type"].set(CONDITION_TYPE[0])
        self.order_info["announcement"]["time_condition"].set('0.0')
        self.order_info["announcement"]["color"].set(PART_COLORS[0])
        self.order_info["announcement"]["type"].set(PART_TYPES[0])
        self.order_info["announcement"]["agv"].set(AGV_OPTIONS[0])
        self.order_info["announcement"]["submission_id"].set('' if len(self.used_ids)==0 else self.used_ids[0])
        self.order_info["priority"].set('0')

        self.order_info["kitting_task"]["agv_number"].set(AGV_OPTIONS[0])
        self.order_info["kitting_task"]["tray_id"].set(TRAY_IDS[0])
        self.order_info["kitting_task"]["parts"] = []

        for i in range(len(self.order_info["assembly_task"]["agv_numbers"])):
            self.order_info["assembly_task"]["agv_numbers"][i].set("0")
        self.order_info["assembly_task"]["station"].set(ASSEMBLY_STATIONS[0])
        self.order_info["assembly_task"]["parts"] = []

        self.order_info["combined_task"]["station"].set(ASSEMBLY_STATIONS[0])
        self.order_info["combined_task"]["parts"] = []
    
    def show_correct_announcement_menu(self,_,__,___):
        self.right_row_index = 3
        if self.order_info["announcement_type"].get()=="time":
            self.show_time_announcement_menu()
        elif self.order_info["announcement_type"].get()=="part_place":
            self.show_part_place_announcement_menu()
        else:
            self.show_submission_announcement_menu()
        self.move_order_widgets()
        
    def show_time_announcement_menu(self):
        for widget in self.current_right_order_widgets:
            widget.grid_forget()
        self.current_right_order_widgets.clear()

        time_label = ctk.CTkLabel(self.orders_frame,text="Enter the time for the announcement:")
        self.grid_right_column(time_label)
        self.current_right_order_widgets.append(time_label)

        time_entry = ctk.CTkEntry(self.orders_frame, textvariable=self.order_info["announcement"]["time_condition"])
        self.grid_right_column(time_entry)
        self.current_right_order_widgets.append(time_entry)
    
    def show_part_place_announcement_menu(self):
        for widget in self.current_right_order_widgets:
            widget.grid_forget()

        self.current_right_order_widgets.clear()
        color_menu = ctk.CTkOptionMenu(self.orders_frame, variable=self.order_info["announcement"]["color"],values=PART_COLORS)
        self.grid_right_column(color_menu)
        self.current_right_order_widgets.append(color_menu)
        type_menu = ctk.CTkOptionMenu(self.orders_frame, variable=self.order_info["announcement"]["type"],values=PART_TYPES)
        self.grid_right_column(type_menu)
        self.current_right_order_widgets.append(type_menu)
        quadrant_menu = ctk.CTkOptionMenu(self.orders_frame, variable=self.order_info["announcement"]["agv"], values=AGV_OPTIONS)
        self.grid_right_column(quadrant_menu)
        self.current_right_order_widgets.append(quadrant_menu)
    
    def show_submission_announcement_menu(self):
        for widget in self.current_right_order_widgets:
            widget.grid_forget()
            
        self.current_right_order_widgets.clear()
        id_menu = ctk.CTkOptionMenu(self.orders_frame, variable=self.order_info["announcement"]["submission_id"],values=self.used_ids)
        self.grid_right_column(id_menu)
        self.current_right_order_widgets.append(id_menu)
    
    def move_order_widgets(self):
        for widget in self.current_order_part_widgets:
            widget.grid_forget()
        self.current_order_part_widgets.clear()
        current_row = max(self.left_row_index,self.right_row_index)+1
        index = 0
        if self.order_info["order_type"].get() == "kitting":
            if len(self.order_info["kitting_task"]["parts"])>0:
                current_parts_label = ctk.CTkLabel(self.orders_frame, text="Current kitting parts:")
                current_parts_label.grid(row = current_row, column = MIDDLE_COLUMN)
                self.current_order_part_widgets.append(current_parts_label)
                current_row+=1
                for part in self.order_info["kitting_task"]["parts"]:
                    part:KittingPartMsg
                    part_label = ctk.CTkLabel(self.orders_frame, text=f"{_part_color_str[part.part.color]} {_part_type_str[part.part.type]}\n Quadrant: {part.quadrant}")
                    part_label.grid(row = current_row, column = LEFT_COLUMN)
                    self.current_order_part_widgets.append(part_label)
                    edit_part_button = ctk.CTkButton(self.orders_frame, text="edit part", command=partial(self.add_kitting_part,part, index))
                    edit_part_button.grid(row = current_row, column = MIDDLE_COLUMN, pady = 3)
                    self.current_order_part_widgets.append(edit_part_button)
                    delete_part_button = ctk.CTkButton(self.orders_frame, text="Delete part", command=partial(self.remove_order_part,"kitting_task", index))
                    delete_part_button.grid(row = current_row, column = RIGHT_COLUMN)
                    self.current_order_part_widgets.append(delete_part_button)
                    index+=1
                    current_row+=1
        elif self.order_info["order_type"].get()=="assembly":
            if len(self.order_info["assembly_task"]["parts"])>0:
                current_parts_label = ctk.CTkLabel(self.orders_frame, text="Current assembly parts:")
                current_parts_label.grid(row = current_row, column = MIDDLE_COLUMN)
                self.current_order_part_widgets.append(current_parts_label)
                current_row+=1
                for part in self.order_info["assembly_task"]["parts"]:
                    part:AssemblyPartMsg
                    part_label = ctk.CTkLabel(self.orders_frame, text=f"{_part_color_str[part.part.color]} {_part_type_str[part.part.type]}")
                    part_label.grid(row = current_row, column = LEFT_COLUMN)
                    self.current_order_part_widgets.append(part_label)
                    edit_part_button = ctk.CTkButton(self.orders_frame, text="Edit part", command=partial(self.add_assembly_part,part, index))
                    edit_part_button.grid(row = current_row, column = MIDDLE_COLUMN, pady = 3)
                    self.current_order_part_widgets.append(edit_part_button)
                    delete_part_button = ctk.CTkButton(self.orders_frame, text="Delete part", command=partial(self.remove_order_part,"assembly_task", index))
                    delete_part_button.grid(row = current_row, column = RIGHT_COLUMN)
                    self.current_order_part_widgets.append(delete_part_button)
                    index+=1
                    current_row+=1
        else:
            if len(self.order_info["combined_task"]["parts"])>0:
                current_parts_label = ctk.CTkLabel(self.orders_frame, text="Current combined parts:")
                current_parts_label.grid(row = current_row, column = MIDDLE_COLUMN)
                self.current_order_part_widgets.append(current_parts_label)
                current_row+=1
                for part in self.order_info["combined_task"]["parts"]:
                    part:AssemblyPartMsg
                    part_label = ctk.CTkLabel(self.orders_frame, text=f"{_part_color_str[part.part.color]} {_part_type_str[part.part.type]}")
                    part_label.grid(row = current_row, column = LEFT_COLUMN)
                    self.current_order_part_widgets.append(part_label)
                    edit_part_button = ctk.CTkButton(self.orders_frame, text="Edit part", command=partial(self.add_combined_part,part, index))
                    edit_part_button.grid(row = current_row, column = MIDDLE_COLUMN, pady = 3)
                    self.current_order_part_widgets.append(edit_part_button)
                    part_label = ctk.CTkButton(self.orders_frame, text="Delete part", command=partial(self.remove_order_part,"combined_task", index))
                    part_label.grid(row = current_row, column = RIGHT_COLUMN)
                    self.current_order_part_widgets.append(part_label)
                    index+=1
                    current_row+=1
        
        self.save_error_message.grid_forget()
        self.save_error_message.grid(column = LEFT_COLUMN, columnspan=3, row=current_row, pady=10)
        current_row+=1
        self.save_order_button.grid_forget()
        self.save_order_button.grid(column = MIDDLE_COLUMN, row=current_row, pady=5)
        current_row+=1
        self.cancel_order_button.grid_forget()
        self.cancel_order_button.grid(column = MIDDLE_COLUMN, row=current_row, pady=5)
    
    def remove_order_part(self, task, index):
        if task == "kitting_task":
            self.add_part_kitting_task_button.grid_forget()
            self.left_row_index-=1
            self.grid_left_column(self.add_part_kitting_task_button)
        del self.order_info[task]["parts"][index]
        if len(self.order_info[task]["parts"])<1 and task!="assembly_task":
            self.save_order_button.configure(state=DISABLED)
            self.save_error_message.configure(text="To save, you need at least one part")
        if task=="assembly_task":
            self.activate_assembly_save(1,1,1)
        self.move_order_widgets()

    def get_remaining_quadrants(self, current_selection = -1):
        quadrant_options = [1,2,3,4]
        if current_selection!=-1:
            quadrant_options.append(current_selection)
        for part in self.order_info["kitting_task"]["parts"]:
            part : KittingPartMsg
            quadrant_options.remove(part.quadrant)
        return sorted(quadrant_options)

    def show_kitting_menu(self):
        self.order_info["order_type"].set("kitting")
        for widget in self.current_left_order_widgets:
            widget.grid_forget()
        self.current_left_order_widgets.clear()
        
        agv_number_label = ctk.CTkLabel(self.orders_frame,text="Select the agv for the kitting order")
        self.grid_left_column(agv_number_label)
        self.current_left_order_widgets.append(agv_number_label)

        agv_number_menu = ctk.CTkOptionMenu(self.orders_frame,variable=self.order_info["kitting_task"]["agv_number"], values = AGV_OPTIONS)
        self.grid_left_column(agv_number_menu)
        self.current_left_order_widgets.append(agv_number_menu)

        tray_id_label = ctk.CTkLabel(self.orders_frame,text="Select the tray for the kitting order")
        self.grid_left_column(tray_id_label)
        self.current_left_order_widgets.append(tray_id_label)

        tray_id_menu = ctk.CTkOptionMenu(self.orders_frame,variable=self.order_info["kitting_task"]["tray_id"], values = TRAY_IDS)
        self.grid_left_column(tray_id_menu)
        self.current_left_order_widgets.append(tray_id_menu)

        self.add_part_kitting_task_button = ctk.CTkButton(self.orders_frame, text="Add part", command=self.add_kitting_part)
        self.grid_left_column(self.add_part_kitting_task_button)
        self.current_left_order_widgets.append(self.add_part_kitting_task_button)
        self.cancel_order_button.configure(text="Cancel kitting order")
        if len(self.order_info["kitting_task"]["parts"]) == 0:
            self.save_error_message.configure(text="To save, you need at least one part")
            self.save_order_button.configure(text="Save kitting order", state=DISABLED)
        else:
            self.save_order_button.configure(state=NORMAL)
    
    def add_kitting_part(self, kitting_part = None, index = -1):
        add_k_part_wind = ctk.CTkToplevel()
        quadrant_options = self.get_remaining_quadrants()
        k_part_dict = {}
        k_part_dict["color"] = ctk.StringVar()
        k_part_dict["pType"] = ctk.StringVar()
        k_part_dict["quadrant"] = ctk.StringVar()
        quadrant_options = [-1]
        if kitting_part!=None:
            k_part_dict["color"].set(_part_color_str[kitting_part.part.color].lower())
            k_part_dict["pType"].set(_part_type_str[kitting_part.part.type].lower())
            k_part_dict["quadrant"].set(str(kitting_part.quadrant))
            quadrant_options = [str(val) for val in self.get_remaining_quadrants(kitting_part.quadrant)]
        else:
            quadrant_options = [str(val) for val in self.get_remaining_quadrants()]
            k_part_dict["color"].set(PART_COLORS[0])
            k_part_dict["pType"].set(PART_TYPES[0])
            k_part_dict["quadrant"].set(quadrant_options[0])

        color_label = ctk.CTkLabel(add_k_part_wind, text="Select the color for the kitting part")
        color_label.pack()
        color_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_label = ctk.CTkLabel(add_k_part_wind, text="Select the type for the kitting part")
        type_label.pack()
        type_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()
        quadrant_label = ctk.CTkLabel(add_k_part_wind, text="Select the quadrant for the kitting part")
        quadrant_label.pack()
        quadrant_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["quadrant"], values=quadrant_options)
        quadrant_menu.pack()

        save_button = ctk.CTkButton(add_k_part_wind, text="Save kitting part", command=partial(self.save_kitting_part, k_part_dict, add_k_part_wind, index))
        save_button.pack(pady = 10)
        add_k_part_wind.mainloop()

    def save_kitting_part(self, k_part_dict, window, index):
        new_kitting_part = KittingPartMsg()
        new_kitting_part.part.color = _part_color_ints[k_part_dict["color"].get().upper()]
        new_kitting_part.part.type = _part_type_ints[k_part_dict["pType"].get().upper()]
        new_kitting_part.quadrant = int(k_part_dict["quadrant"].get())
        if index == -1:
            self.order_info["kitting_task"]["parts"].append(new_kitting_part)
        else:
            self.order_info["kitting_task"]["parts"][index]=new_kitting_part
        if len(self.order_info["kitting_task"]["parts"])>=4:
            self.add_part_kitting_task_button.grid_forget()
        self.move_order_widgets()
        self.save_error_message.configure(text="")
        self.save_order_button.configure(state=NORMAL)
        window.destroy()
    
    def show_assembly_menu(self):
        self.order_info["order_type"].set("assembly")
        for widget in self.current_left_order_widgets:
            widget.grid_forget()
        self.current_left_order_widgets.clear()

        agv_number_label = ctk.CTkLabel(self.orders_frame,text="Select the agvs for the assembly order")
        self.grid_left_column(agv_number_label)
        self.current_left_order_widgets.append(agv_number_label)

        for i in range(len(self.order_info["assembly_task"]["agv_numbers"])):
            check_box = ctk.CTkCheckBox(self.orders_frame,
                                        text=f"AGV {i+1}",
                                        variable=self.order_info["assembly_task"]["agv_numbers"][i],
                                        offvalue="0",
                                        onvalue="1",
                                        height=1, 
                                        width=20)
            self.grid_left_column(check_box)
            self.current_left_order_widgets.append(check_box)

        station_label = ctk.CTkLabel(self.orders_frame,text="Select the assembly station for the assembly order")
        self.grid_left_column(station_label)
        self.current_left_order_widgets.append(station_label)

        station_menu = ctk.CTkOptionMenu(self.orders_frame,variable=self.order_info["assembly_task"]["station"], values = ASSEMBLY_STATIONS)
        self.grid_left_column(station_menu)
        self.current_left_order_widgets.append(station_menu)

        add_part_assembly_task = ctk.CTkButton(self.orders_frame, text="Add part", command=self.add_assembly_part)
        self.grid_left_column(add_part_assembly_task)
        self.current_left_order_widgets.append(add_part_assembly_task)

        self.cancel_order_button.configure(text="Cancel assembly order")
        self.activate_assembly_save(1,1,1)
    
    def activate_assembly_save(self,_,__,___):
        if len([1 for i in range(4) if self.order_info["assembly_task"]["agv_numbers"][i].get()=="0"]) == 4 and len(self.order_info["assembly_task"]["parts"])<1:
            self.save_error_message.configure(text="To save, you need at least one part and one agv selected")
            self.save_order_button.configure(state=DISABLED)
        elif len(self.order_info["assembly_task"]["parts"])<1:
            self.save_error_message.configure(text="To save, you need at least one part")
            self.save_order_button.configure(state=DISABLED)
        elif len([1 for i in range(4) if self.order_info["assembly_task"]["agv_numbers"][i].get()=="0"]) == 4:
            self.save_error_message.configure(text="To save, you need at least one agv selected")
            self.save_order_button.configure(state=DISABLED)
        else:
            self.save_error_message.configure(text="")
            self.save_order_button.configure(state=NORMAL)
    
    def add_assembly_part(self, assembly_part = None, index = -1):
        add_a_part_wind = ctk.CTkToplevel()

        a_part_dict = {}
        a_part_dict["color"] = ctk.StringVar()
        a_part_dict["pType"] = ctk.StringVar()
        if assembly_part==None:
            a_part_dict["color"].set(PART_COLORS[0])
            a_part_dict["pType"].set(PART_TYPES[0])
        else:
            a_part_dict["color"].set(_part_color_str[assembly_part.part.color].lower())
            a_part_dict["pType"].set(_part_type_str[assembly_part.part.type].lower())

        color_label = ctk.CTkLabel(add_a_part_wind, text="Select the color for the assembly part")
        color_label.pack()
        color_menu = ctk.CTkOptionMenu(add_a_part_wind, variable=a_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_label = ctk.CTkLabel(add_a_part_wind, text="Select the type for the assembly part")
        type_label.pack()
        type_menu = ctk.CTkOptionMenu(add_a_part_wind, variable=a_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()

        save_button = ctk.CTkButton(add_a_part_wind, text="Save assembly part", command=partial(self.save_assembly_part, a_part_dict, add_a_part_wind, index))
        save_button.pack(pady = 10)
        add_a_part_wind.mainloop()

    def save_assembly_part(self, a_part_dict, window, index):
        new_assembly_part = AssemblyPartMsg()

        new_assembly_part.part.color = _part_color_ints[a_part_dict["color"].get().upper()]
        new_assembly_part.part.type = _part_type_ints[a_part_dict["pType"].get().upper()]
        new_assembly_part.assembled_pose = _assembly_part_poses[a_part_dict["pType"].get().upper()]
        new_assembly_part.install_direction = _assembly_part_install_directions[a_part_dict["pType"].get().upper()]
        if index == -1:
            self.order_info["assembly_task"]["parts"].append(new_assembly_part)
        else:
            self.order_info["assembly_task"]["parts"][index] = new_assembly_part
        self.move_order_widgets()
        self.activate_assembly_save(1,1,1)
        window.destroy()
    
    def show_combined_menu(self):
        self.order_info["order_type"].set("combined")
        for widget in self.current_left_order_widgets:
            widget.grid_forget()
        self.current_left_order_widgets.clear()
        
        station_label = ctk.CTkLabel(self.orders_frame,text="Select the assembly station for the combined order")
        self.grid_left_column(station_label)
        self.current_left_order_widgets.append(station_label)

        station_menu = ctk.CTkOptionMenu(self.orders_frame,variable=self.order_info["combined_task"]["station"], values = ASSEMBLY_STATIONS)
        self.grid_left_column(station_menu)
        self.current_left_order_widgets.append(station_menu)

        add_part_combined_task = ctk.CTkButton(self.orders_frame, text="Add part", command=self.add_combined_part)
        self.grid_left_column(add_part_combined_task)
        self.current_left_order_widgets.append(add_part_combined_task)

        self.cancel_order_button.configure(text="Cancel combined order")
        if len(self.order_info["assembly_task"]["parts"]) == 0:
            self.save_error_message.configure(text="To save, you need at least one part")
            self.save_order_button.configure(text="Save kitting order", state=DISABLED)
        else:
            self.save_order_button.configure(state=NORMAL)
    
    def add_combined_part(self, combined_part = None, index = -1):
        add_c_part_wind = ctk.CTkToplevel()

        c_part_dict = {}
        c_part_dict["color"] = ctk.StringVar()
        c_part_dict["pType"] = ctk.StringVar()

        if combined_part==None:
            c_part_dict["color"].set(PART_COLORS[0])
            c_part_dict["pType"].set(PART_TYPES[0])
        else:
            c_part_dict["color"].set(_part_color_str[combined_part.part.color].lower())
            c_part_dict["pType"].set(_part_type_str[combined_part.part.type].lower())

        color_label = ctk.CTkLabel(add_c_part_wind, text="Select the color for the assembly part")
        color_label.pack()
        color_menu = ctk.CTkOptionMenu(add_c_part_wind, variable=c_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_label = ctk.CTkLabel(add_c_part_wind, text="Select the type for the combined part")
        type_label.pack()
        type_menu = ctk.CTkOptionMenu(add_c_part_wind, variable=c_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()

        save_button = ctk.CTkButton(add_c_part_wind, text="Save combined part", command=partial(self.save_combined_part, c_part_dict, add_c_part_wind, index))
        save_button.pack(pady = 10)
        add_c_part_wind.mainloop()

    def save_combined_part(self, c_part_dict, window, index):
        new_combined_part = AssemblyPartMsg()
        new_part = PartMsg()
        new_part.color = _part_color_ints[c_part_dict["color"].get().upper()]
        new_part.type = _part_type_ints[c_part_dict["pType"].get().upper()]
        
        new_combined_part.part = new_part
        new_combined_part.assembled_pose = _assembly_part_poses[c_part_dict["pType"].get().upper()]
        new_combined_part.install_direction = _assembly_part_install_directions[c_part_dict["pType"].get().upper()]
        if index == -1:
            self.order_info["combined_task"]["parts"].append(new_combined_part)
        else:
            self.order_info["combined_task"]["parts"][index] = new_combined_part
        self.move_order_widgets()
        self.save_order_button.configure(state=NORMAL)
        window.destroy()

    def clear_order_menu(self):
        for widget_list in [self.current_left_order_widgets,self.current_right_order_widgets, self.current_main_order_widgets, self.current_order_part_widgets]:
            for widget in widget_list:
                widget.grid_forget()
            widget_list.clear()

    def save_order(self, index):
        new_order = OrderMsg()
        if index==-1:
            self.used_ids.append(self.generate_order_id())
            new_order.id = self.used_ids[-1]
        else:
            new_order.id = self.current_orders[index].id
        new_order.type = ORDER_TYPES.index(self.order_info["order_type"].get())
        new_order.priority = True if self.order_info["priority"].get() == "1" else False
        if self.order_info["order_type"].get() == "kitting":
            new_order.kitting_task = self.create_kitting_task_msg()
        elif self.order_info["order_type"].get() == "assembly":
            new_order.assembly_task = self.create_assembly_task_msg()
        else:
            new_order.combined_task = self.create_combined_task_msg()
        new_order.condition.type = CONDITION_TYPE.index(self.order_info["announcement_type"].get())
        if self.order_info["announcement_type"].get() == "time":
            new_order.condition.time_condition.seconds = float(self.order_info["announcement"]["time_condition"].get())
        elif self.order_info["announcement_type"].get() == "part_place":
            new_order.condition.part_place_condition.part.color = _part_color_ints[self.order_info["announcement"]["color"].get().upper()]
            new_order.condition.part_place_condition.part.type = _part_type_ints[self.order_info["announcement"]["type"].get().upper()]
            new_order.condition.part_place_condition.agv = int(self.order_info["announcement"]["agv"].get())
        else:
            new_order.condition.submission_condition.order_id = self.order_info["announcement"]["submission_id"].get()
        if index == -1:
            self.current_orders.append(new_order)
        else:
            self.current_orders[index] = new_order
        self.order_counter.set(str(len(self.used_ids)))
        self.reset_order()
        self.show_main_order_menu()
        if 'submission' not in CONDITION_TYPE:
            CONDITION_TYPE.append('submission')

    def create_kitting_task_msg(self)->KittingTaskMsg:
        new_kitting_task = KittingTaskMsg()
        new_kitting_task.agv_number = int(self.order_info["kitting_task"]["agv_number"].get())
        new_kitting_task.tray_id = int(self.order_info["kitting_task"]["tray_id"].get())
        new_kitting_task.destination = 3
        new_kitting_task.parts = self.order_info["kitting_task"]["parts"]
        return new_kitting_task

    def create_assembly_task_msg(self)->AssemblyTaskMsg:
        new_assembly_task = AssemblyTaskMsg()
        agv_numbers_list = [i+1 for i in range(len(self.order_info["assembly_task"]["agv_numbers"])) if self.order_info["assembly_task"]["agv_numbers"][i].get()=="1"]
        new_assembly_task.agv_numbers = agv_numbers_list
        new_assembly_task.station = ASSEMBLY_STATIONS.index(self.order_info["assembly_task"]["station"].get())
        new_assembly_task.parts = self.order_info["assembly_task"]["parts"]
        return new_assembly_task

    def create_combined_task_msg(self)->CombinedTaskMsg:
        new_combined_task = CombinedTaskMsg()
        new_combined_task.station = ASSEMBLY_STATIONS.index(self.order_info["combined_task"]["station"].get())
        new_combined_task.parts = self.order_info["combined_task"]["parts"]
        return new_combined_task

    def orders_to_dict(self):
        if len(self.current_orders)>0:
            self.orders_dict["orders"] = []
            for order in self.current_orders:
                temp_order_dict = {}
                order : OrderMsg
                temp_order_dict["id"] = order.id
                temp_order_dict["type"] = ORDER_TYPES[order.type]
                temp_order_dict["announcement"] = self.announcement_to_dict(order.condition)
                temp_order_dict["priority"] = order.priority
                if order.type == 0:
                    temp_order_dict["kitting_task"] = {}
                    temp_order_dict["kitting_task"]["agv_number"] = order.kitting_task.agv_number
                    temp_order_dict["kitting_task"]["tray_id"] = order.kitting_task.tray_id
                    temp_order_dict["kitting_task"]["destination"] = "warehouse"
                    temp_order_dict["kitting_task"]["products"] = []
                    for part in order.kitting_task.parts:
                        part : KittingPartMsg
                        temp_kitting_part_dict = {}
                        temp_kitting_part_dict["type"] = _part_type_str[part.part.type].lower()
                        temp_kitting_part_dict["color"] = _part_color_str[part.part.color].lower()
                        temp_kitting_part_dict["quadrant"] = part.quadrant
                        temp_order_dict["kitting_task"]["products"].append(temp_kitting_part_dict)
                elif order.type == 1:
                    temp_order_dict["assembly_task"] = {}
                    temp_order_dict["assembly_task"]["agv_number"] = [order.assembly_task.agv_numbers[i] for i in range(len(order.assembly_task.agv_numbers))]
                    temp_order_dict["assembly_task"]["station"] = order.assembly_task.station
                    temp_order_dict["assembly_task"]["products"] = []
                    for part in order.assembly_task.parts:
                        part : AssemblyPartMsg
                        temp_assembly_part_dict = {}
                        temp_assembly_part_dict["type"] = _part_type_str[part.part.type].lower()
                        temp_assembly_part_dict["color"] = _part_color_str[part.part.color].lower()
                        for key in _assembly_part_pose_and_direction_dicts[_part_type_str[part.part.type].upper()].keys():
                            temp_assembly_part_dict[key] = copy(_assembly_part_pose_and_direction_dicts[_part_type_str[part.part.type].upper()][key])
                        temp_order_dict["assembly_task"]["products"].append(temp_assembly_part_dict)
                else:
                    temp_order_dict["combined_task"] = {}
                    temp_order_dict["combined_task"]["station"] = order.combined_task.station
                    temp_order_dict["combined_task"]["products"] = []
                    for part in order.combined_task.parts:
                        part : AssemblyPartMsg
                        temp_combined_part_dict = {}
                        temp_combined_part_dict["type"] = _part_type_str[part.part.type].lower()
                        temp_combined_part_dict["color"] = _part_color_str[part.part.color].lower()
                        for key in _assembly_part_pose_and_direction_dicts[_part_type_str[part.part.type].upper()].keys():
                            temp_combined_part_dict[key] = copy(_assembly_part_pose_and_direction_dicts[_part_type_str[part.part.type].upper()][key])
                        temp_order_dict["combined_task"]["products"].append(temp_combined_part_dict)
                self.orders_dict["orders"].append(temp_order_dict)
    
    # =======================================================
    #                  Challenges functions
    # =======================================================
    def add_challenges_widgets_to_frame(self):
        self.reset_all_challenges()
        self.challenges_frame.grid_rowconfigure(0, weight=1)
        self.challenges_frame.grid_rowconfigure(100, weight=1)
        self.challenges_frame.grid_columnconfigure(0, weight=1)
        self.challenges_frame.grid_columnconfigure(4, weight=1)
        self.add_dropped_part_button = ctk.CTkButton(self.challenges_frame, text="Add dropped part challenge", command=self.add_dropped_part_challenge)
        self.add_robot_malfunction_button = ctk.CTkButton(self.challenges_frame, text="Add robot malfunction challenge", command=self.add_robot_malfunction_challenge)
        self.add_sensor_blackout_button = ctk.CTkButton(self.challenges_frame, text="Add sensor blackout challenge", command=self.add_sensor_blackout_challenge)
        self.add_faulty_part_button = ctk.CTkButton(self.challenges_frame, text = "Add faulty part challenge", command=self.add_faulty_part_challenge)
        self.add_human_button = ctk.CTkButton(self.challenges_frame, text="Add human challenge", command = self.add_human_challenge)

        self.show_main_challenges_menu(1,1,1)

        self.save_challenge_button = ctk.CTkButton(self.challenges_frame,text="Save challenge", command=self.save_challenge)
        self.cancel_challenge_button = ctk.CTkButton(self.challenges_frame,text="Cancel challenge", command=partial(self.show_main_challenges_menu,1,1,1))
        self.order_counter.trace('w', self.show_main_challenges_menu)
        self.challenges_counter.trace('w', self.show_main_challenges_menu)
        self.challenge_condition_type.trace('w', self.show_correct_condition_menu)

    def reset_challenge_condition_variables(self, type_aswell = True):
        if type_aswell:
            self.challenge_condition_type.set(CONDITION_TYPE[0])
        self.challenge_condition_info["time_condition"].set("0.0")
        self.challenge_condition_info["color"].set(PART_COLORS[0])
        self.challenge_condition_info["type"].set(PART_TYPES[0])
        self.challenge_condition_info["agv"].set(AGV_OPTIONS[0])
        self.challenge_condition_info["submission_id"].set("" if len(self.used_ids) == 0 else self.used_ids[0])

    def show_challenges_condition_menu(self):
        challenge_condition_type_label = ctk.CTkLabel(self.challenges_frame, text="Select the type of condition for the challenge")
        self.grid_and_append_challenge_widget(challenge_condition_type_label)
        challenge_condition_type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_type,values=CONDITION_TYPE)
        self.grid_and_append_challenge_widget(challenge_condition_type_menu)
        self.reset_challenge_condition_variables()
    
    def show_correct_condition_menu(self,_,__,____):
        for widget in self.current_challenges_condition_widgets:
            widget.grid_forget()
        self.reset_challenge_condition_variables(False)
        self.current_challenges_condition_widgets.clear()
        if self.challenge_condition_type.get() == "time":
            time_label = ctk.CTkLabel(self.challenges_frame, text="Enter the time for the time_condition:")
            time_label.grid(column = MIDDLE_COLUMN, row = 25)
            self.current_challenges_condition_widgets.append(time_label)
            time_entry = ctk.CTkEntry(self.challenges_frame, textvariable=self.challenge_condition_info["time_condition"])
            time_entry.grid(column = MIDDLE_COLUMN, row = 26)
            self.current_challenges_condition_widgets.append(time_entry)
        elif self.challenge_condition_type.get() == "part_place":
            color_label = ctk.CTkLabel(self.challenges_frame, text="Select the color for the part condition")
            color_label.grid(column = MIDDLE_COLUMN, row = 25)
            self.current_challenges_condition_widgets.append(color_label)
            color_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["color"], values=PART_COLORS)
            color_menu.grid(column = MIDDLE_COLUMN, row = 26)
            self.current_challenges_condition_widgets.append(color_menu)
            type_label = ctk.CTkLabel(self.challenges_frame, text="Select the type for the part condition")
            type_label.grid(column = MIDDLE_COLUMN, row = 27)
            self.current_challenges_condition_widgets.append(type_label)
            type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["type"], values=PART_TYPES)
            type_menu.grid(column = MIDDLE_COLUMN, row = 28)
            self.current_challenges_condition_widgets.append(type_menu)
            agv_label = ctk.CTkLabel(self.challenges_frame, text="Select the agv for the condition")
            agv_label.grid(column = MIDDLE_COLUMN, row = 29)
            self.current_challenges_condition_widgets.append(agv_label)
            agv_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["agv"], values=AGV_OPTIONS)
            agv_menu.grid(column = MIDDLE_COLUMN, row = 30)
            self.current_challenges_condition_widgets.append(agv_menu)
        else:
            submission_id_label = ctk.CTkLabel(self.challenges_frame, text="Select the order id for the submission condition")
            submission_id_label.grid(column = MIDDLE_COLUMN, row = 25)
            self.current_challenges_condition_widgets.append(submission_id_label)
            submission_id_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["submission_id"], values=self.used_ids)
            submission_id_menu.grid(column = MIDDLE_COLUMN, row = 26)
            self.current_challenges_condition_widgets.append(submission_id_menu)
            
    def grid_and_append_challenge_widget(self, widget):
        widget.grid(column = MIDDLE_COLUMN, row = self.current_challenges_row, pady = 5)
        self.current_challenges_row+=1
        self.current_challenges_widgets.append(widget)

    def show_main_challenges_menu(self, _,__,___):
        self.clear_challenges_menu()

        self.grid_and_append_challenge_widget(self.add_dropped_part_button)
        self.grid_and_append_challenge_widget(self.add_robot_malfunction_button)
        self.grid_and_append_challenge_widget(self.add_sensor_blackout_button)
        if len(self.current_orders)>0:
            self.grid_and_append_challenge_widget(self.add_faulty_part_button)
        self.grid_and_append_challenge_widget(self.add_human_button)
        index = 0
        if len(self.current_challenges)>0:
            self.grid_and_append_challenge_widget(ctk.CTkLabel(self.challenges_frame,text="Current challenges:"))
        for challenge in self.current_challenges:
            challenge : ChallengeMsg

            label_text = f"Type: {CHALLENGE_TYPES[challenge.type]} "
            if challenge.type == ChallengeMsg.FAULTY_PART:
                faulty_part_challenge_quadrants = []
                if challenge.faulty_part_challenge.quadrant1:
                    faulty_part_challenge_quadrants.append(1)
                if challenge.faulty_part_challenge.quadrant2:
                    faulty_part_challenge_quadrants.append(2)
                if challenge.faulty_part_challenge.quadrant3:
                    faulty_part_challenge_quadrants.append(3)
                if challenge.faulty_part_challenge.quadrant4:
                    faulty_part_challenge_quadrants.append(4)
                label_text+=f"\nOrder_id: {challenge.faulty_part_challenge.order_id}\nQuadrants: {faulty_part_challenge_quadrants}"
            elif challenge.type == ChallengeMsg.DROPPED_PART:
                label_text+=f"\nRobot: {challenge.dropped_part_challenge.robot}\nPart: {_part_color_str[challenge.dropped_part_challenge.part_to_drop.color]} {_part_type_str[challenge.dropped_part_challenge.part_to_drop.type]}"
            elif challenge.type == ChallengeMsg.SENSOR_BLACKOUT:
                sensor_blackout_disabled_sensors = []
                if challenge.sensor_blackout_challenge.sensors_to_disable.break_beam:
                    sensor_blackout_disabled_sensors.append("break_beam")
                if challenge.sensor_blackout_challenge.sensors_to_disable.proximity:
                    sensor_blackout_disabled_sensors.append("proximity")
                if challenge.sensor_blackout_challenge.sensors_to_disable.laser_profiler:
                    sensor_blackout_disabled_sensors.append("laser_profiler")
                if challenge.sensor_blackout_challenge.sensors_to_disable.lidar:
                    sensor_blackout_disabled_sensors.append("lidar")
                if challenge.sensor_blackout_challenge.sensors_to_disable.camera:
                    sensor_blackout_disabled_sensors.append("camera")
                if challenge.sensor_blackout_challenge.sensors_to_disable.logical_camera:
                    sensor_blackout_disabled_sensors.append("logical_camera")
                label_text+=f"\nDuration: {challenge.sensor_blackout_challenge.duration}\nSensors: {sensor_blackout_disabled_sensors}"
            elif challenge.type == ChallengeMsg.ROBOT_MALFUNCTION:
                disabled_robots = []
                if challenge.robot_malfunction_challenge.robots_to_disable.floor_robot:
                    disabled_robots.append("floor_robot")
                if challenge.robot_malfunction_challenge.robots_to_disable.ceiling_robot:
                    disabled_robots.append("ceiling_robot")
                label_text+=f"\nDuration: {challenge.robot_malfunction_challenge.duration}\nRobots: {disabled_robots}"
            else:
                label_text+=f"\nBehavior: {BEHAVIORS[challenge.human_challenge.behavior]}"

            challenge_label = ctk.CTkLabel(self.challenges_frame, text=label_text)
            challenge_label.grid(column = LEFT_COLUMN, row = self.current_challenges_row, pady=3)
            self.current_challenges_widgets.append(challenge_label)

            edit_challenge_button = ctk.CTkButton(self.challenges_frame, text="Edit challenge", command = partial(self.edit_challenge, challenge, index))
            edit_challenge_button.grid(column = MIDDLE_COLUMN, row = self.current_challenges_row)
            self.current_challenges_widgets.append(edit_challenge_button)

            remove_challenge_button = ctk.CTkButton(self.challenges_frame,text="Remove challenge", command=partial(self.remove_challenge, index))
            remove_challenge_button.grid(column = RIGHT_COLUMN, row = self.current_challenges_row)
            self.current_challenges_widgets.append(remove_challenge_button)
            self.current_challenges_row+=1
            index+=1
    
    def edit_challenge(self, challenge, index):
        if challenge.type == ChallengeMsg.FAULTY_PART:
            self.add_faulty_part_challenge(challenge.faulty_part_challenge, index)
        elif challenge.type == ChallengeMsg.DROPPED_PART:
            self.add_dropped_part_challenge(challenge.dropped_part_challenge,index)
        elif challenge.type == ChallengeMsg.SENSOR_BLACKOUT:
            self.add_sensor_blackout_challenge(challenge.sensor_blackout_challenge,index)
        elif challenge.type == ChallengeMsg.ROBOT_MALFUNCTION:
            self.add_robot_malfunction_challenge(challenge.robot_malfunction_challenge, index)
        else:
            self.add_human_challenge(challenge.human_challenge, index)
    
    def remove_challenge(self, index):
        del self.current_challenges[index]
        self.challenges_counter.set(str(len(self.current_challenges)))
    
    def set_challenge_condition_info_to_existing(self, condition):
        if condition.type == ConditionMsg.TIME:
            self.challenge_condition_type.set("time")
            self.challenge_condition_info["time_condition"].set(str(condition.time_condition.seconds))
        elif condition.type == ConditionMsg.PART_PLACE:
            self.challenge_condition_type.set("part_place")
            self.challenge_condition_info["color"].set(_part_color_str[condition.part_place_condition.part.color])
            self.challenge_condition_info["type"].set(_part_type_str[condition.part_place_condition.part.type])
            self.challenge_condition_info["agv"].set(str(condition.part_place_condition.agv))
        else:
            self.challenge_condition_type.set("submission")
            self.challenge_condition_info["submission_id"].set(condition.submission_condition.order_id)
    
    def clear_challenges_menu(self):
        for widget in self.current_challenges_widgets:
            widget.grid_forget()
        self.current_challenges_widgets.clear()
        for widget in self.current_challenges_condition_widgets:
            widget.grid_forget()
        self.current_challenges_condition_widgets.clear()
        self.current_challenges_row = 1

    def reset_dropped_part_info(self):
        self.dropped_part_info["robot"].set(ROBOTS[0])
        self.dropped_part_info["color"].set(PART_COLORS[0])
        self.dropped_part_info["type"].set(PART_TYPES[0])
        self.dropped_part_info["drop_after"].set("0")
        self.dropped_part_info["delay"].set("0.0")
    
    def add_dropped_part_challenge(self, dropped_part_challenge = None, index = -1):
        self.clear_challenges_menu()

        robot_label = ctk.CTkLabel(self.challenges_frame, text="Select the robot for the dropped part")
        self.grid_and_append_challenge_widget(robot_label)
        robot_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["robot"],values=ROBOTS)
        self.grid_and_append_challenge_widget(robot_menu)

        color_label = ctk.CTkLabel(self.challenges_frame, text="Select the color of the dropped part")
        self.grid_and_append_challenge_widget(color_label)
        color_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["color"],values=PART_COLORS)
        self.grid_and_append_challenge_widget(color_menu)

        type_label = ctk.CTkLabel(self.challenges_frame, text="Select the type of the dropped part")
        self.grid_and_append_challenge_widget(type_label)
        type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["type"],values=PART_TYPES)
        self.grid_and_append_challenge_widget(type_menu)

        drop_after_label = CTkLabel(self.challenges_frame, text="Select the part number to drop after")
        self.grid_and_append_challenge_widget(drop_after_label)
        drop_after_entry = CTkEntry(self.challenges_frame, textvariable=self.dropped_part_info["drop_after"])
        self.grid_and_append_challenge_widget(drop_after_entry)

        delay_label = CTkLabel(self.challenges_frame, text="Select the time to drop after")
        self.grid_and_append_challenge_widget(delay_label)
        delay_entry = CTkEntry(self.challenges_frame, textvariable=self.dropped_part_info["delay"])
        self.grid_and_append_challenge_widget(delay_entry)

        if dropped_part_challenge!=None:
            self.dropped_part_info["robot"].set(dropped_part_challenge.robot)
            self.dropped_part_info["color"].set(_part_color_str[dropped_part_challenge.part_to_drop.color])
            self.dropped_part_info["type"].set(_part_type_str[dropped_part_challenge.part_to_drop.type])
            self.dropped_part_info["drop_after"].set(dropped_part_challenge.drop_after_num)
            self.dropped_part_info["delay"].set(dropped_part_challenge.drop_after_time)

        self.save_challenge_button.configure(text="Save dropped part challenge", command=partial(self.save_challenge, "dropped_part", index))
        self.cancel_challenge_button.grid(column = MIDDLE_COLUMN, row = 49)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
        self.save_challenge_button.grid(column = MIDDLE_COLUMN, row = 48)
        self.current_challenges_widgets.append(self.save_challenge_button)
    
    def save_dropped_part_challenge(self,index):
        new_challenge = ChallengeMsg()
        new_challenge.type = 1
        dropped_part_challenge = DroppedPartChallengeMsg()
        dropped_part_challenge.robot = self.dropped_part_info["robot"].get()
        dropped_part_challenge.part_to_drop.type = _part_type_ints[self.dropped_part_info["type"].get().upper()]
        dropped_part_challenge.part_to_drop.color = _part_color_ints[self.dropped_part_info["color"].get().upper()]
        dropped_part_challenge.drop_after_num = int(self.dropped_part_info["drop_after"].get())
        dropped_part_challenge.drop_after_time = float(self.dropped_part_info["delay"].get())
        new_challenge.dropped_part_challenge = dropped_part_challenge
        if index == -1:
            self.current_challenges.append(new_challenge)
        else:
            self.current_challenges[index] = new_challenge

    def reset_robot_malfunction_info(self):
        self.robot_malfunction_info["duration"].set('0.0')
        self.robot_malfunction_info["floor_robot"].set('0')
        self.robot_malfunction_info["ceiling_robot"].set('0')
    
    def add_robot_malfunction_challenge(self, robot_malfunction_challenge = None, index = -1):
        self.clear_challenges_menu()

        duration_label = ctk.CTkLabel(self.challenges_frame, text="Enter the duration for the robot malfunction")
        self.grid_and_append_challenge_widget(duration_label)
        duration_menu = ctk.CTkEntry(self.challenges_frame, textvariable=self.robot_malfunction_info["duration"])
        self.grid_and_append_challenge_widget(duration_menu)

        robots_label = ctk.CTkLabel(self.challenges_frame, text="Select the robot or robots you would like to malfunction")
        self.grid_and_append_challenge_widget(robots_label)
        floor_robot_cb = ctk.CTkCheckBox(self.challenges_frame,text="floor_robot",variable=self.robot_malfunction_info["floor_robot"], onvalue="1", offvalue="0", height=1, width=20)
        self.grid_and_append_challenge_widget(floor_robot_cb)
        ceiling_robot_cb = ctk.CTkCheckBox(self.challenges_frame,text="ceiling_robot",variable=self.robot_malfunction_info["ceiling_robot"], onvalue="1", offvalue="0", height=1, width=20)
        self.grid_and_append_challenge_widget(ceiling_robot_cb)
        self.show_challenges_condition_menu()

        if robot_malfunction_challenge != None:
            self.robot_malfunction_info["duration"].set(str(robot_malfunction_challenge.duration))
            self.robot_malfunction_info["floor_robot"].set("1" if robot_malfunction_challenge.robots_to_disable.floor_robot else "0")
            self.robot_malfunction_info["ceiling_robot"].set("1" if robot_malfunction_challenge.robots_to_disable.ceiling_robot else "0")
            self.set_challenge_condition_info_to_existing(robot_malfunction_challenge.condition)

        self.save_challenge_button.configure(text="Save robot malfunction challenge", command=partial(self.save_challenge, "robot_malfunction", index))
        self.cancel_challenge_button.grid(column = MIDDLE_COLUMN, row = 49)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
        self.save_challenge_button.grid(column = MIDDLE_COLUMN, row = 48)
        self.current_challenges_widgets.append(self.save_challenge_button)
        
    def save_robot_malfunction_challenge(self, index):
        new_challenge = ChallengeMsg()
        new_challenge.type = 3
        robot_malfunction_challenge = RobotMalfunctionChallengeMsg()
        robot_malfunction_challenge.duration = float(self.robot_malfunction_info["duration"].get())
        robot_malfunction_challenge.condition.type = CONDITION_TYPE.index(self.challenge_condition_type.get())
        if self.challenge_condition_type.get()=="time":
            robot_malfunction_challenge.condition.time_condition.seconds = float(self.challenge_condition_info["time_condition"].get())
        elif self.challenge_condition_type.get()=="part_place":
            robot_malfunction_challenge.condition.part_place_condition.part.color = _part_color_ints[self.challenge_condition_info["color"].get().upper()]
            robot_malfunction_challenge.condition.part_place_condition.part.type = _part_type_ints[self.challenge_condition_info["type"].get().upper()]
            robot_malfunction_challenge.condition.part_place_condition.agv = int(self.challenge_condition_info["agv"].get())
        else:
            robot_malfunction_challenge.condition.submission_condition.order_id = self.challenge_condition_info["submission_id"].get()
        robot_malfunction_challenge.robots_to_disable.floor_robot = True if self.robot_malfunction_info["floor_robot"].get()=="1" else False
        robot_malfunction_challenge.robots_to_disable.ceiling_robot = True if self.robot_malfunction_info["ceiling_robot"].get()=="1" else False
        new_challenge.robot_malfunction_challenge = robot_malfunction_challenge
        if index == -1:
            self.current_challenges.append(new_challenge)
        else:
            self.current_challenges[index] = new_challenge

    def reset_sensor_blackout_info(self):
        self.sensor_blackout_info["duration"].set('0.0')
        for sensor in SENSORS:
            self.sensor_blackout_info["sensors_to_disable"][sensor].set('0')

    def add_sensor_blackout_challenge(self, sensor_blackout_challenge = None, index = -1):
        self.clear_challenges_menu()

        duration_label = ctk.CTkLabel(self.challenges_frame, text="Enter the duration for the sensor blackout")
        self.grid_and_append_challenge_widget(duration_label)
        duration_menu = ctk.CTkEntry(self.challenges_frame, textvariable=self.sensor_blackout_info["duration"])
        self.grid_and_append_challenge_widget(duration_menu)

        sensors_label = ctk.CTkLabel(self.challenges_frame, text="Select the sensors for the sensor blackout")
        self.grid_and_append_challenge_widget(sensors_label)
        for i in range(len(self.sensor_blackout_info["sensors_to_disable"])):
            sensor_cb = ctk.CTkCheckBox(self.challenges_frame,text=SENSORS[i],variable=self.sensor_blackout_info["sensors_to_disable"][SENSORS[i]], onvalue="1", offvalue="0", height=1, width=20)
            self.grid_and_append_challenge_widget(sensor_cb)
        
        self.show_challenges_condition_menu()

        if sensor_blackout_challenge != None:
            self.sensor_blackout_info["duration"].set(str(sensor_blackout_challenge.duration))
            self.sensor_blackout_info["sensors_to_disable"]["break_beam"].set("1" if sensor_blackout_challenge.sensors_to_disable.break_beam else "0")
            self.sensor_blackout_info["sensors_to_disable"]["proximity"].set("1" if sensor_blackout_challenge.sensors_to_disable.proximity else "0")
            self.sensor_blackout_info["sensors_to_disable"]["laser_profiler"].set("1" if sensor_blackout_challenge.sensors_to_disable.laser_profiler else "0")
            self.sensor_blackout_info["sensors_to_disable"]["lidar"].set("1" if sensor_blackout_challenge.sensors_to_disable.lidar else "0")
            self.sensor_blackout_info["sensors_to_disable"]["camera"].set("1" if sensor_blackout_challenge.sensors_to_disable.camera else "0")
            self.sensor_blackout_info["sensors_to_disable"]["logical_camera"].set("1" if sensor_blackout_challenge.sensors_to_disable.logical_camera else "0")
            self.set_challenge_condition_info_to_existing(sensor_blackout_challenge.condition)

        self.save_challenge_button.configure(text="Save sensor blackout challenge", command=partial(self.save_challenge, "sensor_blackout", index))
        self.cancel_challenge_button.grid(column = MIDDLE_COLUMN, row = 49)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
        self.save_challenge_button.grid(column = MIDDLE_COLUMN, row = 48)
        self.current_challenges_widgets.append(self.save_challenge_button)
    
    def save_sensor_blackout_challenge(self, index):
        new_challenge = ChallengeMsg()
        new_challenge.type = 2
        sensor_blackout_challenge = SensorBlackoutChallengeMsg()
        sensor_blackout_challenge.duration = float(self.sensor_blackout_info["duration"].get())
        sensor_blackout_challenge.condition.type = CONDITION_TYPE.index(self.challenge_condition_type.get())
        if self.challenge_condition_type.get()=="time":
            sensor_blackout_challenge.condition.time_condition.seconds = float(self.challenge_condition_info["time_condition"].get())
        elif self.challenge_condition_type.get()=="part_place":
            sensor_blackout_challenge.condition.part_place_condition.part.color = _part_color_ints[self.challenge_condition_info["color"].get().upper()]
            sensor_blackout_challenge.condition.part_place_condition.part.type = _part_type_ints[self.challenge_condition_info["type"].get().upper()]
            sensor_blackout_challenge.condition.part_place_condition.agv = int(self.challenge_condition_info["agv"].get())
        else:
            sensor_blackout_challenge.condition.submission_condition.order_id = self.challenge_condition_info["submission_id"].get()
        sensor_blackout_challenge.sensors_to_disable.break_beam = self.sensor_blackout_info["sensors_to_disable"]["break_beam"].get() == "1"
        sensor_blackout_challenge.sensors_to_disable.camera = self.sensor_blackout_info["sensors_to_disable"]["camera"].get() == "1"
        sensor_blackout_challenge.sensors_to_disable.laser_profiler = self.sensor_blackout_info["sensors_to_disable"]["laser_profiler"].get() == "1"
        sensor_blackout_challenge.sensors_to_disable.lidar = self.sensor_blackout_info["sensors_to_disable"]["lidar"].get() == "1"
        sensor_blackout_challenge.sensors_to_disable.proximity = self.sensor_blackout_info["sensors_to_disable"]["proximity"].get() == "1"
        sensor_blackout_challenge.sensors_to_disable.logical_camera = self.sensor_blackout_info["sensors_to_disable"]["logical_camera"].get() == "1"
        new_challenge.sensor_blackout_challenge = sensor_blackout_challenge
        if index == -1:
            self.current_challenges.append(new_challenge)
        else:
            self.current_challenges[index] = new_challenge
    
    def reset_faulty_part_info(self):
        self.faulty_part_info["order_id"].set("" if len(self.used_ids) == 0 else self.used_ids[0])
        for variable in self.faulty_part_info["quadrants"]:
            variable.set('0')

    def add_faulty_part_challenge(self,faulty_part_challenge = None, index = -1):
        self.clear_challenges_menu()

        submission_id_label = ctk.CTkLabel(self.challenges_frame, text="Select the submission id for the faulty part order")
        self.grid_and_append_challenge_widget(submission_id_label)
        submission_id_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.faulty_part_info["order_id"],values=self.used_ids)
        self.grid_and_append_challenge_widget(submission_id_menu)

        quadrants_label = ctk.CTkLabel(self.challenges_frame, text="Select the quadrants for the faulty part challenge")
        self.grid_and_append_challenge_widget(quadrants_label)
        for i in range(len(QUADRANTS)):
            quadrants_cb = ctk.CTkCheckBox(self.challenges_frame,text=f"Quadrant {QUADRANTS[i]}",variable=self.faulty_part_info["quadrants"][i], onvalue="1", offvalue="0", height=1, width=20)
            self.grid_and_append_challenge_widget(quadrants_cb)
        
        if faulty_part_challenge!=None:
            self.faulty_part_info["order_id"].set(faulty_part_challenge.order_id)
            self.faulty_part_info["quadrants"][0].set("1" if faulty_part_challenge.quadrant1 else "0")
            self.faulty_part_info["quadrants"][1].set("1" if faulty_part_challenge.quadrant2 else "0")
            self.faulty_part_info["quadrants"][2].set("1" if faulty_part_challenge.quadrant3 else "0")
            self.faulty_part_info["quadrants"][3].set("1" if faulty_part_challenge.quadrant4 else "0")
        else:
            self.reset_faulty_part_info()

        self.save_challenge_button.configure(text="Save faulty part challenge", command=partial(self.save_challenge, "faulty_part", index))
        self.cancel_challenge_button.grid(column = MIDDLE_COLUMN, row = 49)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
        self.save_challenge_button.grid(column = MIDDLE_COLUMN, row = 48)
        self.current_challenges_widgets.append(self.save_challenge_button)
    
    def save_faulty_part_challenge(self, index):
        new_challenge = ChallengeMsg()
        new_challenge.type = 0
        faulty_part_challenge = FaultyPartChallengeMsg()
        faulty_part_challenge.order_id = self.faulty_part_info["order_id"].get()
        faulty_part_challenge.quadrant1 = self.faulty_part_info["quadrants"][0].get()=="1"
        faulty_part_challenge.quadrant2 = self.faulty_part_info["quadrants"][1].get()=="1"
        faulty_part_challenge.quadrant3 = self.faulty_part_info["quadrants"][2].get()=="1"
        faulty_part_challenge.quadrant4 = self.faulty_part_info["quadrants"][3].get()=="1"
        new_challenge.faulty_part_challenge = faulty_part_challenge
        if index == -1:
            self.current_challenges.append(new_challenge)
        else:
            self.current_challenges[index] = new_challenge

    def reset_human_info(self):
        self.human_info["behavior"].set(BEHAVIORS[0])
    
    def add_human_challenge(self, human_challenge = None, index = -1):
        self.clear_challenges_menu()

        behavior_label = ctk.CTkLabel(self.challenges_frame, text="Select the behavior for the human")
        self.grid_and_append_challenge_widget(behavior_label)
        behavior_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.human_info["behavior"],values=BEHAVIORS)
        self.grid_and_append_challenge_widget(behavior_menu)

        self.show_challenges_condition_menu()

        if human_challenge != None:
            self.human_info["behavior"].set(BEHAVIORS[human_challenge.behavior])
            self.set_challenge_condition_info_to_existing(human_challenge.condition)

        self.save_challenge_button.configure(text="Save human challenge", command=partial(self.save_challenge, "human", index))
        self.cancel_challenge_button.grid(column = MIDDLE_COLUMN, row = 49)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
        self.save_challenge_button.grid(column = MIDDLE_COLUMN, row = 48)
        self.current_challenges_widgets.append(self.save_challenge_button)
    
    def save_human_challenge(self, index):
        new_challenge = ChallengeMsg()
        new_challenge.type = 4
        human_challenge = HumanChallengeMsg()
        human_challenge.behavior = BEHAVIORS.index(self.human_info["behavior"].get())
        human_challenge.condition.type = CONDITION_TYPE.index(self.challenge_condition_type.get())
        if self.challenge_condition_type.get()=="time":
            human_challenge.condition.time_condition.seconds = float(self.challenge_condition_info["time_condition"].get())
        elif self.challenge_condition_type.get()=="part_place":
            human_challenge.condition.part_place_condition.part.color = _part_color_ints[self.challenge_condition_info["color"].get().upper()]
            human_challenge.condition.part_place_condition.part.type = _part_type_ints[self.challenge_condition_info["type"].get().upper()]
            human_challenge.condition.part_place_condition.agv = int(self.challenge_condition_info["agv"].get())
        else:
            human_challenge.condition.submission_condition.order_id = self.challenge_condition_info["submission_id"].get()
        new_challenge.human_challenge = human_challenge
        if index == -1:
            self.current_challenges.append(new_challenge)
        else:
            self.current_challenges[index] = new_challenge

    def save_challenge(self, type_of_challenge:str, index):
        if type_of_challenge == "dropped_part":
            self.save_dropped_part_challenge(index)
        elif type_of_challenge == "robot_malfunction":
            self.save_robot_malfunction_challenge(index)
        elif type_of_challenge == "sensor_blackout":
            self.save_sensor_blackout_challenge(index)
        elif type_of_challenge == "faulty_part":
            self.save_faulty_part_challenge(index)
        else:
            self.save_human_challenge(index)
        self.reset_all_challenges()
        self.clear_challenges_menu()
        self.show_main_challenges_menu(1,1,1)
    
    def reset_all_challenges(self):
        self.reset_challenge_condition_variables()
        self.reset_dropped_part_info()
        self.reset_faulty_part_info()
        self.reset_human_info()
        self.reset_robot_malfunction_info()
        self.reset_sensor_blackout_info()
    
    def challenges_to_dict(self):
        if len(self.current_challenges)>0:
            self.challenges_dict["challenges"] = []
            for challenge in self.current_challenges:
                challenge:ChallengeMsg
                if challenge.type == ChallengeMsg.DROPPED_PART:
                    dropped_part_dict = {"dropped_part":{}}
                    dropped_part_dict["dropped_part"]["robot"] = challenge.dropped_part_challenge.robot
                    dropped_part_dict["dropped_part"]["type"] = _part_type_str[challenge.dropped_part_challenge.part_to_drop.type]
                    dropped_part_dict["dropped_part"]["color"] = _part_color_str[challenge.dropped_part_challenge.part_to_drop.color]
                    dropped_part_dict["dropped_part"]["drop_after"] = challenge.dropped_part_challenge.drop_after_num
                    dropped_part_dict["dropped_part"]["delay"] = challenge.dropped_part_challenge.drop_after_time
                    self.challenges_dict["challenges"].append(dropped_part_dict)

                elif challenge.type == ChallengeMsg.FAULTY_PART:
                    faulty_part_dict = {"faulty_part":{}}
                    faulty_part_dict["faulty_part"]["order_id"] = challenge.faulty_part_challenge.order_id
                    if challenge.faulty_part_challenge.quadrant1:
                        faulty_part_dict["faulty_part"]["quadrant1"] = True
                    if challenge.faulty_part_challenge.quadrant2:
                        faulty_part_dict["faulty_part"]["quadrant2"] = True
                    if challenge.faulty_part_challenge.quadrant3:
                        faulty_part_dict["faulty_part"]["quadrant3"] = True
                    if challenge.faulty_part_challenge.quadrant4:
                        faulty_part_dict["faulty_part"]["quadrant4"] = True
                    self.challenges_dict["challenges"].append(faulty_part_dict)

                elif challenge.type == ChallengeMsg.HUMAN:
                    human_dict = {"human":{}}
                    human_dict["human"]["behavior"] = BEHAVIORS[challenge.human_challenge.behavior]
                    condition_dict = self.announcement_to_dict(challenge.human_challenge.condition)
                    for key in condition_dict.keys():
                        human_dict["human"][key] = condition_dict[key]
                    self.challenges_dict["challenges"].append(human_dict)

                elif challenge.type == ChallengeMsg.ROBOT_MALFUNCTION:
                    robot_malfunction_dict = {"robot_malfunction":{}}
                    robot_malfunction_dict["robot_malfunction"]["duration"] = challenge.robot_malfunction_challenge.duration
                    robot_malfunction_dict["robot_malfunction"]["robots_to_disable"] = []
                    if challenge.robot_malfunction_challenge.robots_to_disable.floor_robot:
                        robot_malfunction_dict["robot_malfunction"]["robots_to_disable"].append("floor_robot")
                    if challenge.robot_malfunction_challenge.robots_to_disable.ceiling_robot:
                        robot_malfunction_dict["robot_malfunction"]["robots_to_disable"].append("ceiling_robot")
                    condition_dict = self.announcement_to_dict(challenge.robot_malfunction_challenge.condition)
                    for key in condition_dict.keys():
                        robot_malfunction_dict["robot_malfunction"][key] = copy(condition_dict[key])
                    self.challenges_dict["challenges"].append(robot_malfunction_dict)
                    

                else:
                    sensor_blackout_dict = {"sensor_blackout":{}}
                    sensor_blackout_dict["sensor_blackout"]["duration"] = challenge.sensor_blackout_challenge.duration
                    sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"] = []
                    if challenge.sensor_blackout_challenge.sensors_to_disable.break_beam:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("break_beam")
                    if challenge.sensor_blackout_challenge.sensors_to_disable.lidar:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("lidar")
                    if challenge.sensor_blackout_challenge.sensors_to_disable.laser_profiler:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("laser_profiler")
                    if challenge.sensor_blackout_challenge.sensors_to_disable.logical_camera:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("logical_camera")
                    if challenge.sensor_blackout_challenge.sensors_to_disable.camera:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("camera")
                    if challenge.sensor_blackout_challenge.sensors_to_disable.proximity:
                        sensor_blackout_dict["sensor_blackout"]["sensors_to_disable"].append("proximity")
                    condition_dict = self.announcement_to_dict(challenge.sensor_blackout_challenge.condition)
                    for key in condition_dict.keys():
                        sensor_blackout_dict["sensor_blackout"][key] = copy(condition_dict[key])
                    self.challenges_dict["challenges"].append(sensor_blackout_dict)

    # =======================================================
    #              Save configuration file
    # =======================================================
    def build_file_dict(self):
        self.kitting_trays_to_dict()
        self.assembly_inserts_to_dict()
        self.agv_parts_to_dict()
        self.bin_parts_to_dict()
        self.conveyor_parts_to_dict()
        self.orders_to_dict()
        self.challenges_to_dict()

    def choose_save_location(self, window = None):
        if window != None:
            window.destroy()
        if self.trial_name.get()=="":
            file_to_open=filedialog.asksaveasfile(defaultextension=".yaml", filetypes=[("YAML file", ".yaml")], initialdir=self.trials_file_location,title='Save ARIAC configuration')
        else:
            file_to_open=filedialog.asksaveasfile(defaultextension=".yaml", filetypes=[("YAML file", ".yaml")], initialdir=self.trials_file_location,title='Save ARIAC configuration',initialfile=f"{self.trial_name.get()}")
        try:
            self.file_name = file_to_open.name
            self.save_file()
        except:
            pass
    
    def run_overwrite_window(self):
        overwrite_window = ctk.CTkToplevel()
        overwrite_question_label = ctk.CTkLabel(overwrite_window, text=f"Would you like to overwrite {self.trial_name.get()}.yaml?")
        overwrite_question_label.grid(column = MIDDLE_COLUMN, row = 0)
        overwrite_yes_button = ctk.CTkButton(overwrite_window, text="Yes", command = self.save_file)
        overwrite_yes_button.grid(column = LEFT_COLUMN, row = 1)
        overwrite_no_button = ctk.CTkButton(overwrite_window, text="No", command = partial(self.choose_save_location, overwrite_window))
        overwrite_no_button.grid(column = RIGHT_COLUMN, row = 1)
        overwrite_window.mainloop()
    
    def save_file(self):
        self.build_file_dict()
        with open(self.file_name,'w') as f:
            f.write(f"# Trial name: {self.trial_name.get()}.yaml\n")
            f.write("# ARIAC2024\n")
            f.write(f"# Author: {self.author.get()}\n")
            f.write(f"# {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n# ENVIRONMENT SETUP\n\n")
            f.write(f"time_limit: {self.time_limit.get()}\n\n")

            kitting_trays_data = yaml.dump(self.kitting_trays_dict,sort_keys=False,Dumper=NoAliasDumper)
            f.write(f"\n{kitting_trays_data}\n")

            assembly_inserts_data = yaml.dump(self.assembly_inserts_dict,sort_keys=False,Dumper=NoAliasDumper)
            f.write(f"\n{assembly_inserts_data}\n")

            if self.has_parts.get()=="1":
                parts_dict = {"parts":{"agvs":self.agv_parts_dict,"bins":self.bin_parts_dict,"conveyor_belt":self.conveyor_parts_dict}}
            else:
                parts_dict = {"parts":{"agvs":self.agv_parts_dict,"bins":self.bin_parts_dict}}
            parts_data = yaml.dump(parts_dict,sort_keys=False,Dumper=NoAliasDumper)
            f.write(f"\n{parts_data}\n")

            orders_data = yaml.dump(self.orders_dict,sort_keys=False,Dumper=NoAliasDumper)
            f.write(f"\n{orders_data}\n")

            challenges_data = yaml.dump(self.challenges_dict,sort_keys=False,Dumper=NoAliasDumper)
            f.write(f"\n{challenges_data}\n")
        self.destroy()
                  
    # =======================================================
    #               General Gui Functions
    # =======================================================
    def nearest_slider_value(self,value,slider,label,_,__,___):
        newvalue = min(SLIDER_VALUES, key=lambda x:abs(x-float(value.get())))
        slider.set(newvalue)
        label.configure(text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(newvalue)]}")
    
    def nearest_slider_value_assembly_inserts(self,value,slider,label,station,_,__,___):
        newvalue = min(SLIDER_VALUES, key=lambda x:abs(x-float(value.get())))
        slider.set(newvalue)
        new_text = f"{station} rotation value: {SLIDER_STR[SLIDER_VALUES.index(newvalue)]}"
        label.configure(text=new_text)
    
    def announcement_to_dict(self, announcement : ConditionMsg)->dict:
        temp_announcement_dict = {}
        if announcement.type == 0:
            temp_announcement_dict["time_condition"] = announcement.time_condition.seconds
        elif announcement.type == 1:
            temp_announcement_dict["part_place_condition"] = {"color":_part_color_str[announcement.part_place_condition.part.color],
                                                              "type":_part_type_str[announcement.part_place_condition.part.type],
                                                              "agv": announcement.part_place_condition.agv}
        else:
            temp_announcement_dict["submission_condition"] = {"order_id":announcement.submission_condition.order_id}
        return temp_announcement_dict


if __name__=="__main__":
    app = GUI_CLASS()
    app.mainloop()