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
from turtle import right
from PIL import Image  # needed for images in gui
from math import pi
import random
import string
import yaml

from click import command
from ariac_msgs.msg import (
    Part as PartMsg,
    PartLot as PartLotMsg,
    OrderCondition as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    KittingPart as KittingPartMsg,
    KittingTask as KittingTaskMsg,
    AssemblyTask as AssemblyTaskMsg,
    CombinedTask as CombinedTaskMsg,
    BinParts as BinPartsMsg,
    BinInfo as BinInfoMsg,
    ConveyorParts as ConveyorPartsMsg,
    Condition as ConditionMsg,
    TimeCondition as TimeConditionMsg,
    PartPlaceCondition as PartPlaceConditionMsg,
    SubmissionCondition as SubmissionConditionMsg,
    FaultyPartChallenge as FaultyPartChallengeMsg,
    DroppedPartChallenge as DroppedPartChallengeMsg,
    SensorBlackoutChallenge as SensorBlackoutChallengeMsg,
    RobotMalfunctionChallenge as RobotMalfunctionChallengeMsg,
    HumanChallenge as HumanChallengeMsg,
)
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, Quaternion
from ament_index_python.packages import get_package_share_directory
from ariac_gui.utils import (build_competition_from_file, quaternion_from_euler, 
                             rpy_from_quaternion, 
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
ASSEMBLY_STATIONS=["as1","as2","as3","as4"]
CONDITION_TYPE=['time','part_place']
TRAY_IDS=[str(i) for i in range(10)]

SENSORS = ["break_beam", "proximity", "laser_profiler", "lidar", "camera", "logical_camera"]
ROBOTS = ["floor_robot", "ceiling_robot"]
BEHAVIORS = ["antagonistic","indifferent","helpful"]


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

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(pady=10,column=MIDDLE_COLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

        # Setup info
        self.time_limit = ctk.StringVar()
        self.trial_name = ctk.StringVar()
        self.author = ctk.StringVar()
        self.time_limit.set('0')
        self.trial_name.set('')
        self.author.set('')

        # Kitting tray info
        self.kitting_tray_selections = [ctk.StringVar() for _ in range(6)]

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

        # Order widgets
        self.current_left_order_widgets = []
        self.current_right_order_widgets = []
        self.current_main_order_widgets = []

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

        self.current_orders = []

        # Challenges widgets
        self.current_challenges_widgets = []
        self.current_challenges_condition_widgets = []

        # Dropped part challenge variables
        self.dropped_part_info = {}
        self.dropped_part_info["robot"] = ctk.StringVar()
        self.dropped_part_info["type"] = ctk.StringVar()
        self.dropped_part_info["color"] = ctk.StringVar()
        self.dropped_part_info["drop_after"] = ctk.StringVar()
        self.dropped_part_info["delay"] = ctk.StringVar()

        # Robot malfunction challenge variables
        self.robot_malfunction_info = {}
        self.robot_malfunction_info["duration"] = ctk.StringVar()
        self.robot_malfunction_info["floor_robot"] = ctk.StringVar()
        self.robot_malfunction_info["ceiling_robot"] = ctk.StringVar()

        # Sensor blackout challenge variables
        self.sensor_blackout_info = {}
        self.sensor_blackout_info["duration"] = ctk.StringVar()
        self.sensor_blackout_info["sensors_to_disable"] = [ctk.StringVar() for _ in range(len(SENSORS))]

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

        # List to hold saved challenges
        self.current_challenges = []

        # Menu tabs
        self.setup_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.setup_frame.pack(fill='both',expand=True)
        self.notebook.add(self.setup_frame,text="Setup")
        self.add_setup_widgets_to_frame()

        self.kitting_tray_frame = ttk.Frame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.kitting_tray_frame.pack(fill='both',expand=True)
        self.notebook.add(self.kitting_tray_frame,text="Kitting Trays")
        self.add_kitting_trays_widgets_to_frame()

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

        self.load_in_from_file_button = ctk.CTkButton(self, text="Load in data from file", command=self._load_file)
        self.load_in_from_file_button.grid(pady=10,column=MIDDLE_COLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

        self._build_assembly_parts_pose_direction()
    
    # =======================================================
    #            Load gui from a previous file
    # =======================================================

    def _load_file(self):
        file_to_open=filedialog.askopenfile("r", filetypes =[('Yaml Files', '*.yaml')])
        with open(file_to_open.name) as f:
            yaml_dict = yaml.load(f, Loader=yaml.SafeLoader)
        self.trial_name.set(file_to_open.name.split("/")[-1].replace(".yaml",""))
        self._load_options_from_competition_class(build_competition_from_file(yaml_dict))

    def _load_options_from_competition_class(self, competition: CompetitionClass):
        self.time_limit.set(competition.competition["time_limit"])

        self.bin_parts = competition.competition["bin_parts"]
        self.current_bin_parts = competition.competition["current_bin_parts"]
        self.bin_parts_counter.set(str(len([1 for part in self.current_bin_parts if part !=""])))

        self.conveyor_setup_vals["active"].set(competition.competition["conveyor_belt"]["active"])
        self.conveyor_setup_vals["spawn_rate"].set(competition.competition["conveyor_belt"]["spawn_rate"])
        self.conveyor_setup_vals["order"].set(competition.competition["conveyor_belt"]["order"])
        self.conveyor_parts = competition.competition["conveyor_belt"]["parts_to_spawn"]
        self.current_conveyor_parts = competition.competition["conveyor_belt"]["current_conveyor_parts"]
        self.conveyor_parts_counter.set(str(len(self.conveyor_parts)))

        self.current_orders = competition.competition["orders"]
        self.order_counter.set(str(len(self.current_orders)))
        for order in self.current_orders:
            self.used_ids.append(order.id)

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
        tray_label = ctk.CTkLabel(self.kitting_tray_frame,text="Select the tray ids for each slot")
        tray_label.pack()
        kitting_tray_canvas = Canvas(self.kitting_tray_frame)
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
            kitting_tray_canvas.create_window(label_coordinates[i], window=ctk.CTkLabel(self.kitting_tray_frame, text=f"{i}:"))
            kitting_tray_canvas.create_window(menu_coordinates[i], window = tray_menus[i])
        kitting_tray_canvas.create_window((90,325),window=ctk.CTkLabel(self.kitting_tray_frame,text="kts_1"))
        kitting_tray_canvas.create_window((310,325),window=ctk.CTkLabel(self.kitting_tray_frame,text="kts_2"))
        kitting_tray_canvas.pack(fill = BOTH, expand = 1)
    
    # =======================================================
    #                 Bin Parts Functions
    # =======================================================
    def add_bin_parts_widgets_to_frame(self):
        bin_selection = ctk.StringVar()
        bin_selection.set(ALL_BINS[0])
        bin_label = ctk.CTkLabel(self.bin_parts_frame,text="Select the bin you would like to add parts to:")
        bin_label.pack()
        bin_menu = ctk.CTkOptionMenu(self.bin_parts_frame,
                                        variable=bin_selection,
                                        values=ALL_BINS,
                                        fg_color = "#e2e2e2",
                                        text_color="black",
                                        button_color="#d3d3d3",
                                        button_hover_color="#9e9e9e",
                                        anchor='center',
                                        )
        bin_menu.pack()

        bin_parts_canvas = Canvas(self.bin_parts_frame)
        
        bin_parts_canvas.create_rectangle(50, 10, 350, 310, 
                                outline = "black", fill = "#60c6f1",
                                width = 2)
        self.show_grid(bin_selection,bin_parts_canvas,self.bin_parts_frame)
        bin_parts_canvas.pack(fill = BOTH, expand = 1)
        add_multiple_parts_button = ctk.CTkButton(self.bin_parts_frame,text="Add multiple parts",command=partial(self.add_multiple_parts,bin_selection.get()))
        add_multiple_parts_button.pack(pady=15)
        flipped_meaning_label = ctk.CTkLabel(self.bin_parts_frame, text="When a part if flipped, an \"F\" will show up in the bottom right of the part image.")
        flipped_meaning_label.pack(pady=10)
        bin_selection.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))
        self.bin_parts_counter.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))

    def show_grid(self,bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk):
        button_coordinates = [(100,60),(200,60),(300,60),
                            (100,160),(200,160),(300,160),
                            (100,260),(200,260),(300,260)]
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
            self.current_bin_parts[bin][index]=color+pType
            temp_part = PartMsg()
            temp_part.color = _part_color_ints[color.upper()]
            temp_part.type = _part_type_ints[pType.upper()]
            self.bin_parts[bin][index].part = temp_part
            self.bin_parts[bin][index].rotation = bin_vals["rotation"].get() 
            self.bin_parts[bin][index].flipped = bin_vals["flipped"].get()
        self.bin_parts_counter.set(str(int(self.bin_parts_counter.get())+len(slot_indices)))
        window.destroy()

    # =======================================================
    #               Conveyor Parts Functions
    # =======================================================
    def add_conveyor_parts_widgets_to_frame(self):
        has_parts = ctk.StringVar()
        has_parts.set("0")
        trial_has_parts_cb = ctk.CTkCheckBox(self.conveyor_parts_frame,text="Trial has conveyor parts",variable=has_parts, onvalue="1", offvalue="0", height=1, width=20)
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
        flipped_meaning_label = ctk.CTkLabel(self.conveyor_parts_frame, text="When a part if flipped, an \"F\" will show up in the bottom right of the part image.")
        flipped_meaning_label.pack(pady=10)
        self.present_conveyor_widgets.append(add_parts_button)
        self.conveyor_setup_vals["spawn_rate"].trace('w',partial(self.update_spawn_rate_slider,self.conveyor_setup_vals["spawn_rate"],spawn_rate_label))
        has_parts.trace('w', partial(self.activate_deactivate_menu, has_parts,self.conveyor_setup_vals))
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
                self.current_bin_canvas_elements.append(canvas.create_window(flipped_label_coordinates[i], window=current_flipped_labels[i]))

    def remove_conveyor_part(self, index):
        del self.current_conveyor_parts[index]
        self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))

    def update_spawn_rate_slider(self,value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
        label.configure(text=f"current spawn rate (seconds): {value.get()}")

    def activate_deactivate_menu(self,has_parts:ctk.StringVar,conveyor_setup_vals,_,__,___):
        if has_parts.get()=="1":
            for widget in self.present_conveyor_widgets:
                widget.configure(state=tk.NORMAL)
        else:
            for widget in self.present_conveyor_widgets:
                widget.configure(state=tk.DISABLED)
            conveyor_setup_vals["active"].set('1')
            conveyor_setup_vals['spawn_rate'].set(1)
            conveyor_setup_vals["order"].set(CONVEYOR_ORDERS[0])
    
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
        if index == -1:    
            self.conveyor_parts.append(ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get(), conveyor_part_vals["flipped"].get()))
            self.current_conveyor_parts.append(color+pType)
            self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))
        else:
            self.conveyor_parts[index] = ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get(),conveyor_part_vals["flipped"].get())
            self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))
        window.destroy()

    # =======================================================
    #                 Order Functions
    # =======================================================
    def add_order_widgets_to_frame(self):
        self.add_kitting_order_button = ctk.CTkButton(self.orders_frame, text="Add kitting order", command=self.add_kitting_order)
        self.add_assembly_order_button = ctk.CTkButton(self.orders_frame, text="Add assembly order", command=self.add_assembly_order)
        self.add_combined_order_button = ctk.CTkButton(self.orders_frame, text="Add combined order", command=self.add_combined_order)

        self.show_main_order_menu()

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
            edit_order_button.grid(column=MIDDLE_COLUMN, row=current_row+i)
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
        self.order_info["priority"].set('1' if order.priority else '0')
        self.order_info["announcement_type"].set(CONDITION_TYPE[order.condition.type])
        if order.condition.type == 0:
            self.order_info["announcement"]["time_condition"].set(str(order.condition.time_condition.seconds))
        elif order.condition.type == 1:
            self.order_info["announcement"]["color"].set(_part_color_str[order.condition.part_place_condition.part.color])
            self.order_info["announcement"]["type"].set(_part_type_str[order.condition.part_place_condition.part.type])
            self.order_info["announcement"]["agv"].set(str(order.condition.part_place_condition.agv))
        else:
            self.order_info["announcement"]["submission_id"].set(str(order.condition.submission_condition.order_id))
        self.order_info["priority"].set('1' if order.priority else'0')
        
        if order.type == 0:
            self.order_info["kitting_task"]["agv_number"].set(str(order.kitting_task.agv_number))
            self.order_info["kitting_task"]["tray_id"].set(str(order.kitting_task.tray_id))
            self.order_info["kitting_task"]["parts"] = order.kitting_task.parts

        elif order.type == 1:
            print(order.assembly_task.station)
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
        self.save_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+1)
        self.current_main_order_widgets.append(self.save_order_button)
        self.cancel_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+2)
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
        self.move_save_cancel_order_buttons()
        
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

    def show_correct_menu(self,_,__,___):
        self.left_row_index = 1
        if self.order_info["order_type"].get() == "kitting":
            self.show_kitting_menu()
        elif self.order_info["order_type"].get() == "assembly":
            self.show_assembly_menu()
        else:
            self.show_combined_menu()
    
    def move_save_cancel_order_buttons(self):
        self.save_order_button.grid_forget()
        self.save_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+1)
        self.cancel_order_button.grid_forget()
        self.cancel_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+2)
    
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

        add_part_kitting_task = ctk.CTkButton(self.orders_frame, text="Add part", command=self.add_kitting_part)
        self.grid_left_column(add_part_kitting_task)
        self.current_left_order_widgets.append(add_part_kitting_task)

        self.cancel_order_button.configure(text="Cancel kitting order")
        self.save_order_button.configure(text="Save kitting order")
    
    def add_kitting_part(self):
        add_k_part_wind = ctk.CTkToplevel()

        k_part_dict = {}
        k_part_dict["color"] = ctk.StringVar()
        k_part_dict["pType"] = ctk.StringVar()
        k_part_dict["quadrant"] = ctk.StringVar()

        k_part_dict["color"].set(PART_COLORS[0])
        k_part_dict["pType"].set(PART_TYPES[0])
        k_part_dict["quadrant"].set(QUADRANTS[0])

        color_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()
        quadrant_menu = ctk.CTkOptionMenu(add_k_part_wind, variable=k_part_dict["quadrant"], values=QUADRANTS)
        quadrant_menu.pack()

        save_button = ctk.CTkButton(add_k_part_wind, text="Save kitting part", command=partial(self.save_kitting_part, k_part_dict, add_k_part_wind))
        save_button.pack(pady = 10)
        add_k_part_wind.mainloop()

    def save_kitting_part(self, k_part_dict, window):
        new_kitting_part = KittingPartMsg()
        new_kitting_part.part.color = _part_color_ints[k_part_dict["color"].get().upper()]
        new_kitting_part.part.type = _part_type_ints[k_part_dict["pType"].get().upper()]
        new_kitting_part.quadrant = int(k_part_dict["quadrant"].get())
        self.order_info["kitting_task"]["parts"].append(new_kitting_part)
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
        self.save_order_button.configure(text="Save assembly order")
    
    def add_assembly_part(self):
        add_a_part_wind = ctk.CTkToplevel()

        a_part_dict = {}
        a_part_dict["color"] = ctk.StringVar()
        a_part_dict["pType"] = ctk.StringVar()

        a_part_dict["color"].set(PART_COLORS[0])
        a_part_dict["pType"].set(PART_TYPES[0])

        color_menu = ctk.CTkOptionMenu(add_a_part_wind, variable=a_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_a_part_wind, variable=a_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()

        save_button = ctk.CTkButton(add_a_part_wind, text="Save assembly part", command=partial(self.save_assembly_part, a_part_dict, add_a_part_wind))
        save_button.pack(pady = 10)
        add_a_part_wind.mainloop()

    def save_assembly_part(self, a_part_dict, window):
        new_assembly_part = AssemblyPartMsg()

        new_assembly_part.part.color = _part_color_ints[a_part_dict["color"].get().upper()]
        new_assembly_part.part.type = _part_type_ints[a_part_dict["pType"].get().upper()]
        new_assembly_part.assembled_pose = _assembly_part_poses[a_part_dict["pType"].get().upper()]
        new_assembly_part.install_direction = _assembly_part_install_directions[a_part_dict["pType"].get().upper()]
        self.order_info["combined_task"]["parts"].append(new_assembly_part)
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
        self.save_order_button.configure(text="Save combined order")
    
    def add_combined_part(self):
        add_c_part_wind = ctk.CTkToplevel()

        c_part_dict = {}
        c_part_dict["color"] = ctk.StringVar()
        c_part_dict["pType"] = ctk.StringVar()

        c_part_dict["color"].set(PART_COLORS[0])
        c_part_dict["pType"].set(PART_TYPES[0])

        color_menu = ctk.CTkOptionMenu(add_c_part_wind, variable=c_part_dict["color"],values=PART_COLORS)
        color_menu.pack()
        type_menu = ctk.CTkOptionMenu(add_c_part_wind, variable=c_part_dict["pType"],values=PART_TYPES)
        type_menu.pack()

        save_button = ctk.CTkButton(add_c_part_wind, text="Save combined part", command=partial(self.save_combined_part, c_part_dict, add_c_part_wind))
        save_button.pack(pady = 10)
        add_c_part_wind.mainloop()

    def save_combined_part(self, c_part_dict, window):
        new_combined_part = AssemblyPartMsg()
        new_part = PartMsg()
        new_part.color = _part_color_ints[c_part_dict["color"].get().upper()]
        new_part.type = _part_type_ints[c_part_dict["pType"].get().upper()]
        
        new_combined_part.part = new_part
        new_combined_part.assembled_pose = _assembly_part_poses[c_part_dict["pType"].get().upper()]
        new_combined_part.install_direction = _assembly_part_install_directions[c_part_dict["pType"].get().upper()]
        self.order_info["combined_task"]["parts"].append(new_combined_part)
        window.destroy()

    def clear_order_menu(self):
        for widget_list in [self.current_left_order_widgets,self.current_right_order_widgets, self.current_main_order_widgets]:
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
        new_order.priority = True if self.order_info["priority"].get() == 1 else False
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
        new_assembly_task.agv_numbers = [i+1 
                                         for i in range(len(self.order_info["assembly_task"]["agv_numbers"]))
                                         if self.order_info["assembly_task"]["agv_numbers"][i].get()=="1"]
        new_assembly_task.station = ASSEMBLY_STATIONS.index(self.order_info["assembly_task"]["station"].get())
        new_assembly_task.parts = self.order_info["assembly_task"]["parts"]
        return new_assembly_task

    def create_combined_task_msg(self)->CombinedTaskMsg:
        new_combined_task = CombinedTaskMsg()
        new_combined_task.station = ASSEMBLY_STATIONS.index(self.order_info["combined_task"]["station"].get())
        new_combined_task.parts = self.order_info["combined_task"]["parts"]
        return new_combined_task
    
    # =======================================================
    #                  Challenges functions
    # =======================================================

    def add_challenges_widgets_to_frame(self):
        self.reset_all_challenges()
        self.add_dropped_part_button = ctk.CTkButton(self.challenges_frame, text="Add dropped part challenge", command=self.add_dropped_part_challenge)
        self.add_robot_malfunction_button = ctk.CTkButton(self.challenges_frame, text="Add robot malfunction challenge", command=self.add_robot_malfunction_challenge)
        self.add_sensor_blackout_button = ctk.CTkButton(self.challenges_frame, text="Add sensor blackout challenge", command=self.add_sensor_blackout_challenge)
        self.add_faulty_part_button = ctk.CTkButton(self.challenges_frame, text = "Add faulty part challenge", command=self.add_faulty_part_challenge)
        self.add_human_button = ctk.CTkButton(self.challenges_frame, text="Add human challenge", command = self.add_human_challenge)

        self.show_main_challenges_menu(1,1,1)

        self.save_challenge_button = ctk.CTkButton(self.challenges_frame,text="Save challenge", command=self.save_challenge)
        self.cancel_challenge_button = ctk.CTkButton(self.challenges_frame,text="Cancel challenge", command=partial(self.show_main_challenges_menu,1,1,1))
        self.order_counter.trace('w', self.show_main_challenges_menu)
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
        self.pack_and_append_challenge_widget(challenge_condition_type_label)
        challenge_condition_type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_type,values=CONDITION_TYPE)
        self.pack_and_append_challenge_widget(challenge_condition_type_menu)
        self.reset_challenge_condition_variables()
    
    def show_correct_condition_menu(self,_,__,____):
        for widget in self.current_challenges_condition_widgets:
            widget.pack_forget()
        self.current_challenges_condition_widgets.clear()
        self.reset_challenge_condition_variables(False)
        if self.challenge_condition_type.get() == "time":
            time_label = ctk.CTkLabel(self.challenges_frame, text="Enter the time for the time_condition:")
            time_label.pack()
            self.current_challenges_condition_widgets.append(time_label)
            time_entry = ctk.CTkEntry(self.challenges_frame, textvariable=self.challenge_condition_info["time_condition"])
            time_entry.pack()
            self.current_challenges_condition_widgets.append(time_entry)
        elif self.challenge_condition_type.get() == "part_place":
            color_label = ctk.CTkLabel(self.challenges_frame, text="Select the color for the part condition")
            color_label.pack()
            self.current_challenges_condition_widgets.append(color_label)
            color_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["color"], values=PART_COLORS)
            color_menu.pack()
            self.current_challenges_condition_widgets.append(color_menu)
            type_label = ctk.CTkLabel(self.challenges_frame, text="Select the type for the part condition")
            type_label.pack()
            self.current_challenges_condition_widgets.append(type_label)
            type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["type"], values=PART_TYPES)
            type_menu.pack()
            self.current_challenges_condition_widgets.append(type_menu)
            agv_label = ctk.CTkLabel(self.challenges_frame, text="Select the agv for the condition")
            agv_label.pack()
            self.current_challenges_condition_widgets.append(agv_label)
            agv_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["agv"], values=AGV_OPTIONS)
            agv_menu.pack()
            self.current_challenges_condition_widgets.append(agv_menu)
        else:
            submission_id_label = ctk.CTkLabel(self.challenges_frame, text="Select the order id for the submission condition")
            submission_id_label.pack()
            self.current_challenges_condition_widgets.append(submission_id_label)
            submission_id_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.challenge_condition_info["submission_id"], values=self.used_ids)
            submission_id_menu.pack()
            self.current_challenges_condition_widgets.append(submission_id_menu)
            
    def pack_and_append_challenge_widget(self, widget):
        widget.pack(pady=10)
        self.current_challenges_widgets.append(widget)

    def show_main_challenges_menu(self, _,__,___):
        self.clear_challenges_menu()

        self.pack_and_append_challenge_widget(self.add_dropped_part_button)
        self.pack_and_append_challenge_widget(self.add_robot_malfunction_button)
        self.pack_and_append_challenge_widget(self.add_sensor_blackout_button)
        if len(self.used_ids)>0:
            self.pack_and_append_challenge_widget(self.add_faulty_part_button)
        self.pack_and_append_challenge_widget(self.add_human_button)
    
    def clear_challenges_menu(self):
        for widget in self.current_challenges_widgets:
            widget.pack_forget()
        self.current_challenges_widgets.clear()
        for widget in self.current_challenges_condition_widgets:
            widget.pack_forget()
        self.current_challenges_condition_widgets.clear()

    def reset_dropped_part_info(self):
        self.dropped_part_info["robot"].set(ROBOTS[0])
        self.dropped_part_info["color"].set(PART_COLORS[0])
        self.dropped_part_info["type"].set(PART_TYPES[0])
        self.dropped_part_info["drop_after"].set("0")
        self.dropped_part_info["delay"].set("0.0")
    
    def add_dropped_part_challenge(self):
        self.clear_challenges_menu()

        robot_label = ctk.CTkLabel(self.challenges_frame, text="Select the robot for the dropped part")
        self.pack_and_append_challenge_widget(robot_label)
        robot_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["robot"],values=ROBOTS)
        self.pack_and_append_challenge_widget(robot_menu)

        color_label = ctk.CTkLabel(self.challenges_frame, text="Select the color of the dropped part")
        self.pack_and_append_challenge_widget(color_label)
        color_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["color"],values=PART_COLORS)
        self.pack_and_append_challenge_widget(color_menu)

        type_label = ctk.CTkLabel(self.challenges_frame, text="Select the type of the dropped part")
        self.pack_and_append_challenge_widget(type_label)
        type_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.dropped_part_info["type"],values=PART_TYPES)
        self.pack_and_append_challenge_widget(type_menu)

        drop_after_label = CTkLabel(self.challenges_frame, text="Select the part number to drop after")
        self.pack_and_append_challenge_widget(drop_after_label)
        drop_after_entry = CTkEntry(self.challenges_frame, textvariable=self.dropped_part_info["drop_after"])
        self.pack_and_append_challenge_widget(drop_after_entry)

        delay_label = CTkLabel(self.challenges_frame, text="Select the time to drop after")
        self.pack_and_append_challenge_widget(delay_label)
        delay_entry = CTkEntry(self.challenges_frame, textvariable=self.dropped_part_info["delay"])
        self.pack_and_append_challenge_widget(delay_entry)

        self.save_challenge_button.configure(text="Save dropped part challenge", command=partial(self.save_challenge, "dropped_part"))
        self.save_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.save_challenge_button)
        self.cancel_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.cancel_challenge_button)

    def reset_robot_malfunction_info(self):
        self.robot_malfunction_info["duration"].set('0.0')
        self.robot_malfunction_info["floor_robot"].set('0')
        self.robot_malfunction_info["ceiling_robot"].set('0')
    
    def add_robot_malfunction_challenge(self):
        self.clear_challenges_menu()

        duration_label = ctk.CTkLabel(self.challenges_frame, text="Enter the duration for the robot malfunction")
        self.pack_and_append_challenge_widget(duration_label)
        duration_menu = ctk.CTkEntry(self.challenges_frame, textvariable=self.robot_malfunction_info["duration"])
        self.pack_and_append_challenge_widget(duration_menu)

        robots_label = ctk.CTkLabel(self.challenges_frame, text="Select the robot or robots you would like to malfunction")
        self.pack_and_append_challenge_widget(robots_label)
        floor_robot_cb = ctk.CTkCheckBox(self.challenges_frame,text="floor_robot",variable=self.robot_malfunction_info["floor_robot"], onvalue="1", offvalue="0", height=1, width=20)
        self.pack_and_append_challenge_widget(floor_robot_cb)
        ceiling_robot_cb = ctk.CTkCheckBox(self.challenges_frame,text="ceiling_robot",variable=self.robot_malfunction_info["ceiling_robot"], onvalue="1", offvalue="0", height=1, width=20)
        self.pack_and_append_challenge_widget(ceiling_robot_cb)

        self.show_challenges_condition_menu()

        self.save_challenge_button.configure(text="Save robot malfunction challenge", command=partial(self.save_challenge, "robot_malfunction"))
        self.save_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.save_challenge_button)
        self.cancel_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
    
    def reset_sensor_blackout_info(self):
        self.sensor_blackout_info["duration"].set('0.0')
        for variable in self.sensor_blackout_info["sensors_to_disable"]:
            variable.set('0')

    def add_sensor_blackout_challenge(self):
        self.clear_challenges_menu()

        duration_label = ctk.CTkLabel(self.challenges_frame, text="Enter the duration for the sensor blackout")
        self.pack_and_append_challenge_widget(duration_label)
        duration_menu = ctk.CTkEntry(self.challenges_frame, textvariable=self.sensor_blackout_info["duration"])
        self.pack_and_append_challenge_widget(duration_menu)

        sensors_label = ctk.CTkLabel(self.challenges_frame, text="Select the sensors for the sensor blackout")
        self.pack_and_append_challenge_widget(sensors_label)
        for i in range(len(self.sensor_blackout_info["sensors_to_disable"])):
            sensor_cb = ctk.CTkCheckBox(self.challenges_frame,text=SENSORS[i],variable=self.sensor_blackout_info["sensors_to_disable"][i], onvalue="1", offvalue="0", height=1, width=20)
            self.pack_and_append_challenge_widget(sensor_cb)
        
        self.show_challenges_condition_menu()

        self.save_challenge_button.configure(text="Save sensor blackout challenge", command=partial(self.save_challenge, "sensor_blackout"))
        self.save_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.save_challenge_button)
        self.cancel_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.cancel_challenge_button)
    
    def reset_faulty_part_info(self):
        self.faulty_part_info["order_id"].set("" if len(self.used_ids) == 0 else self.used_ids[0])
        for variable in self.faulty_part_info["quadrants"]:
            variable.set('0')

    def add_faulty_part_challenge(self):
        self.clear_challenges_menu()

        submission_id_label = ctk.CTkLabel(self.challenges_frame, text="Select the submission id for the faulty part order")
        self.pack_and_append_challenge_widget(submission_id_label)
        submission_id_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.faulty_part_info["order_id"],values=self.used_ids)
        self.pack_and_append_challenge_widget(submission_id_menu)

        quadrants_label = ctk.CTkLabel(self.challenges_frame, text="Select the quadrants for the faulty part challenge")
        self.pack_and_append_challenge_widget(quadrants_label)
        for i in range(len(QUADRANTS)):
            quadrants_cb = ctk.CTkCheckBox(self.challenges_frame,text=f"Quadrant {QUADRANTS[i]}",variable=self.faulty_part_info["quadrants"][i], onvalue="1", offvalue="0", height=1, width=20)
            self.pack_and_append_challenge_widget(quadrants_cb)

        self.save_challenge_button.configure(text="Save faulty part challenge", command=partial(self.save_challenge, "faulty_part"))
        self.save_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.save_challenge_button)
        self.cancel_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.cancel_challenge_button)

    def reset_human_info(self):
        self.human_info["behavior"].set(BEHAVIORS[0])
    
    def add_human_challenge(self):
        self.clear_challenges_menu()
        
        behavior_label = ctk.CTkLabel(self.challenges_frame, text="Select the behavior for the human")
        self.pack_and_append_challenge_widget(behavior_label)
        behavior_menu = ctk.CTkOptionMenu(self.challenges_frame, variable=self.human_info["behavior"],values=BEHAVIORS)
        self.pack_and_append_challenge_widget(behavior_menu)

        self.show_challenges_condition_menu()

        self.save_challenge_button.configure(text="Save human challenge", command=partial(self.save_challenge, "human"))
        self.save_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.save_challenge_button)
        self.cancel_challenge_button.pack(side=tk.BOTTOM)
        self.current_challenges_widgets.append(self.cancel_challenge_button)

    def save_challenge(self, type_of_challenge:str):
        print(type_of_challenge)
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


    # =======================================================
    #               General Gui Functions
    # =======================================================
    def nearest_slider_value(self,value,slider,label,_,__,___):
        newvalue = min(SLIDER_VALUES, key=lambda x:abs(x-float(value.get())))
        slider.set(newvalue)
        label.configure(text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(newvalue)]}")


if __name__=="__main__":
    app = GUI_CLASS()
    app.mainloop()