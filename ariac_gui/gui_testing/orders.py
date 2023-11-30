import customtkinter as ctk
from customtkinter import *
from tkinter import *
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi
import random
import string

ORDER_TYPES=["kitting", "assembly", "combined"]
QUADRANTS=["1","2","3","4"]
AGV_OPTIONS=["1","2","3","4"]
KITTING_DESTINATIONS=["warehouse", "assembly_front","assembly_back","kitting"]
ASSEMBLY_STATIONS=["as1","as2","as3","as4"]
PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
CONDITION_TYPE=['time','part_place']
TRAY_IDS=[str(i) for i in range(10)]
COLOR_TYPE=["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]
MENU_IMAGES = {part_label:Image.open(os.getcwd()+f"/ariac_gui/resource/{part_label}.png") for part_label in ["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]}
SLIDER_VALUES = [-pi,-4*pi/5,-3*pi/4,-3*pi/5,-pi/2,-2*pi/5,-pi/4,-pi/5,0,pi/5,pi/4,2*pi/5,pi/2,3*pi/5,3*pi/4,4*pi/5,pi]
SLIDER_STR = ["-pi","-4pi/5","-3pi/4","-pi/2","-2pi/5","-pi/4","-pi/5","0","pi/5","pi/4","2pi/5","pi/2","3pi/5","3pi/4","4pi/5","pi"]

LEFT_COLUMN = 1
MIDDLE_COLUMN = 2
RIGHT_COLUMN = 3

class order_gui(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
        ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

        self.title("Orders Testing Window")
        
        self.geometry("700x500 + 300 + 300")
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(100, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(4, weight=1)

        # Order info
        self.current_left_widgets = []
        self.current_right_widgets = []
        self.current_main_widgets = []

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
        self.order_info["kitting_task"]["destination"] = ctk.StringVar()
        self.order_info["kitting_task"]["parts"] = []

        self.order_info["assembly_task"] = {}
        self.order_info["assembly_task"]["agv_number"] = ctk.StringVar()
        self.order_info["assembly_task"]["station"] = ctk.StringVar()
        self.order_info["assembly_task"]["parts"] = []

        self.order_info["combined_task"] = {}
        self.order_info["combined_task"]["station"] = ctk.StringVar()
        self.order_info["combined_task"]["parts"] = []

        # Row indeces
        self.left_row_index = 1
        self.right_row_index = 3
        
        # Main menu
        self.add_kitting_order_button = ctk.CTkButton(self, text="Add kitting order", command=self.add_kitting_order)
        self.add_kitting_order_button.grid(column=MIDDLE_COLUMN, row=1)
        self.add_assembly_order_button = ctk.CTkButton(self, text="Add assembly order", command=self.add_assembly_order)
        self.add_assembly_order_button.grid(column=MIDDLE_COLUMN, row=2)
        self.add_combined_order_button = ctk.CTkButton(self, text="Add combined order", command=self.add_combined_order)
        self.add_combined_order_button.grid(column=MIDDLE_COLUMN, row=3)

        self.current_main_widgets.append(self.add_kitting_order_button)
        self.current_main_widgets.append(self.add_assembly_order_button)
        self.current_main_widgets.append(self.add_combined_order_button)

        self.save_order_button = ctk.CTkButton(self,text="Save_order", command=self.save_order)
        self.cancel_order_button = ctk.CTkButton(self,text="Cancel order", command=self.cancel_order)

        # Trace functions
        self.order_info["announcement_type"].trace('w', self.show_correct_announcement_menu)


    def add_kitting_order(self):
        self.clear_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("kitting")
        self.show_kitting_menu()
        self.add_order_main_widgets()
    
    def add_assembly_order(self):
        self.clear_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("assembly")
        self.show_assembly_menu()
        self.add_order_main_widgets()

    def add_combined_order(self):
        self.clear_menu()

        self.left_row_index = 1
        self.order_info["order_type"].set("combined")
        self.show_combined_menu()
        self.add_order_main_widgets()

    def add_order_main_widgets(self):
        self.priority_cb = ctk.CTkCheckBox(self,text="Priority",variable=self.order_info["priority"], onvalue="1", offvalue="0", height=1, width=20)
        self.priority_cb.grid(column=MIDDLE_COLUMN, row=1)
        self.current_main_widgets.append(self.priority_cb)
        self.save_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+1)
        self.current_main_widgets.append(self.save_order_button)
        self.cancel_order_button.grid(column = MIDDLE_COLUMN, row=max(self.left_row_index,self.right_row_index)+2)
        self.current_main_widgets.append(self.cancel_order_button)

        self.announcement_type_label = ctk.CTkLabel(self, text = "Select the type of announcement:")
        self.announcement_type_label.grid(column = RIGHT_COLUMN , row = 1)
        self.current_main_widgets.append(self.announcement_type_label)
        self.announcement_type_menu = ctk.CTkOptionMenu(self, variable=self.order_info["announcement_type"],values=CONDITION_TYPE)
        self.announcement_type_menu.grid(column = RIGHT_COLUMN, row = 2)
        self.current_main_widgets.append(self.announcement_type_menu)

        self.reset_order()
    
    def grid_left_column(self, widget):
        widget.grid(column = LEFT_COLUMN, row = self.left_row_index)
        self.left_row_index+=1
        
    def grid_right_column(self, widget):
        widget.grid(column = RIGHT_COLUMN, row = self.right_row_index)
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
        self.order_info["kitting_task"]["destination"].set(KITTING_DESTINATIONS[0])
        self.order_info["kitting_task"]["parts"] = []

        self.order_info["assembly_task"]["agv_number"].set(AGV_OPTIONS[0])
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
        for widget in self.current_right_widgets:
            widget.grid_forget()
        self.current_right_widgets.clear()

        time_label = ctk.CTkLabel(self,text="Enter the time for the announcement:")
        self.grid_right_column(time_label)
        self.current_right_widgets.append(time_label)

        time_entry = ctk.CTkEntry(self, textvariable=self.order_info["announcement"]["time_condition"])
        self.grid_right_column(time_entry)
        self.current_right_widgets.append(time_entry)
    
    def show_part_place_announcement_menu(self):
        for widget in self.current_right_widgets:
            widget.grid_forget()

        self.current_right_widgets.clear()
        color_menu = ctk.CTkOptionMenu(self, variable=self.order_info["announcement"]["color"],values=PART_COLORS)
        self.grid_right_column(color_menu)
        self.current_right_widgets.append(color_menu)
        type_menu = ctk.CTkOptionMenu(self, variable=self.order_info["announcement"]["type"],values=PART_TYPES)
        self.grid_right_column(type_menu)
        self.current_right_widgets.append(type_menu)
        quadrant_menu = ctk.CTkOptionMenu(self, variable=self.order_info["announcement"]["agv"], values=AGV_OPTIONS)
        self.grid_right_column(quadrant_menu)
        self.current_right_widgets.append(quadrant_menu)
    
    def show_submission_announcement_menu(self):
        for widget in self.current_right_widgets:
            widget.grid_forget()
            
        self.current_right_widgets.clear()
        id_menu = ctk.CTkOptionMenu(self, variable=self.order_info["announcement"]["submission_id"],values=self.used_ids)
        self.grid_right_column(id_menu)
        self.current_right_widgets.append(id_menu)

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
        for widget in self.current_left_widgets:
            widget.grid_forget()
        self.current_left_widgets.clear()
        
        agv_number_label = ctk.CTkLabel(self,text="Select the agv for the kitting order")
        self.grid_left_column(agv_number_label)
        self.current_left_widgets.append(agv_number_label)

        agv_number_menu = ctk.CTkOptionMenu(self,variable=self.order_info["kitting_task"]["agv_number"], values = AGV_OPTIONS)
        self.grid_left_column(agv_number_menu)
        self.current_left_widgets.append(agv_number_menu)

        tray_id_label = ctk.CTkLabel(self,text="Select the tray for the kitting order")
        self.grid_left_column(tray_id_label)
        self.current_left_widgets.append(tray_id_label)

        tray_id_menu = ctk.CTkOptionMenu(self,variable=self.order_info["kitting_task"]["tray_id"], values = TRAY_IDS)
        self.grid_left_column(tray_id_menu)
        self.current_left_widgets.append(tray_id_menu)

        desination_label = ctk.CTkLabel(self,text="Select the destination for the kitting order")
        self.grid_left_column(desination_label)
        self.current_left_widgets.append(desination_label)

        desination_menu = ctk.CTkOptionMenu(self,variable=self.order_info["kitting_task"]["destination"],values = KITTING_DESTINATIONS)
        self.grid_left_column(desination_menu)
        self.current_left_widgets.append(desination_menu)

        add_part_kitting_task = ctk.CTkButton(self, text="Add part", command=self.add_kitting_part)
        self.grid_left_column(add_part_kitting_task)
        self.current_left_widgets.append(add_part_kitting_task)

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
        self.order_info["kitting_task"]["parts"].append(k_part_dict["color"].get()+k_part_dict["pType"].get())
        window.destroy()
    
    def show_assembly_menu(self):
        for widget in self.current_left_widgets:
            widget.grid_forget()
        self.current_left_widgets.clear()

        agv_number_label = ctk.CTkLabel(self,text="Select the agv for the assembly order")
        self.grid_left_column(agv_number_label)
        self.current_left_widgets.append(agv_number_label)

        agv_number_menu = ctk.CTkOptionMenu(self,variable=self.order_info["assembly_task"]["agv_number"], values = AGV_OPTIONS)
        self.grid_left_column(agv_number_menu)
        self.current_left_widgets.append(agv_number_menu)

        station_label = ctk.CTkLabel(self,text="Select the assembly station for the assembly order")
        self.grid_left_column(station_label)
        self.current_left_widgets.append(station_label)

        station_menu = ctk.CTkOptionMenu(self,variable=self.order_info["assembly_task"]["station"], values = ASSEMBLY_STATIONS)
        self.grid_left_column(station_menu)
        self.current_left_widgets.append(station_menu)

        add_part_assembly_task = ctk.CTkButton(self, text="Add part", command=self.add_assembly_part)
        self.grid_left_column(add_part_assembly_task)
        self.current_left_widgets.append(add_part_assembly_task)

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
        self.order_info["assembly_task"]["parts"].append(a_part_dict["color"].get()+a_part_dict["pType"].get())
        window.destroy()
    
    def show_combined_menu(self):
        for widget in self.current_left_widgets:
            widget.grid_forget()
        self.current_left_widgets.clear()
        
        station_label = ctk.CTkLabel(self,text="Select the assembly station for the combined order")
        self.grid_left_column(station_label)
        self.current_left_widgets.append(station_label)

        station_menu = ctk.CTkOptionMenu(self,variable=self.order_info["combined_task"]["station"], values = ASSEMBLY_STATIONS)
        self.grid_left_column(station_menu)
        self.current_left_widgets.append(station_menu)

        add_part_combined_task = ctk.CTkButton(self, text="Add part", command=self.add_combined_part)
        self.grid_left_column(add_part_combined_task)
        self.current_left_widgets.append(add_part_combined_task)

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
        self.order_info["combined_task"]["parts"].append(c_part_dict["color"].get()+c_part_dict["pType"].get())
        window.destroy()

    def clear_menu(self):
        for widget_list in [self.current_left_widgets,self.current_right_widgets, self.current_main_widgets]:
            for widget in widget_list:
                widget.grid_forget()
            widget_list.clear()

    def save_order(self):
        self.clear_menu()
        self.add_kitting_order_button.grid(column=MIDDLE_COLUMN, row=1)
        self.add_assembly_order_button.grid(column=MIDDLE_COLUMN, row=2)
        self.add_combined_order_button.grid(column=MIDDLE_COLUMN, row=3)
        self.current_main_widgets.append(self.add_kitting_order_button)
        self.current_main_widgets.append(self.add_assembly_order_button)
        self.current_main_widgets.append(self.add_combined_order_button)
        self.used_ids.append(self.generate_order_id())
        self.order_counter.set(str(len(self.used_ids)))
        if 'submission' not in CONDITION_TYPE:
            CONDITION_TYPE.append('submission')
    
    def cancel_order(self):
        self.clear_menu()
        self.add_kitting_order_button.grid(column=MIDDLE_COLUMN, row=1)
        self.add_assembly_order_button.grid(column=MIDDLE_COLUMN, row=2)
        self.add_combined_order_button.grid(column=MIDDLE_COLUMN, row=3)
        self.current_main_widgets.append(self.add_kitting_order_button)
        self.current_main_widgets.append(self.add_assembly_order_button)
        self.current_main_widgets.append(self.add_combined_order_button)

if __name__=="__main__":
    app = order_gui()
    app.mainloop()