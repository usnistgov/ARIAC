try:
    import customtkinter as ctk
    from customtkinter import *
except:
    print("ERROR: customtkinter not installed")
    quit()    
from tkinter import *
import tkinter as tk
from tkinter import ttk
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi


FRAMEWIDTH=700
FRAMEHEIGHT=900
LEFTCOLUMN=1

PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']

#Options for kitting trays
KITTING_TRAY_OPTIONS = [""]+[str(i) for i in range(10)]

# Bin menu items
ALL_BINS=['bin'+str(i) for i in range(1,9)]

# Conveyor order types
CONVEYOR_ORDERS = ["random", "sequential"]

# Menu images
MENU_IMAGES = {part_label:Image.open(os.getcwd()+f"/ariac_gui/resource/{part_label}.png") for part_label in ["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]}

# Values for the sliders
SLIDER_VALUES = [-pi,-3*pi/4,-pi/2,-pi/4,0,pi/4,pi/2,3*pi/4,pi]
SLIDER_STR = ["-pi","-3pi/4","-pi/2","-pi/4","0","pi/4","pi/2","3pi/4","pi"]

class BinPart():
    def __init__(self,color = "", pType = "", rotation = "", flipped = ""):
        self.color = color
        self.pType = pType
        self.rotation = rotation
        self.flipped = flipped

class ConveyorPart():
    def __init__(self,color, pType, num_parts, offset, rotation):
        self.color = color
        self.pType = pType
        self.num_parts = num_parts
        self.rotation = rotation
        self.offset = offset

class GUI_CLASS(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
        ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
        
        self.title("NIST ARIAC CONFIGURATION GUI")

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(pady=10,column=LEFTCOLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

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
        kitting_tray_canvas.create_rectangle(230, 10, 390, 310, 
                                outline = "black", fill = "#f6f6f6",
                                width = 2)
        menu_coordinates = [(95,60),(95,160),(95,260),(315,60),(315,160),(315,260)]
        label_coordinates = [(20,60),(20,160),(20,260),(240,60),(240,160),(240,260)]
        
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
        bin_selection.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))
        self.bin_parts_counter.trace('w',partial(self.update_bin_grid, bin_selection,bin_parts_canvas,self.bin_parts_frame))

    def show_grid(self,bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk):
        button_coordinates = [(100,60),(200,60),(300,60),
                            (100,160),(200,160),(300,160),
                            (100,260),(200,260),(300,260)]
        current_bin_slot_widgets = []
        for i in range(len(button_coordinates)):
            if self.current_bin_parts[bin_selection.get()][i]=="":
                current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(self.add_bin_part, bin_selection.get(), i),
                                                            image=ctk.CTkImage(MENU_IMAGES["plus"],size=(75,75)),
                                                            fg_color="transparent",bg_color="#4FA2C6",hover_color="#458DAC",width=1))
            elif self.bin_parts[bin_selection.get()][i].flipped == "0":
                current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(self.add_bin_part, bin_selection.get(), i),
                                                            image=ctk.CTkImage(MENU_IMAGES[self.current_bin_parts[bin_selection.get()][i]].rotate(self.bin_parts[bin_selection.get()][i].rotation*180/pi),size=(75,75)),
                                                            fg_color="transparent",bg_color="#60c6f1",hover_color="#60c6f1",width=1))
            else:
                current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(self.add_bin_part, bin_selection.get(), i),
                                                            image=ctk.CTkImage(MENU_IMAGES[self.current_bin_parts[bin_selection.get()][i]].rotate(self.bin_parts[bin_selection.get()][i].rotation*180/pi).transpose(Image.FLIP_LEFT_RIGHT),size=(75,75)),
                                                            fg_color="transparent",bg_color="#60c6f1",hover_color="#60c6f1",width=1))

        for i in range(len(current_bin_slot_widgets)):
            self.current_bin_canvas_elements.append(canvas.create_window(button_coordinates[i], window = current_bin_slot_widgets[i]))
    
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
            bin_vals["color"].set(self.bin_parts[bin][index].color)
            bin_vals["pType"].set(self.bin_parts[bin][index].pType)
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
        back_button = ctk.CTkButton(add_part_bin_window,text="Back",command=add_part_bin_window.destroy)
        back_button.pack()
        save_button = ctk.CTkButton(add_part_bin_window,text="Save part",command=partial(self.save_bin_part,bin,index,add_part_bin_window,bin_vals))
        save_button.pack()

    def save_bin_part(self,bin, index, window:ctk.CTkToplevel, bin_vals):
        color = bin_vals["color"].get()
        pType = bin_vals["pType"].get()
        self.current_bin_parts[bin][index]=color+pType
        self.bin_parts[bin][index].color = color
        self.bin_parts[bin][index].pType = pType
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
            self.bin_parts[bin][index].color = color
            self.bin_parts[bin][index].pType = pType
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
        conveyor_setup_vals = {"active":ctk.StringVar(),"spawn_rate":ctk.IntVar(),"order":ctk.StringVar()}
        conveyor_setup_vals["active"].set('1')
        conveyor_setup_vals['spawn_rate'].set(1)
        conveyor_setup_vals["order"].set(CONVEYOR_ORDERS[0])
        conveyor_active_cb = ctk.CTkCheckBox(self.conveyor_parts_frame,text="Conveyor active",variable=conveyor_setup_vals["active"], onvalue="1", offvalue="0", height=1, width=20, state=tk.DISABLED)
        conveyor_active_cb.pack(pady=5)
        self.present_conveyor_widgets.append(conveyor_active_cb)
        spawn_rate_label = ctk.CTkLabel(self.conveyor_parts_frame,text=f"current spawn rate (seconds): {conveyor_setup_vals['spawn_rate'].get()}")
        spawn_rate_label.pack()
        spawn_rate_slider = ctk.CTkSlider(self.conveyor_parts_frame, state=tk.DISABLED,variable=conveyor_setup_vals["spawn_rate"],from_=1, to=10, number_of_steps=9, orientation="horizontal")
        spawn_rate_slider.pack()
        self.present_conveyor_widgets.append(spawn_rate_slider)
        conveyor_order_label = ctk.CTkLabel(self.conveyor_parts_frame,text=f"Select the conveyor order:")
        conveyor_order_label.pack()
        conveyor_order_menu = ctk.CTkOptionMenu(self.conveyor_parts_frame, variable=conveyor_setup_vals["order"],values=CONVEYOR_ORDERS,state=tk.DISABLED)
        conveyor_order_menu.pack()
        self.present_conveyor_widgets.append(conveyor_order_menu)
        add_parts_button = ctk.CTkButton(self.conveyor_parts_frame,text="Add parts", command=partial(self.add_conveyor_parts), state=tk.DISABLED)
        add_parts_button.pack(pady=10)
        current_parts_label = ctk.CTkLabel(self.conveyor_parts_frame, text="Current parts on conveyor belt:")
        current_parts_label.pack()
        conveyor_canvas = Canvas(self.conveyor_parts_frame)
        conveyor_canvas.pack(fill = BOTH, expand = 1)
        self.present_conveyor_widgets.append(add_parts_button)
        conveyor_setup_vals["spawn_rate"].trace('w',partial(self.update_spawn_rate_slider,conveyor_setup_vals["spawn_rate"],spawn_rate_label))
        has_parts.trace('w', partial(self.activate_deactivate_menu, has_parts,conveyor_setup_vals))
        self.conveyor_parts_counter.trace('w',partial(self.show_current_parts,conveyor_canvas))
    
    def show_current_parts(self,canvas : tk.Canvas,_,__,___):
        for e in self.current_conveyor_canvas_elements:
            canvas.delete(e)
        self.current_conveyor_canvas_elements.clear()
        image_coordinates = [(25,10+(75*i)) for i in range(10)]
        num_parts_coordinates = [(65,10+(75*i)) for i in range(10)]
        image_labels = []
        num_parts_labels = []
        for i in range(len(self.current_conveyor_parts)):
            part = self.conveyor_parts[i].color+self.conveyor_parts[i].pType
            image_labels.append(ctk.CTkLabel(self.conveyor_parts_frame,text="",
            image=ctk.CTkImage(MENU_IMAGES[part].rotate(self.conveyor_parts[i].rotation*180/pi),size=(75,75))))
            num_parts_labels.append(ctk.CTkLabel(self.conveyor_parts_frame,text=f"X {self.conveyor_parts[i].num_parts}"))
        for i in range(len(image_labels)):
            self.current_conveyor_canvas_elements.append(canvas.create_window(image_coordinates[i], window = image_labels[i]))
            self.current_conveyor_canvas_elements.append(canvas.create_window(num_parts_coordinates[i], window = num_parts_labels[i]))
    
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
    
    def add_conveyor_parts(self):
        add_parts_conveyor_window = ctk.CTkToplevel()
        add_parts_conveyor_window.geometry("400x450 + 700 + 300")
        conveyor_part_vals = {}
        
        conveyor_part_vals["color"] = ctk.StringVar()
        conveyor_part_vals["color"].set(PART_COLORS[0])
        conveyor_part_vals["pType"] = ctk.StringVar()
        conveyor_part_vals["pType"].set(PART_TYPES[0])
        conveyor_part_vals["num_parts"] = ctk.IntVar()
        conveyor_part_vals["num_parts"].set(1)
        conveyor_part_vals["offset"] = ctk.DoubleVar()
        conveyor_part_vals["offset"].set(0.0)
        conveyor_part_vals["rotation"] = ctk.DoubleVar()
        conveyor_part_vals["rotation"].set(0.0)
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
        back_button = ctk.CTkButton(add_parts_conveyor_window,text="Back",command=add_parts_conveyor_window.destroy)
        back_button.pack()
        save_button = ctk.CTkButton(add_parts_conveyor_window,text="Save part",command=partial(self.save_conveyor_parts,add_parts_conveyor_window,conveyor_part_vals))
        save_button.pack()
    
    def update_num_parts_slider(self,value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
        label.configure(text=f"Current number of parts: {value.get()}")

    def update_offset_slider(self, value : ctk.DoubleVar, label : ctk.CTkLabel,_,__,___):
        value.set(round(value.get(),3))
        label.configure(text=f"Current offset: {value.get()}")
    
    def save_conveyor_parts(self, window:ctk.CTkToplevel, conveyor_part_vals):
        color = conveyor_part_vals["color"].get()
        pType = conveyor_part_vals["pType"].get()
        self.conveyor_parts.append(ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get()))
        self.current_conveyor_parts.append(color+pType)
        self.conveyor_parts_counter.set(str(len(self.current_conveyor_parts)))
        window.destroy()

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