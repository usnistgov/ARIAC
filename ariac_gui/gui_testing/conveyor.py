try:
    import customtkinter as ctk
    from customtkinter import *
except:
    print("ERROR: customtkinter not installed")
    quit()    
from tkinter import *
import tkinter as tk
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi

from numpy import number

PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
current_parts = []
conveyor_parts = []
current_canvas_elements = []
COLOR_TYPE=[color+pType for color in PART_COLORS for pType in PART_TYPES]
MENU_IMAGES = {part_label:ctk.CTkImage(Image.open(os.getcwd()+f"/ariac_gui/resource/{part_label}.png"),size=(50,50)) for part_label in COLOR_TYPE}
CONVEYOR_ORDERS = ["random", "sequential"]
SLIDER_VALUES = [-pi,-3*pi/4,-pi/2,-pi/4,0,pi/4,pi/2,3*pi/4,pi]
SLIDER_STR = ["-pi","-3pi/4","-pi/2","-pi/4","0","pi/4","pi/2","3pi/4","pi"]

class ConveyorPart():
    def __init__(self,color, pType, num_parts, offset, rotation):
        self.color = color
        self.pType = pType
        self.num_parts = num_parts
        self.rotation = rotation
        self.offset = offset

def main():
    main_wind = ctk.CTk()
    total_part_counter = ctk.StringVar()
    total_part_counter.set('0')
    ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
    main_wind.geometry("400x400 + 300 + 300")
    has_parts = ctk.StringVar()
    has_parts.set("0")
    present_widgets = []
    trial_has_parts_cb = ctk.CTkCheckBox(main_wind,text="Trial has conveyor parts",variable=has_parts, onvalue="1", offvalue="0", height=1, width=20)
    trial_has_parts_cb.pack(pady=5)
    conveyor_setup_vals = {"active":ctk.StringVar(),"spawn_rate":ctk.IntVar(),"order":ctk.StringVar()}
    conveyor_setup_vals["active"].set('1')
    conveyor_setup_vals['spawn_rate'].set(1)
    conveyor_setup_vals["order"].set(CONVEYOR_ORDERS[0])
    conveyor_active_cb = ctk.CTkCheckBox(main_wind,text="Conveyor active",variable=conveyor_setup_vals["active"], onvalue="1", offvalue="0", height=1, width=20, state=tk.DISABLED)
    conveyor_active_cb.pack(pady=5)
    present_widgets.append(conveyor_active_cb)
    spawn_rate_label = ctk.CTkLabel(main_wind,text=f"current spawn rate (seconds): {conveyor_setup_vals['spawn_rate'].get()}")
    spawn_rate_label.pack()
    spawn_rate_slider = ctk.CTkSlider(main_wind, state=tk.DISABLED,variable=conveyor_setup_vals["spawn_rate"],from_=1, to=10, number_of_steps=9, orientation="horizontal")
    spawn_rate_slider.pack()
    present_widgets.append(spawn_rate_slider)
    conveyor_order_label = ctk.CTkLabel(main_wind,text=f"Select the conveyor order:")
    conveyor_order_label.pack()
    conveyor_order_menu = ctk.CTkOptionMenu(main_wind, variable=conveyor_setup_vals["order"],values=CONVEYOR_ORDERS,state=tk.DISABLED)
    conveyor_order_menu.pack()
    present_widgets.append(conveyor_order_menu)
    add_parts_button = ctk.CTkButton(main_wind,text="Add parts", command=partial(add_parts, total_part_counter), state=tk.DISABLED)
    add_parts_button.pack(pady=10)
    current_parts_label = ctk.CTkLabel(main_wind, text="Current parts on conveyor belt:")
    current_parts_label.pack()
    canvas = Canvas(main_wind)
    canvas.pack(fill = BOTH, expand = 1)
    present_widgets.append(add_parts_button)
    conveyor_setup_vals["spawn_rate"].trace('w',partial(update_spawn_rate_slider,conveyor_setup_vals["spawn_rate"],spawn_rate_label))
    has_parts.trace('w', partial(activate_deactivate_menu, has_parts,present_widgets,conveyor_setup_vals))
    total_part_counter.trace('w',partial(show_current_parts,canvas, main_wind))
    main_wind.mainloop()

def show_current_parts(canvas : tk.Canvas, main_wind : ctk.CTk,_,__,___):
    for e in current_canvas_elements:
        canvas.delete(e)
    current_canvas_elements.clear()
    image_coordinates = [(25,10+(50*i)) for i in range(10)]
    num_parts_coordinates = [(65,10+(50*i)) for i in range(10)]
    part_count = {color_type:0 for color_type in COLOR_TYPE}
    image_labels = []
    num_parts_labels = []
    for part in conveyor_parts:
        part_count[part.color+part.pType]+=part.num_parts
    for part in list(set(current_parts)):
        image_labels.append(ctk.CTkLabel(main_wind,text="",image=MENU_IMAGES[part]))
        num_parts_labels.append(ctk.CTkLabel(main_wind,text=f"X {part_count[part]}"))
    for i in range(len(image_labels)):
        current_canvas_elements.append(canvas.create_window(image_coordinates[i], window = image_labels[i]))
        current_canvas_elements.append(canvas.create_window(num_parts_coordinates[i], window = num_parts_labels[i]))

def update_spawn_rate_slider(value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
    label.configure(text=f"current spawn rate (seconds): {value.get()}")

def activate_deactivate_menu(has_parts:ctk.StringVar,widgets:list,conveyor_setup_vals,_,__,___):
    if has_parts.get()=="1":
        for widget in widgets:
            widget.configure(state=tk.NORMAL)
    else:
        for widget in widgets:
            widget.configure(state=tk.DISABLED)
        conveyor_setup_vals["active"].set('1')
        conveyor_setup_vals['spawn_rate'].set(1)
        conveyor_setup_vals["order"].set(CONVEYOR_ORDERS[0])

def add_parts(total_part_counter):
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
    conveyor_part_vals["num_parts"].trace('w', partial(update_offset_slider, conveyor_part_vals["num_parts"], num_parts_label))
    offset_label = ctk.CTkLabel(add_parts_conveyor_window,text=f"Current offset: {conveyor_part_vals['offset'].get()}")
    offset_label.pack()
    offset_slider = ctk.CTkSlider(add_parts_conveyor_window,variable=conveyor_part_vals["offset"],from_=-1, to=1, number_of_steps=40, orientation="horizontal")
    offset_slider.pack()
    conveyor_part_vals["offset"].trace('w', partial(update_offset_slider, conveyor_part_vals["offset"], offset_label))
    rotation_label = ctk.CTkLabel(add_parts_conveyor_window, text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(conveyor_part_vals['rotation'].get())]}")
    rotation_label.pack()
    rotation_slider = ctk.CTkSlider(add_parts_conveyor_window, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=conveyor_part_vals["rotation"], orientation="horizontal")
    rotation_slider.pack(pady=5)
    conveyor_part_vals["rotation"].trace('w', partial(nearest_slider_value, conveyor_part_vals["rotation"], rotation_slider,rotation_label))
    back_button = ctk.CTkButton(add_parts_conveyor_window,text="Back",command=add_parts_conveyor_window.destroy)
    back_button.pack()
    save_button = ctk.CTkButton(add_parts_conveyor_window,text="Save part",command=partial(save_parts,total_part_counter,add_parts_conveyor_window,conveyor_part_vals))
    save_button.pack()

def update_num_parts_slider(value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
    label.configure(text=f"Current number of parts: {value.get()}")

def update_offset_slider(value : ctk.DoubleVar, label : ctk.CTkLabel,_,__,___):
    value.set(round(value.get(),3))
    label.configure(text=f"Current offset: {value.get()}")

def nearest_slider_value(value,slider,label,_,__,___):
    newvalue = min(SLIDER_VALUES, key=lambda x:abs(x-float(value.get())))
    slider.set(newvalue)
    label.configure(text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(newvalue)]}")

def save_parts(total_part_counter:ctk.StringVar, window:ctk.CTkToplevel, conveyor_part_vals):
    color = conveyor_part_vals["color"].get()
    pType = conveyor_part_vals["pType"].get()
    conveyor_parts.append(ConveyorPart(color, pType,conveyor_part_vals["num_parts"].get(),conveyor_part_vals["offset"].get(), conveyor_part_vals["rotation"].get()))
    current_parts.append(color+pType)
    total_part_counter.set(str(len(current_parts)))
    window.destroy()

if __name__=="__main__":
    main()