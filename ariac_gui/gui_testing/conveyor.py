import customtkinter as ctk
from customtkinter import *
from tkinter import *
import tkinter as tk
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi

from numpy import number

PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
current_parts = []
current_canvas_elements = []
COLOR_TYPE=[color+pType for color in PART_COLORS for pType in PART_TYPES]
MENU_IMAGES = {part_label:ctk.CTkImage(Image.open(os.getcwd()+f"/ariac_gui/resource/{part_label}.png"),size=(75,75)) for part_label in COLOR_TYPE}
CONVEYOR_ORDERS = ["random", "sequential"]
SLIDER_VALUES = [-pi,-3*pi/4,-pi/2,-pi/4,0,pi/4,pi/2,3*pi/4,pi]
SLIDER_STR = ["-pi","-3pi/4","-pi/2","-pi/4","0","pi/4","pi/2","3pi/4","pi"]

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
    spawn_rate_label = ctk.CTkLabel(main_wind,text=f"Current spawn rate: {conveyor_setup_vals['spawn_rate'].get()}")
    spawn_rate_label.pack()
    spawn_rate_slider = ctk.CTkSlider(main_wind, state=tk.DISABLED,variable=conveyor_setup_vals["spawn_rate"],from_=1, to=10, number_of_steps=9, orientation="horizontal")
    spawn_rate_slider.pack()
    present_widgets.append(spawn_rate_slider)
    conveyor_order_label = ctk.CTkLabel(main_wind,text=f"Select the conveyor order:")
    conveyor_order_label.pack()
    conveyor_order_menu = ctk.CTkOptionMenu(main_wind, variable=conveyor_setup_vals["order"],values=CONVEYOR_ORDERS,state=tk.DISABLED)
    conveyor_order_menu.pack()
    present_widgets.append(conveyor_order_menu)
    add_parts_button = ctk.CTkButton(main_wind,text="Add parts", command=add_parts, state=tk.DISABLED)
    add_parts_button.pack(pady=10)
    present_widgets.append(add_parts_button)
    conveyor_setup_vals["spawn_rate"].trace('w',partial(update_spawn_rate_slider,conveyor_setup_vals["spawn_rate"],spawn_rate_label))
    has_parts.trace('w', partial(activate_deactivate_menu, has_parts,present_widgets,conveyor_setup_vals))
    main_wind.mainloop()

def update_spawn_rate_slider(value : ctk.IntVar, label : ctk.CTkLabel,_,__,___):
    label.configure(text=f"Current spawn rate: {value.get()}")

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

def add_parts():
    print("TEST")
if __name__=="__main__":
    main()