import customtkinter as ctk
from customtkinter import *
from tkinter import *
from functools import partial
from PIL import Image  # needed for images in gui
from math import pi

class BinPart():
    def __init__(self,color = "", pType = "", rotation = "", flipped = ""):
        self.color = color
        self.pType = pType
        self.rotation = rotation
        self.flipped = flipped

PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
ALL_BINS=['bin'+str(i) for i in range(1,9)]
current_parts = {f"bin{i}":["" for _ in range(9)] for i in range(1,9)}
bin_parts = {f"bin{i}":[BinPart() for _ in range(9)] for i in range(1,9)}
current_canvas_elements = []
COLOR_TYPE=["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]
MENU_IMAGES = {part_label:Image.open(os.getcwd()+f"/ariac_gui/resource/{part_label}.png") for part_label in ["plus"]+[color+pType for color in PART_COLORS for pType in PART_TYPES]}
SLIDER_VALUES = [-pi,-3*pi/4,-pi/2,-pi/4,0,pi/4,pi/2,3*pi/4,pi]
SLIDER_STR = ["-pi","-3pi/4","-pi/2","-pi/4","0","pi/4","pi/2","3pi/4","pi"]

def main():
    main_wind = ctk.CTk()
    total_part_counter = ctk.StringVar()
    total_part_counter.set('0')
    ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
    main_wind.geometry("400x400 + 300 + 300")
    bin_selection = ctk.StringVar()
    bin_selection.set(ALL_BINS[0])
    bin_label = ctk.CTkLabel(main_wind,text="Select the bin you would like to add parts to:")
    bin_label.pack()
    bin_menu = ctk.CTkOptionMenu(main_wind,
                                    variable=bin_selection,
                                    values=ALL_BINS,
                                    fg_color = "#e2e2e2",
                                    text_color="black",
                                    button_color="#d3d3d3",
                                    button_hover_color="#9e9e9e",
                                    anchor='center',
                                    )
    bin_menu.pack()

    canvas = Canvas(main_wind)
    
    canvas.create_rectangle(50, 10, 350, 310, 
                            outline = "black", fill = "#60c6f1",
                            width = 2)
    show_grid(bin_selection,canvas,main_wind, total_part_counter)
    canvas.pack(fill = BOTH, expand = 1)
    add_multiple_parts_button = ctk.CTkButton(main_wind,text="Add multiple parts",command=partial(add_multiple_parts,bin_selection.get(),total_part_counter))
    add_multiple_parts_button.pack()
    bin_selection.trace('w',partial(update_grid, bin_selection,canvas,main_wind,total_part_counter))
    total_part_counter.trace('w',partial(update_grid, bin_selection,canvas,main_wind,total_part_counter))
    main_wind.mainloop()

def show_grid(bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk, total_part_counter):
    button_coordinates = [(100,60),(200,60),(300,60),
                        (100,160),(200,160),(300,160),
                        (100,260),(200,260),(300,260)]
    current_bin_slot_widgets = []
    for i in range(len(button_coordinates)):
        if current_parts[bin_selection.get()][i]=="":
            current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(add_part, bin_selection.get(), i,total_part_counter),
                                                          image=ctk.CTkImage(MENU_IMAGES["plus"],size=(75,75)),
                                                          fg_color="transparent",bg_color="#4FA2C6",hover_color="#458DAC",width=1))
        elif bin_parts[bin_selection.get()][i].flipped == "0":
            current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(add_part, bin_selection.get(), i,total_part_counter),
                                                          image=ctk.CTkImage(MENU_IMAGES[current_parts[bin_selection.get()][i]].rotate(bin_parts[bin_selection.get()][i].rotation*180/pi),size=(75,75)),
                                                          fg_color="transparent",bg_color="#60c6f1",hover_color="#60c6f1",width=1))
        else:
            current_bin_slot_widgets.append(ctk.CTkButton(main_wind,text=f"",command=partial(add_part, bin_selection.get(), i,total_part_counter),
                                                          image=ctk.CTkImage(MENU_IMAGES[current_parts[bin_selection.get()][i]].rotate(bin_parts[bin_selection.get()][i].rotation*180/pi).transpose(Image.FLIP_LEFT_RIGHT),size=(75,75)),
                                                          fg_color="transparent",bg_color="#60c6f1",hover_color="#60c6f1",width=1))

    for i in range(len(current_bin_slot_widgets)):
        current_canvas_elements.append(canvas.create_window(button_coordinates[i], window = current_bin_slot_widgets[i]))

def update_grid(bin_selection : ctk.StringVar,canvas:Canvas, main_wind : ctk.CTk,total_part_counter,_,__,___):
    for i in current_canvas_elements:
        canvas.delete(i)
    current_canvas_elements.clear()
    show_grid(bin_selection,canvas,main_wind,total_part_counter)

def add_part(bin, index,total_part_counter:ctk.StringVar):
    bin_vals = {}
    add_part_bin_window = ctk.CTkToplevel()
    add_part_bin_window.geometry("400x400 + 700 + 300")
    bin_vals["color"] = ctk.StringVar()
    
    bin_vals["pType"] = ctk.StringVar()
    
    bin_vals["rotation"] = ctk.DoubleVar()
    
    bin_vals["flipped"] = ctk.StringVar()
    if current_parts[bin][index] == "":
        bin_vals["color"].set(PART_COLORS[0])
        bin_vals["pType"].set(PART_TYPES[0])
        bin_vals["rotation"].set(0.0)
        bin_vals["flipped"].set("0")
    else:
        bin_vals["color"].set(bin_parts[bin][index].color)
        bin_vals["pType"].set(bin_parts[bin][index].pType)
        bin_vals["rotation"].set(bin_parts[bin][index].rotation)
        bin_vals["flipped"].set(bin_parts[bin][index].flipped)
    color_menu = ctk.CTkOptionMenu(add_part_bin_window, variable=bin_vals["color"],values=PART_COLORS)
    color_menu.pack()
    type_menu = ctk.CTkOptionMenu(add_part_bin_window, variable=bin_vals["pType"],values=PART_TYPES)
    type_menu.pack()
    rotation_label = ctk.CTkLabel(add_part_bin_window, text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(bin_vals['rotation'].get())]}")
    rotation_label.pack()
    rotation_slider = ctk.CTkSlider(add_part_bin_window, from_=min(SLIDER_VALUES), to=max(SLIDER_VALUES),variable=bin_vals["rotation"], orientation="horizontal")
    rotation_slider.pack(pady=5)
    bin_vals["rotation"].trace('w', partial(nearest_slider_value, bin_vals["rotation"], rotation_slider,rotation_label))
    flipped_cb = ctk.CTkCheckBox(add_part_bin_window,text="Flipped",variable=bin_vals["flipped"], onvalue="1", offvalue="0", height=1, width=20)
    flipped_cb.pack(pady=5)
    back_button = ctk.CTkButton(add_part_bin_window,text="Back",command=add_part_bin_window.destroy)
    back_button.pack()
    save_button = ctk.CTkButton(add_part_bin_window,text="Save part",command=partial(save_part,bin,index,total_part_counter,add_part_bin_window,bin_vals))
    save_button.pack()

def save_part(bin, index, total_part_counter:ctk.StringVar, window:ctk.CTkToplevel, bin_vals):
    color = bin_vals["color"].get()
    pType = bin_vals["pType"].get()
    current_parts[bin][index]=color+pType
    bin_parts[bin][index].color = color
    bin_parts[bin][index].pType = pType
    bin_parts[bin][index].rotation = bin_vals["rotation"].get() 
    bin_parts[bin][index].flipped = bin_vals["flipped"].get()
    total_part_counter.set(str(int(total_part_counter.get())+1))
    window.destroy()

def add_multiple_parts(bin,total_part_counter):
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
    bin_vals["rotation"].trace('w', partial(nearest_slider_value, bin_vals["rotation"], rotation_slider,rotation_label))
    flipped_cb = ctk.CTkCheckBox(add_parts_bin_window,text="Flipped",variable=bin_vals["flipped"], onvalue="1", offvalue="0", height=1, width=20)
    flipped_cb.pack(pady=5)
    back_button = ctk.CTkButton(add_parts_bin_window,text="Back",command=add_parts_bin_window.destroy)
    back_button.pack()
    save_button = ctk.CTkButton(add_parts_bin_window,text="Save part",command=partial(save_parts,bin,slot_values,total_part_counter,add_parts_bin_window,bin_vals))
    save_button.pack()

def save_parts(bin, slot_values, total_part_counter:ctk.StringVar, window:ctk.CTkToplevel, bin_vals):
    color = bin_vals["color"].get()
    pType = bin_vals["pType"].get()
    slot_indices = [int(val.get()) for val in slot_values if val.get()!="-1"]
    for index in slot_indices:
        current_parts[bin][index]=color+pType
        bin_parts[bin][index].color = color
        bin_parts[bin][index].pType = pType
        bin_parts[bin][index].rotation = bin_vals["rotation"].get() 
        bin_parts[bin][index].flipped = bin_vals["flipped"].get()
    total_part_counter.set(str(int(total_part_counter.get())+len(slot_indices)))
    window.destroy()

def nearest_slider_value(value,slider,label,_,__,___):
    newvalue = min(SLIDER_VALUES, key=lambda x:abs(x-float(value.get())))
    slider.set(newvalue)
    label.configure(text=f"Current rotation value: {SLIDER_STR[SLIDER_VALUES.index(newvalue)]}")



if __name__=="__main__":
    main()