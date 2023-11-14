import customtkinter as ctk
from customtkinter import *
from tkinter import *
from functools import partial

tray_options = [""]+[str(i) for i in range(10)]

def main():
    main_wind = ctk.CTk()
    ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
    tray_label = ctk.CTkLabel(main_wind,text="Select the tray ids for each slot")
    tray_label.pack()
    canvas = Canvas(main_wind)
    main_wind.geometry("400x400 + 300 + 300")
    canvas.create_rectangle(10, 10, 170, 310, 
                            outline = "black", fill = "#f6f6f6",
                            width = 2)
    canvas.create_rectangle(230, 10, 390, 310, 
                            outline = "black", fill = "#f6f6f6",
                            width = 2)
    menu_coordinates = [(95,60),(95,160),(95,260),(315,60),(315,160),(315,260)]
    label_coordinates = [(20,60),(20,160),(20,260),(240,60),(240,160),(240,260)]
    tray_selections = [ctk.StringVar() for _ in range(6)]
    for i in tray_selections:i.set(tray_options[0])
    tray_menus = [ctk.CTkOptionMenu(main_wind,
                                    variable=tray_selections[i],
                                    values=tray_options,
                                    fg_color = "#e2e2e2",
                                    text_color="black",
                                    button_color="#d3d3d3",
                                    button_hover_color="#9e9e9e",
                                    anchor='center') for i in range(6)]
    for i in range(6):
        canvas.create_window(label_coordinates[i], window=ctk.CTkLabel(main_wind, text=f"{i}:"))
        canvas.create_window(menu_coordinates[i], window = tray_menus[i])
    canvas.create_window((90,325),window=ctk.CTkLabel(main_wind,text="kts_1"))
    canvas.create_window((310,325),window=ctk.CTkLabel(main_wind,text="kts_2"))
    canvas.pack(fill = BOTH, expand = 1)
    update_all_tray_menus=partial(updateKTrayMenus,tray_menus,tray_selections)
    for tray in tray_selections:
        tray.trace('w',update_all_tray_menus)
    main_wind.mainloop()
    print(",".join([i.get() for i in tray_selections if i.get()!=""]))

def updateKTrayMenus(tray_menus,tray_selections,_,__,___):
    '''Updates the available trays for kitting trays'''
    tray_options_available = [[] for _ in range(6)]
    currentTrayVals=[tray.get() for tray in tray_selections]
    for tray in tray_options:
        for i in range(6):
            if (tray not in currentTrayVals or tray==tray_selections[i].get()):
                tray_options_available[i].append(tray)
    for i in range(len(tray_options_available)):
        if "" not in tray_options_available[i]:
            tray_options_available[i]=[""]+tray_options_available[i]
        tray_menus[i].configure(values=tray_options_available[i])

    
if __name__=="__main__":
    main()