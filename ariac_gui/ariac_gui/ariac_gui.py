import rclpy
from rclpy.node import Node
from tkinter.messagebox import showinfo
from tkinter import ttk
from tkinter import Menu, Frame, TOP, CENTER, LEFT
import tkinter as tk
from pprint import pprint
from copy import copy
from tkinter.messagebox import askyesno


setup_dict = {
    'time_limit': -1,
    'slots': {
        'slot1': None,
        'slot2': None,
        'slot3': '2',
        'slot4': None,
        'slot5': None,
        'slot6': None,
    }
}

part_location_agvs_dict = {
    'agv1': {'tray_id': 0,
             'parts': {
                 'part1': {
                     'type': 'sensor',
                     'color': 'green',
                     'quadrant': 3,
                     'rotation': 'pi',
                 },
                 'part2': {
                     'type': 'pump',
                     'color': 'green',
                     'quadrant': 1,
                     'rotation': '0',
                 },
                 'part3': {},
                 'part4': {},
             },
             },
    'agv2': {'tray_id': 0,
             'parts': {
                 'part1': {},
                 'part2': {},
                 'part3': {},
                 'part4': {},
             }},
    'agv3': {'tray_id': 0,
             'parts': {
                 'part1': {},
                 'part2': {},
                 'part3': {},
                 'part4': {},
             }},
    'agv4': {'tray_id': 0,
             'parts': {
                 'part1': {},
                 'part2': {},
                 'part3': {},
                 'part4': {},
             }},
}

part_locations = {

    'bin1': 'None',
    'bin2': 'None',
    'bin3': 'None',
    'bin4': 'None',
    'bin5': 'None',
    'bin6': 'None',
    'bin7': 'None',
    'bin8': 'None',
    'conveyor': 'None',
}


class AriacGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        # configure the root window
        self.title('ARIAC 2023 Trial Generator')
        # self.geometry('1000x700')
        self.resizable(width=False, height=False)
        self.grid_columnconfigure(0, weight=1)

        # Get the current screen width and height
        self.screen_width = self.winfo_screenwidth()
        self.screen_height = self.winfo_screenheight()

        self.geometry('1280x800')

        # self.geometry(
        #     f'{int(self.screen_width/1.5)}x{int(self.screen_height/1.5)}')

        # part types
        self.part_types = ['battery', 'pump', 'regulator', 'sensor']
        self.part_colors = ['red', 'green', 'blue', 'orange', 'purple']
        self.part_quadrants = [1, 2, 3, 4]
        self.part_rotations = ['0', 'pi', '-pi/4',
                               '-pi/3', '-pi/2', 'pi/4', 'pi/3', 'pi/2']
        self.tray_ids = ["Select", "0", "1",
                         "2", "3", "4", "5", "6", "7", "8", "9"]
        self.tray_slots = [1, 2, 3, 4, 5, 6]
        # flags
        self.part_location_selected = False

        # ------------------------
        # -- SETUP TAB
        # ------------------------
        self.time_limit_value = tk.StringVar()

        # slots
        self.slot1_cb = None
        self.slot1_value = tk.StringVar()

        self.slot2_cb = None
        self.slot2_value = tk.StringVar()

        self.slot3_cb = None
        self.slot3_value = tk.StringVar()

        self.slot4_cb = None
        self.slot4_value = tk.StringVar()

        self.slot5_cb = None
        self.slot5_value = tk.StringVar()

        self.slot6_cb = None
        self.slot6_value = tk.StringVar()

        self.tray_id_slot1 = copy(self.tray_ids)
        self.tray_id_slot2 = copy(self.tray_ids)
        self.tray_id_slot3 = copy(self.tray_ids)
        self.tray_id_slot4 = copy(self.tray_ids)
        self.tray_id_slot5 = copy(self.tray_ids)
        self.tray_id_slot6 = copy(self.tray_ids)

        # ------------------------
        # -- PART LOCATION TAB
        # ------------------------

        # Frames
        self.left_part_location_frame = None
        self.right_part_location_frame = None

        self.tab_control = ttk.Notebook(self)

        self.agv_selection_label = tk.StringVar()
        self.agv_selection_label.set('Please select an AGV')

        self.part_id_on_agv = tk.StringVar()
        # number of parts to spawn on the selected AGV: 1,2,3,4
        self.number_of_parts_on_agv = tk.StringVar()
        # text for label when the number of parts to spawn on the selected AGV is selected
        self.number_of_parts_selected_label = tk.StringVar()

        # combo box for selecting AGVs for part locations
        self.tab1_option0_selected_agv_cb = None
        self.tab1_option0_selected_agv_value = tk.StringVar()

        # combobox on the number of parts to place on an AGV
        self.part_number_cb = None

        self.tab1_option0_part_label_1 = None
        self.tab1_option0_part_label_1_value = tk.StringVar()
        self.tab1_option0_part_type_1 = None
        self.tab1_option0_part_type_1_value = tk.StringVar()
        self.tab1_option0_part_color_1 = None
        self.tab1_option0_part_color_1_value = tk.StringVar()
        self.tab1_option0_part_quadrant_1 = None
        self.tab1_option0_part_quadrant_1_value = tk.StringVar()
        self.tab1_option0_part_rotation_1 = None
        self.tab1_option0_part_rotation_1_value = tk.StringVar()

        self.tab1_option0_part_label_2 = None
        self.tab1_option0_part_label_2_value = tk.StringVar()
        self.tab1_option0_part_type_2 = None
        self.tab1_option0_part_type_2_value = tk.StringVar()
        self.tab1_option0_part_color_2 = None
        self.tab1_option0_part_color_2_value = tk.StringVar()
        self.tab1_option0_part_quadrant_2 = None
        self.tab1_option0_part_quadrant_2_value = tk.StringVar()
        self.tab1_option0_part_rotation_2 = None
        self.tab1_option0_part_rotation_2_value = tk.StringVar()

        self.tab1_option0_part_label_3 = None
        self.tab1_option0_part_label_3_value = tk.StringVar()
        self.tab1_option0_part_type_3 = None
        self.tab1_option0_part_type_3_value = tk.StringVar()
        self.tab1_option0_part_color_3 = None
        self.tab1_option0_part_color_3_value = tk.StringVar()
        self.tab1_option0_part_quadrant_3 = None
        self.tab1_option0_part_quadrant_3_value = tk.StringVar()
        self.tab1_option0_part_rotation_3 = None
        self.tab1_option0_part_rotation_3_value = tk.StringVar()

        self.tab1_option0_part_label_4 = None
        self.tab1_option0_part_label_4_value = tk.StringVar()
        self.tab1_option0_part_type_4 = None
        self.tab1_option0_part_type_4_value = tk.StringVar()
        self.tab1_option0_part_color_4 = None
        self.tab1_option0_part_color_4_value = tk.StringVar()
        self.tab1_option0_part_quadrant_4 = None
        self.tab1_option0_part_quadrant_4_value = tk.StringVar()
        self.tab1_option0_part_rotation_4 = None
        self.tab1_option0_part_rotation_4_value = tk.StringVar()

        # Combo boxes

        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        # part type
        self.tab1_option0_part_type_1_cb_value = tk.StringVar()
        self.tab1_option0_part_type_1_cb = None

        # part color
        self.tab1_option0_part_color_1_cb_value = tk.StringVar()
        self.tab1_option0_part_color_1_cb = None

        # part quadrant
        self.tab1_option0_part_quadrant_1_cb_value = tk.IntVar()
        self.tab1_option0_part_quadrant_1_cb = None

        # part rotation
        self.tab1_option0_part_rotation_1_cb_value = tk.StringVar()
        self.tab1_option0_part_rotation_1_cb = None

        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        # part type
        self.tab1_option0_part_type_2_cb_value = tk.StringVar()
        self.tab1_option0_part_type_2_cb = None

        # part color
        self.tab1_option0_part_color_2_cb_value = tk.StringVar()
        self.tab1_option0_part_color_2_cb = None

        # part quadrant
        self.tab1_option0_part_quadrant_2_cb_value = tk.IntVar()
        self.tab1_option0_part_quadrant_2_cb = None

        # part rotation
        self.tab1_option0_part_rotation_2_cb_value = tk.StringVar()
        self.tab1_option0_part_rotation_2_cb = None

        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        # part type
        self.tab1_option0_part_type_3_cb_value = tk.StringVar()
        self.tab1_option0_part_type_3_cb = None

        # part color
        self.tab1_option0_part_color_3_cb_value = tk.StringVar()
        self.tab1_option0_part_color_3_cb = None

        # part quadrant
        self.tab1_option0_part_quadrant_3_cb_value = tk.IntVar()
        self.tab1_option0_part_quadrant_3_cb = None

        # part rotation
        self.tab1_option0_part_rotation_3_cb_value = tk.StringVar()
        self.tab1_option0_part_rotation_3_cb = None

        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------
        # part type
        self.tab1_option0_part_type_4_cb_value = tk.StringVar()
        self.tab1_option0_part_type_4_cb = None

        # part color
        self.tab1_option0_part_color_4_cb_value = tk.StringVar()
        self.tab1_option0_part_color_4_cb = None

        # part quadrant
        self.tab1_option0_part_quadrant_4_cb_value = tk.IntVar()
        self.tab1_option0_part_quadrant_4_cb = None

        # part rotation
        self.tab1_option0_part_rotation_4_cb_value = tk.StringVar()
        self.tab1_option0_part_rotation_4_cb = None

        # Flags
        # set some flags to true so we know that the combo box has been created
        self.tab1_option0_part_info_combo_box_1_created = False
        self.tab1_option0_part_info_combo_box_2_created = False
        self.tab1_option0_part_info_combo_box_3_created = False
        self.tab1_option0_part_info_combo_box_4_created = False

        # create a menu bar
        self.create_menu()
        # create tabs
        self.create_tabs()

    def delete_part_number_combo_box(self):
        self.part_number_cb.grid_forget()
        self.part_number_cb.destroy()
        self.part_number_cb = None

    def delete_tab1_option0_part_info_combo_box(self):
        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        if self.tab1_option0_part_info_combo_box_1_created:
            self.tab1_option0_part_type_1_cb.grid_forget()
            self.tab1_option0_part_type_1_cb.destroy()
            self.tab1_option0_part_type_1_cb = None

            self.tab1_option0_part_color_1_cb.grid_forget()
            self.tab1_option0_part_color_1_cb.destroy()
            self.tab1_option0_part_color_1_cb = None

            self.tab1_option0_part_quadrant_1_cb.grid_forget()
            self.tab1_option0_part_quadrant_1_cb.destroy()
            self.tab1_option0_part_quadrant_1_cb = None

            self.tab1_option0_part_rotation_1_cb.grid_forget()
            self.tab1_option0_part_rotation_1_cb.destroy()
            self.tab1_option0_part_rotation_1_cb = None

            self.tab1_option0_part_info_combo_box_1_created = False
        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        if self.tab1_option0_part_info_combo_box_2_created:
            self.tab1_option0_part_type_2_cb.grid_forget()
            self.tab1_option0_part_type_2_cb.destroy()
            self.tab1_option0_part_type_2_cb = None

            self.tab1_option0_part_color_2_cb.grid_forget()
            self.tab1_option0_part_color_2_cb.destroy()
            self.tab1_option0_part_color_2_cb = None

            self.tab1_option0_part_quadrant_2_cb.grid_forget()
            self.tab1_option0_part_quadrant_2_cb.destroy()
            self.tab1_option0_part_quadrant_2_cb = None

            self.tab1_option0_part_rotation_2_cb.grid_forget()
            self.tab1_option0_part_rotation_2_cb.destroy()
            self.tab1_option0_part_rotation_2_cb = None

            self.tab1_option0_part_info_combo_box_2_created = False
        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        if self.tab1_option0_part_info_combo_box_3_created:
            self.tab1_option0_part_type_3_cb.grid_forget()
            self.tab1_option0_part_type_3_cb.destroy()
            self.tab1_option0_part_type_3_cb = None

            self.tab1_option0_part_color_3_cb.grid_forget()
            self.tab1_option0_part_color_3_cb.destroy()
            self.tab1_option0_part_color_3_cb = None

            self.tab1_option0_part_quadrant_3_cb.grid_forget()
            self.tab1_option0_part_quadrant_3_cb.destroy()
            self.tab1_option0_part_quadrant_3_cb = None

            self.tab1_option0_part_rotation_3_cb.grid_forget()
            self.tab1_option0_part_rotation_3_cb.destroy()
            self.tab1_option0_part_rotation_3_cb = None

            self.tab1_option0_part_info_combo_box_3_created = False
        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------
        if self.tab1_option0_part_info_combo_box_4_created:
            self.tab1_option0_part_type_4_cb.grid_forget()
            self.tab1_option0_part_type_4_cb.destroy()
            self.tab1_option0_part_type_4_cb = None

            self.tab1_option0_part_color_4_cb.grid_forget()
            self.tab1_option0_part_color_4_cb.destroy()
            self.tab1_option0_part_color_4_cb = None

            self.tab1_option0_part_quadrant_4_cb.grid_forget()
            self.tab1_option0_part_quadrant_4_cb.destroy()
            self.tab1_option0_part_quadrant_4_cb = None

            self.tab1_option0_part_rotation_4_cb.grid_forget()
            self.tab1_option0_part_rotation_4_cb.destroy()
            self.tab1_option0_part_rotation_4_cb = None

            self.tab1_option0_part_info_combo_box_4_created = False

    def clear_tab1_option0_part_info(self):
        """Clear all the labels for part location on AGVs
        """
        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        self.tab1_option0_part_label_1_value.set("")
        self.tab1_option0_part_type_1_value.set("")
        self.tab1_option0_part_color_1_value.set("")
        self.tab1_option0_part_quadrant_1_value.set("")
        self.tab1_option0_part_rotation_1_value.set("")

        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        self.tab1_option0_part_label_2_value.set("")
        self.tab1_option0_part_type_2_value.set("")
        self.tab1_option0_part_color_2_value.set("")
        self.tab1_option0_part_quadrant_2_value.set("")
        self.tab1_option0_part_rotation_2_value.set("")

        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        self.tab1_option0_part_label_3_value.set("")
        self.tab1_option0_part_type_3_value.set("")
        self.tab1_option0_part_color_3_value.set("")
        self.tab1_option0_part_quadrant_3_value.set("")
        self.tab1_option0_part_rotation_3_value.set("")

        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------
        self.tab1_option0_part_label_4_value.set("")
        self.tab1_option0_part_type_4_value.set("")
        self.tab1_option0_part_color_4_value.set("")
        self.tab1_option0_part_quadrant_4_value.set("")
        self.tab1_option0_part_rotation_4_value.set("")

    def create_part_info_for_agv(self):
        """Create a combobox to select the part for the selected AGV.

        Each AGV can have up to 4 parts.
        """

        # Which AGV is selected
        agv = self.tab1_option0_selected_agv_value.get()
        output = ""

        # Create a combobox to select the number of parts to spawn on the selected AGV
        # The combobox will be destroyed when a new AGV is selected
        self.part_number_cb = ttk.Combobox(
            self.right_part_location_frame,
            textvariable=self.number_of_parts_on_agv,
            state='readonly',
            width=3,
            font='Arial 12')
        self.part_number_cb['values'] = ('0', '1', '2', '3', '4')

        # If the selected AGV is agv1, agv2, agv3, or agv4
        if agv in ['agv1', 'agv2', 'agv3', 'agv4']:
            output = "How many parts to spawn on "+agv+"?"
            self.part_number_cb.grid(column=0, row=1, sticky='w', padx=270)
            self.part_number_cb.set('0')

        # Update the text for the label
        self.number_of_parts_selected_label.set(output)

        # Create the label
        label = ttk.Label(self.right_part_location_frame,
                          textvariable=self.number_of_parts_selected_label,
                          font='Arial 12', anchor='w').grid(column=0, row=1, padx=30, sticky='w')

        # Callback for the combobox
        self.part_number_cb.bind(
            '<<ComboboxSelected>>', self.on_part_number_selected_callback)

    def on_part_number_selected_callback(self, event):
        """Handle the selection of a part for an AGV."""

        self.delete_tab1_option0_part_info_combo_box()

        # Parse dictionary to get the parts for the selected AGV
        parts = part_location_agvs_dict[self.tab1_option0_selected_agv_value.get(
        )]['parts']

        part1_result = parts.get('part1', None)
        part1_info = []

        part2_result = parts.get('part2', None)
        part2_info = []

        part3_result = parts.get('part3', None)
        part3_info = []

        part4_result = parts.get('part4', None)
        part4_info = []

        if part1_result is not None:
            if parts['part1'].get('type', None) is not None:
                part1_info.append(part1_result['type'])
                part1_info.append(part1_result['color'])
                part1_info.append(part1_result['quadrant'])
                part1_info.append(part1_result['rotation'])

        if part2_result is not None:
            if parts['part2'].get('type', None) is not None:
                part2_info.append(part2_result['type'])
                part2_info.append(part2_result['color'])
                part2_info.append(part2_result['quadrant'])
                part2_info.append(part2_result['rotation'])

        if part3_result is not None:
            if parts['part3'].get('type', None) is not None:
                part3_info.append(part3_result['type'])
                part3_info.append(part3_result['color'])
                part3_info.append(part3_result['quadrant'])
                part3_info.append(part3_result['rotation'])

        if part4_result is not None:
            if parts['part4'].get('type', None) is not None:
                part4_info.append(part4_result['type'])
                part4_info.append(part4_result['color'])
                part4_info.append(part4_result['quadrant'])
                part4_info.append(part4_result['rotation'])

        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        self.tab1_option0_part_label_1_value.set("")
        self.tab1_option0_part_type_1_value.set("")
        self.tab1_option0_part_color_1_value.set("")
        self.tab1_option0_part_quadrant_1_value.set("")
        self.tab1_option0_part_rotation_1_value.set("")

        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        self.tab1_option0_part_label_2_value.set("")
        self.tab1_option0_part_type_2_value.set("")
        self.tab1_option0_part_color_2_value.set("")
        self.tab1_option0_part_quadrant_2_value.set("")
        self.tab1_option0_part_rotation_2_value.set("")

        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        self.tab1_option0_part_label_3_value.set("")
        self.tab1_option0_part_type_3_value.set("")
        self.tab1_option0_part_color_3_value.set("")
        self.tab1_option0_part_quadrant_3_value.set("")
        self.tab1_option0_part_rotation_3_value.set("")

        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------
        self.tab1_option0_part_label_4_value.set("")
        self.tab1_option0_part_type_4_value.set("")
        self.tab1_option0_part_color_4_value.set("")
        self.tab1_option0_part_quadrant_4_value.set("")
        self.tab1_option0_part_rotation_4_value.set("")

        # ----------------------------------------
        # -- CREATE LABELS --
        # ----------------------------------------

        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        self.tab1_option0_part_label_1 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_label_1_value,
                                                   font='Arial 13 bold',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_type_1 = ttk.Label(self.right_part_location_frame,
                                                  textvariable=self.tab1_option0_part_type_1_value,
                                                  font='Arial 12',
                                                  justify=LEFT,
                                                  anchor="w")

        self.tab1_option0_part_color_1 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_color_1_value,
                                                   font='Arial 12',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_quadrant_1 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_quadrant_1_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        self.tab1_option0_part_rotation_1 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_rotation_1_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        self.tab1_option0_part_label_2 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_label_2_value,
                                                   font='Arial 13 bold',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_type_2 = ttk.Label(self.right_part_location_frame,
                                                  textvariable=self.tab1_option0_part_type_2_value,
                                                  font='Arial 12',
                                                  justify=LEFT,
                                                  anchor="w")

        self.tab1_option0_part_color_2 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_color_2_value,
                                                   font='Arial 12',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_quadrant_2 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_quadrant_2_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        self.tab1_option0_part_rotation_2 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_rotation_2_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        self.tab1_option0_part_label_3 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_label_3_value,
                                                   font='Arial 13 bold',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_type_3 = ttk.Label(self.right_part_location_frame,
                                                  textvariable=self.tab1_option0_part_type_3_value,
                                                  font='Arial 12',
                                                  justify=LEFT,
                                                  anchor="w")

        self.tab1_option0_part_color_3 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_color_3_value,
                                                   font='Arial 12',
                                                   justify=LEFT,
                                                   anchor="w")

        self.tab1_option0_part_quadrant_3 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_quadrant_3_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        self.tab1_option0_part_rotation_3 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_rotation_3_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------

        # part label
        self.tab1_option0_part_label_4 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_label_4_value,
                                                   font='Arial 13 bold',
                                                   justify=LEFT,
                                                   anchor="w")

        # part type
        self.tab1_option0_part_type_4 = ttk.Label(self.right_part_location_frame,
                                                  textvariable=self.tab1_option0_part_type_4_value,
                                                  font='Arial 12',
                                                  justify=LEFT,
                                                  anchor="w")
        # part color
        self.tab1_option0_part_color_4 = ttk.Label(self.right_part_location_frame,
                                                   textvariable=self.tab1_option0_part_color_4_value,
                                                   font='Arial 12',
                                                   justify=LEFT,
                                                   anchor="w")
        # part quadrant
        self.tab1_option0_part_quadrant_4 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_quadrant_4_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        # part rotation
        self.tab1_option0_part_rotation_4 = ttk.Label(self.right_part_location_frame,
                                                      textvariable=self.tab1_option0_part_rotation_4_value,
                                                      font='Arial 12',
                                                      justify=LEFT,
                                                      anchor="w")

        # ----------------------------------------
        # -- POSITION LABELS and COMBOBOXES--
        # ----------------------------------------

        # ----------------------------------------
        # -- Part 1 --
        # ----------------------------------------
        if self.number_of_parts_on_agv.get() in ["1", "2", "3", "4"]:
            # set flag to true so we know that the combo box has been created
            self.tab1_option0_part_info_combo_box_1_created = True

            self.tab1_option0_part_label_1_value.set("Part1 ---")
            self.tab1_option0_part_type_1_value.set("Part type: ")
            self.tab1_option0_part_color_1_value.set("Part color: ")
            self.tab1_option0_part_quadrant_1_value.set("Part quadrant: ")
            self.tab1_option0_part_rotation_1_value.set("Part rotation: ")

            # --- PART LABEL ---
            self.tab1_option0_part_label_1.grid(
                sticky='w', column=0, row=4, pady=5)

            # --- PART TYPE ---
            self.tab1_option0_part_type_1.grid(
                sticky='w', column=0, row=5, pady=2)
            # add combobox for part type
            self.tab1_option0_part_type_1_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_type_1_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_types)
            # prefilled part type from dictionary
            if part1_info:
                if part1_info[0] in self.part_types:
                    self.tab1_option0_part_type_1_cb.current(
                        self.part_types.index(part1_info[0]))
            else:
                self.tab1_option0_part_type_1_cb.current(0)

            self.tab1_option0_part_type_1_cb.grid(
                sticky='w', column=1, row=5, pady=2)

            # --- PART COLOR ---
            self.tab1_option0_part_color_1.grid(
                sticky='w', column=0, row=6, padx=0, pady=2)

            # add combobox for part color
            self.tab1_option0_part_color_1_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_color_1_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_colors)
            # prefilled part color from dictionary
            if part1_info:
                if part1_info[1] in self.part_colors:
                    self.tab1_option0_part_color_1_cb.current(
                        self.part_colors.index(part1_info[1]))
            else:
                self.tab1_option0_part_color_1_cb.current(0)

            self.tab1_option0_part_color_1_cb.grid(
                sticky='w', column=1, row=6, pady=2)

            # --- PART QUADRANT ---
            self.tab1_option0_part_quadrant_1.grid(
                sticky='w', column=0, row=7, padx=0, pady=2)

            # add combobox for part quadrant
            self.tab1_option0_part_quadrant_1_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_quadrant_1_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_quadrants)
            # prefilled part quadrant from dictionary
            if part1_info:
                if part1_info[2] in self.part_quadrants:
                    self.tab1_option0_part_quadrant_1_cb.current(
                        self.part_quadrants.index(part1_info[2]))
            else:
                self.tab1_option0_part_quadrant_1_cb.current(0)

            self.tab1_option0_part_quadrant_1_cb.grid(
                sticky='w', column=1, row=7, pady=2)

            # --- PART ROTATION ---
            self.tab1_option0_part_rotation_1.grid(
                sticky='w', column=0, row=8, padx=0, pady=2)

            # add combobox for part rotation
            self.tab1_option0_part_rotation_1_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_rotation_1_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_rotations)
            # prefilled part rotation from dictionary
            if part1_info:
                if part1_info[3] in self.part_rotations:
                    self.tab1_option0_part_rotation_1_cb.current(
                        self.part_rotations.index(part1_info[3]))
            else:
                self.tab1_option0_part_rotation_1_cb.current(0)

            self.tab1_option0_part_rotation_1_cb.grid(
                sticky='w', column=1, row=8, pady=2)

        # ----------------------------------------
        # -- Part 2 --
        # ----------------------------------------
        if self.number_of_parts_on_agv.get() in ["2", "3", "4"]:

            # set flag to true so we know that the combo box has been created
            self.tab1_option0_part_info_combo_box_2_created = True

            self.tab1_option0_part_label_2_value.set("Part2 ---")
            self.tab1_option0_part_type_2_value.set("Part type: ")
            self.tab1_option0_part_color_2_value.set("Part color: ")
            self.tab1_option0_part_quadrant_2_value.set("Part quadrant: ")
            self.tab1_option0_part_rotation_2_value.set("Part rotation: ")

            # --- PART LABEL ---
            self.tab1_option0_part_label_2.grid(
                sticky='w', column=0, row=9, pady=5)

            # --- PART TYPE ---
            self.tab1_option0_part_type_2.grid(
                sticky='w', column=0, row=10, pady=2)
            # add combobox for part type
            self.tab1_option0_part_type_2_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_type_2_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_types)
            # prefilled part type from dictionary
            if part2_info:
                if part2_info[0] in self.part_types:
                    self.tab1_option0_part_type_2_cb.current(
                        self.part_types.index(part2_info[0]))
            else:
                self.tab1_option0_part_type_2_cb.current(0)

            self.tab1_option0_part_type_2_cb.grid(
                sticky='w', column=1, row=10, pady=2)

            # --- PART COLOR ---
            self.tab1_option0_part_color_2.grid(
                sticky='w', column=0, row=11, padx=0, pady=2)

            # add combobox for part color
            self.tab1_option0_part_color_2_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_color_2_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_colors)
            # prefilled part color from dictionary
            if part2_info:
                if part2_info[1] in self.part_colors:
                    self.tab1_option0_part_color_2_cb.current(
                        self.part_colors.index(part2_info[1]))
            else:
                self.tab1_option0_part_color_2_cb.current(0)

            self.tab1_option0_part_color_2_cb.grid(
                sticky='w', column=1, row=11, pady=2)

            # --- PART QUADRANT ---
            self.tab1_option0_part_quadrant_2.grid(
                sticky='w', column=0, row=12, padx=0, pady=2)

            # add combobox for part quadrant
            self.tab1_option0_part_quadrant_2_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_quadrant_2_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_quadrants)
            # prefilled part quadrant from dictionary
            if part2_info:
                if part2_info[2] in self.part_quadrants:
                    self.tab1_option0_part_quadrant_2_cb.current(
                        self.part_quadrants.index(part2_info[2]))
            else:
                self.tab1_option0_part_quadrant_2_cb.current(0)

            self.tab1_option0_part_quadrant_2_cb.grid(
                sticky='w', column=1, row=12, pady=2)

            # --- PART ROTATION ---
            self.tab1_option0_part_rotation_2.grid(
                sticky='w', column=0, row=13, padx=0, pady=2)

            # add combobox for part rotation
            self.tab1_option0_part_rotation_2_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_rotation_2_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_rotations)
            # prefilled part rotation from dictionary
            if part2_info:
                if part2_info[3] in self.part_rotations:
                    self.tab1_option0_part_rotation_2_cb.current(
                        self.part_rotations.index(part2_info[3]))
            else:
                self.tab1_option0_part_rotation_2_cb.current(0)

            self.tab1_option0_part_rotation_2_cb.grid(
                sticky='w', column=1, row=13, pady=2)

        # ----------------------------------------
        # -- Part 3 --
        # ----------------------------------------
        if self.number_of_parts_on_agv.get() in ["3", "4"]:

            # set flag to true so we know that the combo box has been created
            self.tab1_option0_part_info_combo_box_3_created = True

            self.tab1_option0_part_label_3_value.set("Part3 ---")
            self.tab1_option0_part_type_3_value.set("Part type: ")
            self.tab1_option0_part_color_3_value.set("Part color: ")
            self.tab1_option0_part_quadrant_3_value.set("Part quadrant: ")
            self.tab1_option0_part_rotation_3_value.set("Part rotation: ")

            # --- PART LABEL ---
            self.tab1_option0_part_label_3.grid(
                sticky='w', column=0, row=14, pady=5)

            # --- PART TYPE ---
            self.tab1_option0_part_type_3.grid(
                sticky='w', column=0, row=15, pady=2)
            # add combobox for part type
            self.tab1_option0_part_type_3_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_type_3_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_types)
            # prefilled part type from dictionary
            if part3_info:
                if part3_info[0] in self.part_types:
                    self.tab1_option0_part_type_3_cb.current(
                        self.part_types.index(part3_info[0]))
            else:
                self.tab1_option0_part_type_3_cb.current(0)

            self.tab1_option0_part_type_3_cb.grid(
                sticky='w', column=1, row=15, pady=2)

            # --- PART COLOR ---
            self.tab1_option0_part_color_3.grid(
                sticky='w', column=0, row=16, padx=0, pady=2)

            # add combobox for part color
            self.tab1_option0_part_color_3_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_color_3_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_colors)
            # prefilled part color from dictionary
            if part3_info:
                if part3_info[1] in self.part_colors:
                    self.tab1_option0_part_color_3_cb.current(
                        self.part_colors.index(part3_info[1]))
            else:
                self.tab1_option0_part_color_3_cb.current(0)

            self.tab1_option0_part_color_3_cb.grid(
                sticky='w', column=1, row=16, pady=2)

            # --- PART QUADRANT ---
            self.tab1_option0_part_quadrant_3.grid(
                sticky='w', column=0, row=17, padx=0, pady=2)

            # add combobox for part quadrant
            self.tab1_option0_part_quadrant_3_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_quadrant_3_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_quadrants)
            # prefilled part quadrant from dictionary
            if part3_info:
                if part3_info[2] in self.part_quadrants:
                    self.tab1_option0_part_quadrant_3_cb.current(
                        self.part_quadrants.index(part3_info[2]))
            else:
                self.tab1_option0_part_quadrant_3_cb.current(0)

            self.tab1_option0_part_quadrant_3_cb.grid(
                sticky='w', column=1, row=17, pady=2)

            # --- PART ROTATION ---
            self.tab1_option0_part_rotation_3.grid(
                sticky='w', column=0, row=18, padx=0, pady=2)

            # add combobox for part rotation
            self.tab1_option0_part_rotation_3_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_rotation_3_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_rotations)
            # prefilled part rotation from dictionary
            if part3_info:
                if part3_info[3] in self.part_rotations:
                    self.tab1_option0_part_rotation_3_cb.current(
                        self.part_rotations.index(part3_info[3]))
            else:
                self.tab1_option0_part_rotation_3_cb.current(0)

            self.tab1_option0_part_rotation_3_cb.grid(
                sticky='w', column=1, row=18, pady=2)

        # ----------------------------------------
        # -- Part 4 --
        # ----------------------------------------
        if self.number_of_parts_on_agv.get() in ["4"]:

            # set flag to true so we know that the combo box has been created
            self.tab1_option0_part_info_combo_box_4_created = True

            self.tab1_option0_part_label_4_value.set("Part4 ---")
            self.tab1_option0_part_type_4_value.set("Part type: ")
            self.tab1_option0_part_color_4_value.set("Part color: ")
            self.tab1_option0_part_quadrant_4_value.set("Part quadrant: ")
            self.tab1_option0_part_rotation_4_value.set("Part rotation: ")

            # --- PART LABEL ---
            self.tab1_option0_part_label_4.grid(
                sticky='w', column=0, row=19, pady=5)

            # --- PART TYPE ---
            self.tab1_option0_part_type_4.grid(
                sticky='w', column=0, row=20, pady=2)
            # add combobox for part type
            self.tab1_option0_part_type_4_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_type_4_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_types)
            # prefilled part type from dictionary
            if part4_info:
                if part4_info[0] in self.part_types:
                    self.tab1_option0_part_type_4_cb.current(
                        self.part_types.index(part4_info[0]))
            else:
                self.tab1_option0_part_type_4_cb.current(0)

            self.tab1_option0_part_type_4_cb.grid(
                sticky='w', column=1, row=20, pady=2)

            # --- PART COLOR ---
            self.tab1_option0_part_color_4.grid(
                sticky='w', column=0, row=21, padx=0, pady=2)

            # add combobox for part color
            self.tab1_option0_part_color_4_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_color_4_cb_value,
                state='readonly',
                width=10,
                font='Arial 12',
                values=self.part_colors)
            # prefilled part color from dictionary
            if part4_info:
                if part4_info[1] in self.part_colors:
                    self.tab1_option0_part_color_4_cb.current(
                        self.part_colors.index(part4_info[1]))
            else:
                self.tab1_option0_part_color_4_cb.current(0)

            self.tab1_option0_part_color_4_cb.grid(
                sticky='w', column=1, row=21, pady=2)

            # --- PART QUADRANT ---
            self.tab1_option0_part_quadrant_4.grid(
                sticky='w', column=0, row=22, padx=0, pady=2)

            # add combobox for part quadrant
            self.tab1_option0_part_quadrant_4_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_quadrant_4_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_quadrants)
            # prefilled part quadrant from dictionary
            if part4_info:
                if part4_info[2] in self.part_quadrants:
                    self.tab1_option0_part_quadrant_4_cb.current(
                        self.part_quadrants.index(part4_info[2]))
            else:
                self.tab1_option0_part_quadrant_4_cb.current(0)

            self.tab1_option0_part_quadrant_4_cb.grid(
                sticky='w', column=1, row=22, pady=2)

            # --- PART ROTATION ---
            self.tab1_option0_part_rotation_4.grid(
                sticky='w', column=0, row=23, padx=0, pady=2)

            # add combobox for part rotation
            self.tab1_option0_part_rotation_4_cb = ttk.Combobox(
                self.right_part_location_frame,
                textvariable=self.tab1_option0_part_rotation_4_cb_value,
                state='readonly',
                width=5,
                font='Arial 12',
                values=self.part_rotations)
            # prefilled part rotation from dictionary
            if part4_info:
                if part4_info[3] in self.part_rotations:
                    self.tab1_option0_part_rotation_4_cb.current(
                        self.part_rotations.index(part4_info[3]))
            else:
                self.tab1_option0_part_rotation_4_cb.current(0)

            self.tab1_option0_part_rotation_4_cb.grid(
                sticky='w', column=1, row=23, pady=2)

    def create_menu(self):
        """Create a menu bar with File and Edit options.
        """

        # Menu Bar
        menu = Menu(self)
        self.config(menu=menu)

        # File Menu
        fileMenu = Menu(menu)
        fileMenu.add_command(label="Open")
        fileMenu.add_command(label="Save")
        fileMenu.add_command(label="Exit", command=self.exitProgram)
        menu.add_cascade(label="File", menu=fileMenu)
        
        # Help Menu
        helpMenu = Menu(menu)
        helpMenu.add_command(label = "About", command = self.exitProgram)
        menu.add_cascade(label="Help", menu=helpMenu)

    def exitProgram(self):
        """Exit the program.
        """
        exit()

    def on_agv_selected_callback(self, event):
        """ Handle the selection of an AGV in the part location tab. """

        """Handle the radio button selection in the part location tab.
        """
        # Mark the radio button as selected
        # self.tab1_option0_radio_value.set(1)

        # clear the fields for the part information
        self.clear_tab1_option0_part_info()

        # delete widgets from the part information frame
        self.delete_tab1_option0_part_info_combo_box()

        # if an AGV is selected
        if self.tab1_option0_selected_agv_value.get() != 'None':
            # set the radio button as selected
            output = "Part location on AGVs: " + self.tab1_option0_selected_agv_value.get()
            # Create the form to fill out the part information for the selected AGV
            self.create_part_info_for_agv()
        else:
            output = "Select an AGV."
            self.number_of_parts_selected_label.set("")
            self.delete_part_number_combo_box()

        self.agv_selection_label.set(output)

        # Update the label
        label = ttk.Label(self.right_part_location_frame,
                          textvariable=self.agv_selection_label, font='Arial 17 bold', anchor="w").grid(column=0,
                                                                                                        row=0,
                                                                                                        sticky='w',
                                                                                                        padx=30,
                                                                                                        pady=30)
        save_button = ttk.Button(self.right_part_location_frame, text="Save", command=self.save_part_location_clicked).grid(column=2,
                                                                                                                            row=0,
                                                                                                                            sticky='e',
                                                                                                                            pady=30)

    def save_part_location_clicked(self):
        # Parse dictionary to get the parts for the selected AGV
        parts = part_location_agvs_dict[self.tab1_option0_selected_agv_value.get(
        )]['parts']

        if self.tab1_option0_part_info_combo_box_1_created:
            part1 = {}
            part1['type'] = self.tab1_option0_part_type_1_cb_value.get()
            part1['color'] = self.tab1_option0_part_color_1_cb_value.get()
            part1['quadrant'] = self.tab1_option0_part_quadrant_1_cb_value.get()
            part1['rotation'] = self.tab1_option0_part_rotation_1_cb_value.get()
            parts['part1'] = part1
        else:
            part1 = {}
            parts['part1'] = part1

        if self.tab1_option0_part_info_combo_box_2_created:
            part2 = {}
            part2['type'] = self.tab1_option0_part_type_2_cb_value.get()
            part2['color'] = self.tab1_option0_part_color_2_cb_value.get()
            part2['quadrant'] = self.tab1_option0_part_quadrant_2_cb_value.get()
            part2['rotation'] = self.tab1_option0_part_rotation_2_cb_value.get()
            parts['part2'] = part2
        else:
            part2 = {}
            parts['part2'] = part2

        if self.tab1_option0_part_info_combo_box_3_created:
            part3 = {}
            part3['type'] = self.tab1_option0_part_type_3_cb_value.get()
            part3['color'] = self.tab1_option0_part_color_3_cb_value.get()
            part3['quadrant'] = self.tab1_option0_part_quadrant_3_cb_value.get()
            part3['rotation'] = self.tab1_option0_part_rotation_3_cb_value.get()
            parts['part3'] = part3
        else:
            part3 = {}
            parts['part3'] = part3

        if self.tab1_option0_part_info_combo_box_4_created:
            part4 = {}
            part4['type'] = self.tab1_option0_part_type_4_cb_value.get()
            part4['color'] = self.tab1_option0_part_color_4_cb_value.get()
            part4['quadrant'] = self.tab1_option0_part_quadrant_4_cb_value.get()
            part4['rotation'] = self.tab1_option0_part_rotation_4_cb_value.get()
            parts['part4'] = part4
        else:
            part4 = {}
            parts['part4'] = part4

        pprint(part_location_agvs_dict)

    def create_part_location_tab(self):
        """Create the part location tab for the GUI.
        """
        part_location_tab = ttk.Frame(self.tab_control)

        part_location_tab.grid_columnconfigure(0, weight=1)
        part_location_tab.grid_columnconfigure(1, weight=1)
        part_location_tab.grid_rowconfigure(0, weight=1)

        # Left window
        self.left_part_location_frame = ttk.Frame(part_location_tab)
        self.left_part_location_frame.grid(row=0, column=0, sticky="nesw")
        # Right window
        self.right_part_location_frame = ttk.Frame(part_location_tab)
        self.right_part_location_frame.grid(row=0, column=1, sticky="nesw")

        # Title of the tab
        self.tab_control.add(
            part_location_tab, text='Part Locations', compound=TOP)

        # Description of the tab
        ttk.Label(self.left_part_location_frame,
                  text="Information on part locations.",
                  font='Arial 17 bold', anchor='w').grid(column=0,
                                                         row=0,
                                                         sticky='w',
                                                         padx=10,
                                                         pady=30)

        self.tab1_option0_selected_agv_cb = ttk.Combobox(
            self.left_part_location_frame,
            textvariable=self.tab1_option0_selected_agv_value,
            state='readonly',
            width=5,
            font='Arial 12',
            values=['None', 'agv1', 'agv2', 'agv3', 'agv4'])

        self.tab1_option0_selected_agv_cb.grid(column=0, row=1, padx=270, pady=0)

        # set the default text
        self.tab1_option0_selected_agv_cb.set('None')

        self.tab1_option0_selected_agv_cb.bind(
            '<<ComboboxSelected>>', self.on_agv_selected_callback)

        ttk.Label(self.left_part_location_frame,
                  text="Part location on AGV(s):",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=1,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=0)

    def create_setup_tab(self):
        """Create the setup tab for the GUI.
        """
        
        # Retrieve information from setup_dict
        time_limit = setup_dict['time_limit']
        slot1 = setup_dict['slots']['slot1']
        slot2 = setup_dict['slots']['slot2']
        slot3 = setup_dict['slots']['slot3']
        slot4 = setup_dict['slots']['slot4']
        slot5 = setup_dict['slots']['slot5']
        slot6 = setup_dict['slots']['slot6']
        
        
        
        setup_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(setup_tab, text='Setup')

        # Label describing the tab
        ttk.Label(setup_tab,
                  text="Information on time limit and kitting trays.",
                  font='Arial 17 bold').grid(column=0,
                                             row=0,
                                             sticky='w',
                                             padx=10,
                                             pady=30)

        # Label time limit
        ttk.Label(setup_tab,
                  text="-- Time limit (-1 = no time limit = default if field is empty)",
                  font='Arial 12 bold', anchor='w').grid(column=0,
                                                         row=1,
                                                         sticky='w',
                                                         padx=30,
                                                         pady=10)

        # Label
        ttk.Label(setup_tab,
                  text="Enter a time limit for the trial:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=2,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        # Text field
        time_limit_entry = tk.Entry(setup_tab, textvariable=self.time_limit_value, width=6)
        time_limit_entry.insert(0, time_limit)
        time_limit_entry.grid(column=0,
                             row=2,
                             sticky='w',
                             padx=270,
                             pady=2)

        # Label kitting trays
        ttk.Label(setup_tab,
                  text="-- Kitting trays",
                  font='Arial 12 bold', anchor='w').grid(column=0,
                                                         row=3,
                                                         sticky='w',
                                                         padx=30,
                                                         pady=10)
        # Slots

        # Slot 1
        ttk.Label(setup_tab,
                  text="Slot 1 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=4,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=4,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)

        self.slot1_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot1_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot1,
            postcommand=self.update_tray_id_list_slot1)

        if slot1 is not None:
            self.slot1_cb.current(self.tray_id_slot1.index(slot1))
        else:
            self.slot1_cb.current(0)

        self.slot1_cb.bind('<<ComboboxSelected>>', self.on_slot1_callback)

        self.slot1_cb.grid(column=0,
                           row=4,
                           padx=270,
                           pady=2)

        # Slot 2
        ttk.Label(setup_tab,
                  text="Slot 2 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=5,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=5,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)




        self.slot2_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot2_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot2,
            postcommand=self.update_tray_id_list_slot2)

        if slot2 is not None:
            self.slot2_cb.current(self.tray_id_slot2.index(slot2))
        else:
            self.slot2_cb.current(0)

        self.slot2_cb.bind('<<ComboboxSelected>>', self.on_slot2_callback)

        self.slot2_cb.grid(column=0,
                           row=5,
                           padx=270,
                           pady=2)

        # Slot 3
        ttk.Label(setup_tab,
                  text="Slot 3 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=6,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=6,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)

        self.slot3_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot3_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot3,
            postcommand=self.update_tray_id_list_slot3)

        if slot3 is not None:
            self.slot3_cb.current(self.tray_id_slot3.index(slot3))
        else:
            self.slot3_cb.current(0)

        self.slot3_cb.bind(
            '<<ComboboxSelected>>', self.on_slot3_callback)

        self.slot3_cb.grid(column=0,
                           row=6,
                           padx=270,
                           pady=2)

        # Slot 4
        ttk.Label(setup_tab,
                  text="Slot 4 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=7,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=7,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)

        self.slot4_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot4_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot4,
            postcommand=self.update_tray_id_list_slot4)

        if slot4 is not None:
            self.slot4_cb.current(self.tray_id_slot4.index(slot4))
        else:
            self.slot4_cb.current(0)

        self.slot4_cb.bind(
            '<<ComboboxSelected>>', self.on_slot4_callback)

        self.slot4_cb.grid(column=0,
                           row=7,
                           padx=270,
                           pady=2)

        # Slot 5
        ttk.Label(setup_tab,
                  text="Slot 5 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=8,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=8,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)

        self.slot5_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot5_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot5,
            postcommand=self.update_tray_id_list_slot5)

        if slot5 is not None:
            self.slot5_cb.current(self.tray_id_slot5.index(slot5))
        else:
            self.slot5_cb.current(0)

        self.slot5_cb.bind(
            '<<ComboboxSelected>>', self.on_slot5_callback)

        self.slot5_cb.grid(column=0,
                           row=8,
                           padx=270,
                           pady=2)

        # Slot 6
        ttk.Label(setup_tab,
                  text="Slot 6 --",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=9,
                                                    sticky='w',
                                                    padx=30,
                                                    pady=2)

        ttk.Label(setup_tab,
                  text="Tray ID:",
                  font='Arial 12', anchor='w').grid(column=0,
                                                    row=9,
                                                    sticky='w',
                                                    padx=200,
                                                    pady=2)

        self.slot6_cb = ttk.Combobox(
            setup_tab,
            textvariable=self.slot6_value,
            state='readonly',
            width=10,
            font='Arial 12',
            values=self.tray_id_slot6,
            postcommand=self.update_tray_id_list_slot6)

        if slot6 is not None:
            self.slot6_cb.current(self.tray_id_slot6.index(slot6))
        else:
            self.slot6_cb.current(0)

        self.slot6_cb.bind(
            '<<ComboboxSelected>>', self.on_slot6_callback)

        self.slot6_cb.grid(column=0,
                           row=9,
                           padx=270,
                           pady=2)

        # Buttons
        reset_button = ttk.Button(setup_tab, text="Reset", command=self.confirm_reset_tray_ids).grid(column=0,
                                                                                                  row=10,
                                                                                                  sticky='w',
                                                                                                  padx=30,
                                                                                                  pady=10)
        reset_button = ttk.Button(setup_tab, text="Save", command=self.save_setup_clicked).grid(column=0,
                                                                                                row=10,
                                                                                                sticky='w',
                                                                                                padx=270,
                                                                                                pady=10)

    def confirm_reset_tray_ids(self):
        """Confirmation window to reset tray IDs
        """
        answer = askyesno(title='confirmation',
                          message='Do you want to reset the tray IDs for the slots?')
        if answer:
            self.reset_setup_clicked()
            
    def reset_setup_clicked(self):
        """Reinitialize the tray IDs for the slots and reset the comboboxes
        """
        
        self.tray_id_slot1 = copy(self.tray_ids)
        self.tray_id_slot2 = copy(self.tray_ids)
        self.tray_id_slot3 = copy(self.tray_ids)
        self.tray_id_slot4 = copy(self.tray_ids)
        self.tray_id_slot5 = copy(self.tray_ids)
        self.tray_id_slot6 = copy(self.tray_ids)
        
        self.slot1_cb.current(0)
        self.slot2_cb.current(0)
        self.slot3_cb.current(0)
        self.slot4_cb.current(0)
        self.slot5_cb.current(0)
        self.slot6_cb.current(0)

    def save_setup_clicked(self):
        setup_dict = {
            'time_limit': -1,
            'slots': {
                'slot1': None,
                'slot2': None,
                'slot3': '2',
                'slot4': None,
                'slot5': None,
                'slot6': None,
            }
        }
        
        setup_dict['time_limit'] = self.time_limit_value.get()
        if self.slot1_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot1'] = self.slot1_value.get()
        if self.slot2_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot2'] = self.slot2_value.get()
        if self.slot3_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot3'] = self.slot3_value.get()
        if self.slot4_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot4'] = self.slot4_value.get()
        if self.slot5_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot5'] = self.slot5_value.get()
        if self.slot6_value.get() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            setup_dict['slots']['slot6'] = self.slot6_value.get()
            
        pprint(setup_dict)

    def on_slot1_callback(self, event):
        print("on_slot1_callback")
        if self.slot1_value.get() in self.tray_id_slot2:
            print(f'remove {self.slot1_value.get()} from tray_id_slot2')
            self.tray_id_slot2.remove(self.slot1_value.get())
        if self.slot1_value.get() in self.tray_id_slot3:
            print(f'remove {self.slot1_value.get()} from tray_id_slot3')
            self.tray_id_slot3.remove(self.slot1_value.get())
        if self.slot1_value.get() in self.tray_id_slot4:
            print(f'remove {self.slot1_value.get()} from tray_id_slot4')
            self.tray_id_slot4.remove(self.slot1_value.get())
        if self.slot1_value.get() in self.tray_id_slot5:
            print(f'remove {self.slot1_value.get()} from tray_id_slot5')
            self.tray_id_slot5.remove(self.slot1_value.get())
        if self.slot1_value.get() in self.tray_id_slot6:
            print(f'remove {self.slot1_value.get()} from tray_id_slot6')
            self.tray_id_slot6.remove(self.slot1_value.get())
        self.slot2_cb['values'] = self.tray_id_slot2
        self.slot3_cb['values'] = self.tray_id_slot3
        self.slot4_cb['values'] = self.tray_id_slot4
        self.slot5_cb['values'] = self.tray_id_slot5
        self.slot6_cb['values'] = self.tray_id_slot6

    def update_tray_id_list_slot1(self):
        print(f'list: {self.tray_id_slot1}')
        self.slot1_cb['values'] = self.tray_id_slot1

    def on_slot2_callback(self, event):
        print("on_slot2_callback")
        if self.slot2_value.get() in self.tray_id_slot1:
            print(f'remove {self.slot2_value.get()} from tray_id_slot1')
            self.tray_id_slot1.remove(self.slot2_value.get())
        if self.slot2_value.get() in self.tray_id_slot3:
            print(f'remove {self.slot2_value.get()} from tray_id_slot3')
            self.tray_id_slot3.remove(self.slot2_value.get())
        if self.slot2_value.get() in self.tray_id_slot4:
            print(f'remove {self.slot2_value.get()} from tray_id_slot4')
            self.tray_id_slot4.remove(self.slot2_value.get())
        if self.slot2_value.get() in self.tray_id_slot5:
            print(f'remove {self.slot2_value.get()} from tray_id_slot5')
            self.tray_id_slot5.remove(self.slot2_value.get())
        if self.slot2_value.get() in self.tray_id_slot6:
            print(f'remove {self.slot2_value.get()} from tray_id_slot6')
            self.tray_id_slot6.remove(self.slot2_value.get())
        self.slot1_cb['values'] = self.tray_id_slot1
        self.slot3_cb['values'] = self.tray_id_slot3
        self.slot4_cb['values'] = self.tray_id_slot4
        self.slot5_cb['values'] = self.tray_id_slot5
        self.slot6_cb['values'] = self.tray_id_slot6

    def update_tray_id_list_slot2(self):
        print("update_tray_id_list_slot2")
        self.slot2_cb['values'] = self.tray_id_slot2

    def on_slot3_callback(self, event):
        print("on_slot3_callback")
        if self.slot3_value.get() in self.tray_id_slot1:
            self.tray_id_slot1.remove(self.slot3_value.get())
        if self.slot3_value.get() in self.tray_id_slot2:
            self.tray_id_slot2.remove(self.slot3_value.get())
        if self.slot3_value.get() in self.tray_id_slot4:
            self.tray_id_slot4.remove(self.slot3_value.get())
        if self.slot3_value.get() in self.tray_id_slot5:
            self.tray_id_slot5.remove(self.slot3_value.get())
        if self.slot3_value.get() in self.tray_id_slot6:
            self.tray_id_slot6.remove(self.slot3_value.get())
        self.slot1_cb['values'] = self.tray_id_slot1
        self.slot2_cb['values'] = self.tray_id_slot2
        self.slot4_cb['values'] = self.tray_id_slot4
        self.slot5_cb['values'] = self.tray_id_slot5
        self.slot6_cb['values'] = self.tray_id_slot6

    def update_tray_id_list_slot3(self):
        self.slot3_cb['values'] = self.tray_id_slot3

    def on_slot4_callback(self, event):
        print("on_slot4_callback")
        if self.slot4_value.get() in self.tray_id_slot1:
            self.tray_id_slot1.remove(self.slot4_value.get())
        if self.slot4_value.get() in self.tray_id_slot2:
            self.tray_id_slot2.remove(self.slot4_value.get())
        if self.slot4_value.get() in self.tray_id_slot3:
            self.tray_id_slot3.remove(self.slot4_value.get())
        if self.slot4_value.get() in self.tray_id_slot5:
            self.tray_id_slot5.remove(self.slot4_value.get())
        if self.slot4_value.get() in self.tray_id_slot6:
            self.tray_id_slot6.remove(self.slot4_value.get())
        self.slot1_cb['values'] = self.tray_id_slot1
        self.slot2_cb['values'] = self.tray_id_slot2
        self.slot3_cb['values'] = self.tray_id_slot3
        self.slot5_cb['values'] = self.tray_id_slot5
        self.slot6_cb['values'] = self.tray_id_slot6

    def update_tray_id_list_slot4(self):
        self.slot4_cb['values'] = self.tray_id_slot4

    def on_slot5_callback(self, event):
        print("on_slot5_callback")
        if self.slot5_value.get() in self.tray_id_slot1:
            self.tray_id_slot1.remove(self.slot5_value.get())
        if self.slot5_value.get() in self.tray_id_slot2:
            self.tray_id_slot2.remove(self.slot5_value.get())
        if self.slot5_value.get() in self.tray_id_slot3:
            self.tray_id_slot3.remove(self.slot5_value.get())
        if self.slot5_value.get() in self.tray_id_slot4:
            self.tray_id_slot4.remove(self.slot5_value.get())
        if self.slot5_value.get() in self.tray_id_slot6:
            self.tray_id_slot6.remove(self.slot5_value.get())
        self.slot1_cb['values'] = self.tray_id_slot1
        self.slot2_cb['values'] = self.tray_id_slot2
        self.slot3_cb['values'] = self.tray_id_slot3
        self.slot4_cb['values'] = self.tray_id_slot4
        self.slot6_cb['values'] = self.tray_id_slot6

    def update_tray_id_list_slot5(self):
        self.slot5_cb['values'] = self.tray_id_slot5

    def on_slot6_callback(self, event):
        print("on_slot6_callback")
        if self.slot6_value.get() in self.tray_id_slot1:
            self.tray_id_slot1.remove(self.slot6_value.get())
        if self.slot6_value.get() in self.tray_id_slot2:
            self.tray_id_slot2.remove(self.slot6_value.get())
        if self.slot6_value.get() in self.tray_id_slot3:
            self.tray_id_slot3.remove(self.slot6_value.get())
        if self.slot6_value.get() in self.tray_id_slot4:
            self.tray_id_slot4.remove(self.slot6_value.get())
        if self.slot6_value.get() in self.tray_id_slot5:
            self.tray_id_slot5.remove(self.slot6_value.get())
        self.slot1_cb['values'] = self.tray_id_slot1
        self.slot2_cb['values'] = self.tray_id_slot2
        self.slot3_cb['values'] = self.tray_id_slot3
        self.slot4_cb['values'] = self.tray_id_slot4
        self.slot5_cb['values'] = self.tray_id_slot5

    def update_tray_id_list_slot6(self):
        self.slot6_cb['values'] = self.tray_id_slot6

    def create_order_tab(self):
        """Create the order tab for the GUI.
        """
        order_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(order_tab, text='Orders')
        ttk.Label(order_tab,
                  text="Information on orders.",
                  font='Arial 17 bold').grid(column=0,
                                             row=0,
                                             sticky='w',
                                             padx=10,
                                             pady=30)

    def create_challenge_tab(self):
        """Create the challenge tab for the GUI.
        """
        challenge_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(challenge_tab, text='Challenges')
        ttk.Label(challenge_tab,
                  text="Information on challenges.",
                  font='Arial 17 bold').grid(column=0,
                                             row=0,
                                             sticky='w',
                                             padx=10,
                                             pady=30)


                  
    def create_tabs(self):
        """Create the tabs for the GUI."""

        # Tabs
        self.create_setup_tab()
        self.create_part_location_tab()
        self.create_order_tab()
        self.create_challenge_tab()
        self.tab_control.pack(expand=1, fill="both", padx=5, pady=5)


def main():
    app = AriacGUI()
    app.mainloop()


if __name__ == '__main__':
    main()
