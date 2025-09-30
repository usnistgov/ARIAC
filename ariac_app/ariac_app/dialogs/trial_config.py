from copy import copy
from nicegui import events, ui
from itertools import count

import random
import os

import string

from jsonschema import ValidationError

import yaml
from yaml.scanner import ScannerError
from yaml.parser import ParserError

from ament_index_python import get_package_share_directory

from ariac_interfaces.msg import VacuumTools

from ariac_app.dialogs.file_picker import FilePicker
from ariac_app.structures import (
  Challenge, 
  ConveyorMalfunction, 
  VacuumToolMalfunction, 
  VoltageTesterMalfunction,
  HighPriorityOrder,
  ChallengeType,
  Trial,
  TrialInfo
)
  
class TrialConfigBuilder(ui.dialog):
    def __init__(self, info: TrialInfo):
        super().__init__()
        
        self.info = info
        
        self.id_counter = count(len(self.info.challenges))

        self.challenge_types = ['conveyor malfunction', 'vacuum tool malfunction', 'voltage tester malfunction', 'high priority order']

        with self.classes('w-full'), ui.card().classes('w-full'):
            with ui.row().classes('w-full items-center justify-center'):
                ui.label("Trial Builder").classes("text-lg")
            with ui.expansion('General', group="trial_generation", value=True).classes('w-full items-center text-base') as e:
                
                with ui.row().classes('w-full justify-between items-center'):
                    ui.label("Trial ID:").classes('text-sm font-bold')
                    with ui.row().classes("ml-auto gap-2 items-center"):
                        ui.label().classes('text-sm font-mono w-32').bind_text_from(self.info, "trial_id")
                        ui.button(icon="refresh", on_click=self.generate_trial_id).classes('text-sm').props('flat round dense')
                        self.generate_trial_id()
                    
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Seed:").classes('text-sm font-bold')
                    with ui.row().classes("ml-auto gap-2 items-center"):
                        ui.number(
                            min=0,
                            max=10000,
                            step=1,
                            validation=lambda v: None if v is not None and 0 <= v <= 10000 else "Seed must be in [0, 10,000]"
                        ).classes('text-sm font-mono w-32').bind_value(self.info, "seed")
                        ui.button(icon="refresh", on_click=self.generate_seed).classes('text-sm').props('flat round dense')

                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Time limit:").classes('text-sm font-bold')
                    ui.number(
                        min=100,
                        step=1,
                        validation=lambda v: None if v is not None and v >= 100 else "Time limit must be >= 100 s"
                    ).classes('text-sm font-mono w-32 mr-10').bind_value(self.info, "time_limit")
                
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Kits:").classes('text-sm font-bold')
                    ui.number(
                        min=0,
                        step=1,
                        validation=lambda v: None if v is not None and v>=0 else "Number of kits must be >= 0"
                    ).classes('text-sm font-mono w-32 mr-10').bind_value(self.info, "num_kits")

                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Modules:").classes('text-sm font-bold')
                    ui.number(
                        min=0,
                        step=1,
                        validation=lambda v: None if v is not None and v>=0 else "Number of modules must be >= 0"
                    ).classes('text-sm font-mono w-32 mr-10').bind_value(self.info, "num_modules")

            ui.separator()

            with ui.expansion('Defects', group="trial_generation").classes('w-full items-center text-base'):
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Defect Rate:").classes('text-sm font-bold')
                    ui.knob(min=0, max=1, step=0.01, show_value=True, center_color="knob").bind_value(self.info, "defect_rate")
                
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Possible Defects:").classes('text-sm font-bold')
                    self.available_defects = self.get_defects()
                    self.defect_selection = ui.select(
                        {-1: "All"}|self.available_defects, 
                        multiple=True,
                        on_change=self.defect_selection_changed
                    ).classes('w-fit').props('use-chips').bind_value(self.info, "possible_defects")

            ui.separator()

            with ui.expansion('Challenges', group="trial_generation").classes('w-full items-center text-base'):

                with ui.row().classes('w-full items-center justify-center'):
                    self.add_button = ui.button('Add', on_click=self.add_challenge)
                    self.add_button.disable()

                    self.challenge_to_add = ui.select(
                        [c.capitalize() for c in self.challenge_types], 
                        on_change=lambda e: self.add_button.enable()
                    ).classes('w-60')
                
                self.challenges_grid = ui.aggrid(
                    {
                        'columnDefs': [
                            {'headerName': 'ID', 'field': 'id'},
                            {'headerName': 'Type', 'field': 'type'},
                            {'headerName': 'Occurrence', 'field': 'occurrence'}
                        ],
                        'rowData': [],
                        'rowSelection': 'single'
                    }
                ).style("max-height: 200px; overflow-y: auto;")

                self.challenges_grid.on('rowSelected', self.handle_selection)

                with ui.row().classes('w-full justify-center'):
                    self.edit_button = ui.button('Edit', on_click=self.edit_challenge)
                    self.remove_button = ui.button('Remove', on_click=self.remove_challenge)
                    self.edit_button.disable()
                    self.remove_button.disable()

            ui.button('Save', on_click=self.save_file)

            self.update_grid()

    async def handle_selection(self, e: events.GenericEventArguments) -> None:
        print("Inside handle selected")
        print(e.args['selected'])

        row = await self.challenges_grid.get_selected_row()
        if row is not None:
            print(row["id"], row["type"], row["occurrence"])

        if e.args['selected']:
            self.edit_button.enable()
            self.remove_button.enable()
        else:
            if await self.challenges_grid.get_selected_row() is None:
                self.edit_button.disable()
                self.remove_button.disable()
    
    async def edit_challenge(self):
        row = await self.challenges_grid.get_selected_row()

        print(row["id"], row["type"], row["occurrence"])

        if row is None:
            ui.notify("Unable to get selcted row")
            return
        
        print("Inside edit challenge. Looking for challenge with id "+ str(row["id"]))
        print("Challenges", self.info.challenges)
        
        challenge = copy(self.info.challenges[int(row["id"])])

        print("Got challenge: ", challenge)
        
        del self.info.challenges[int(row["id"])]
        await self.add_challenge(challenge)
    
    async def remove_challenge(self):
        row = await self.challenges_grid.get_selected_row()

        print(row["id"], row["type"], row["occurrence"])

        if row is None:
            ui.notify("Unable to get selcted row")
            return
        
        del self.info.challenges[int(row["id"])]
        self.update_grid()

    def defect_selection_changed(self, selection):
        if selection.value is None:
            return
        if -1 in selection.value:
            self.defect_selection.value = list(self.available_defects.keys())
    
    def generate_trial_id(self):
        self.info.trial_id = ''.join(random.choices(string.ascii_uppercase+string.digits, k=8))
    
    def generate_seed(self):
        self.info.seed = random.randint(0,10000)
    
    def update_grid(self):
        self.edit_button.disable()
        self.remove_button.disable()
        
        rows = []
        for i, c in enumerate(self.info.challenges):
            rows.append(
                {
                    'id': i,
                    'type': str(c.challenge_type),
                    'occurrence': f"Grasp occurrence {c.grasp_occurrence}" if c.challenge_type is ChallengeType.VacuumToolMalfunction else f"Start time {c.start_time}s"
                }
            )

        self.challenges_grid.options['rowData'] = rows

        self.challenges_grid.update()
    
    async def add_challenge(self, original_challenge: Challenge | None = None):
        print("Inside add challenge: ", original_challenge)
        if original_challenge is not None:
            match(self.challenge_to_add.value):
                case "Conveyor malfunction":
                    challenge = await AddConveyorMalfunction(original_challenge) # type: ignore
                case "Vacuum tool malfunction":
                    challenge = await AddVacuumToolMalfunction(original_challenge) # type: ignore
                case "Voltage tester malfunction":
                    challenge = await AddVoltageTesterMalfunction(original_challenge) # type: ignore
                case "High priority order":
                    challenge = await AddHighPriorityOrder(original_challenge) # type: ignore
                case _:
                    raise ValueError("Invalid challenge in add_challenge")
        
        if challenge is None:
            return
            
        if original_challenge is None:
            challenge.id = next(self.id_counter)
        else:
            challenge.id = original_challenge.id

        if original_challenge is not None and challenge is None:
            self.info.challenges.append(original_challenge) # Add back original sensor if edit fails
            ui.notify("Edit sensor failed", type='warning')
        elif challenge is None:
            ui.notify("Unable to add sensor", type='warning')
        else:
            self.info.challenges.append(challenge)  

        self.update_grid()
    
    async def save_file(self):
        if sum([self.info.num_kits, self.info.num_modules]) <= 0:
            ui.notify("Trial must have at least one kit or module", type="warning")
            return
        
        path = await FilePicker("/team_ws" if os.path.exists("/team_ws") else "~", selection_type="directory")

        if path is None:
            ui.notify("No folder selected")
            return

        yaml_path = os.path.join(path, f"{self.info.trial_id}.yaml")
        
        with open(yaml_path, "w") as f:
            f.write(Trial.contents(self.info))
            ui.notify(f"Trial written to file: {yaml_path}", type="info")
        
        self.submit((self.info, yaml_path))
    
    def get_defects(self) -> dict[int, str]:
        share = get_package_share_directory("ariac_setup")

        defects_path = os.path.join(share, "config", "defects.yaml")

        with open(defects_path, 'r') as file:
            try:
                defect_data = yaml.safe_load(file)
            except (ScannerError, ParserError) as e:
                raise ValidationError(f"{defects_path} is malformed {e.problem}")

        defects_dict = {}

        for defect_num, defect_info in defect_data["DEFECT_TYPES"].items():
            defects_dict[defect_num] = f"{defect_num} ({defect_info["DESCRIPTION"]})"
        
        return defects_dict


class AddConveyorMalfunction(ui.dialog):
    def __init__(self, challenge: ConveyorMalfunction | None = None):
        super().__init__()
        if challenge is None:
            self.start_time = 0
            self.duration = 10
        else:
            self.start_time = challenge.start_time
            self.duration = challenge.duration

        with self, ui.card():
            ui.label(f'Enter Conveyor Malfunction Info').classes('text-base')

            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Start Time").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_start_time,
                    precision=0
                ).classes('text-sm font-mono w-32').bind_value(self, "start_time")
            
            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Duration").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_duration,
                    precision=0
                ).classes('text-sm font-mono w-32').bind_value(self, "duration")
            
            with ui.row().classes('w-full justify-begin'):
                self.add_button = ui.button('Add', on_click=self._handle_ok)
        
    def validate_start_time(self, value):
        if value is None:
            return "Start time must not be blank"
        
        if not 0 <= value < 10000:
            return "Start time must be in [0, 10,000)"

        return None

    def validate_duration(self, value):
        if value is None:
            return "Duration must not be blank"
        
        if not 0 < value < 10000:
            return "Duration must be in (0, 10,000)"

        return None
    
    def on_change(self):
        if "add_button" not in vars(self):
            return
        if None in [self.start_time, self.duration]:
            self.add_button.disable()
        elif self.start_time >= 0 and self.duration > 0:
            self.add_button.enable()
        else:
            self.add_button.disable()

    def _handle_ok(self):
        self.submit(
            ConveyorMalfunction(
                ChallengeType.ConveyorMalfunction,
                int(self.start_time),
                int(self.duration)
            )
        )

class AddVacuumToolMalfunction(ui.dialog):
    def __init__(self, challenge: VacuumToolMalfunction | None = None):
        super().__init__()

        if challenge is None:
            self.tool = VacuumTools.VG_2
            self.grasp_occurrence = 5
        else:
            self.tool = challenge.tool
            self.grasp_occurrence = challenge.grasp_occurrence

        toggle_options = {
            VacuumTools.VG_2: "VG_2",
            VacuumTools.VG_4: "VG_4"
        }


        with self, ui.card():
            ui.label(f'Enter Vacuum Tool Malfunction Info').classes('w-full text-base')
            
            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Tool").classes('text-sm font-bold')
                ui.toggle(toggle_options).bind_value(self, "tool")
                
            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Grasp Occurrence").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_grasp_occurrence,
                    precision=0
                ).classes('text-sm font-mono w-32').bind_value(self, "grasp_occurrence")
            
            with ui.row().classes('w-full justify-begin'):
                self.add_button = ui.button('Add', on_click=self._handle_ok)
    
    def validate_grasp_occurrence(self, value):
        if value is None:
            return "Grasp occurrence must not be blank"
        
        if not 1 <= value <= 20:
            return "Grasp occurrence must be in [1, 20]"

        return None

    def on_change(self):
        if "add_button" not in vars(self):
            return
        if self.grasp_occurrence is None:
            self.add_button.disable()
        elif 1 <= self.grasp_occurrence <= 20:
            self.add_button.enable()
        else:
            self.add_button.disable()

    def _handle_ok(self):
        self.submit(
            VacuumToolMalfunction(
                ChallengeType.VacuumToolMalfunction,
                self.tool,
                int(self.grasp_occurrence)
            )
        )

class AddVoltageTesterMalfunction(ui.dialog):
    def __init__(self, challenge: VoltageTesterMalfunction | None = None):
        super().__init__()

        if challenge is None:
            self.start_time = 0
            self.duration = 10
            self.tester = 1
        else:
            self.start_time = challenge.start_time
            self.duration = challenge.duration
            self.tester = challenge.tester

        with self, ui.card():
            ui.label(f'Enter Voltage Tester Malfunction Info').classes('w-full text-base')
            
            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Start time").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_start_time,
                    precision=1
                ).classes('text-sm font-mono w-32').bind_value(self, "start_time")
                
            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Duration").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_duration,
                    precision=1
                ).classes('text-sm font-mono w-32').bind_value(self, "duration")

            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Tester").classes('text-sm font-bold')
                ui.toggle({i: f"Tester {i}" for i in range(1,3)}).bind_value(self, "tester")
            
            with ui.row().classes('w-full justify-begin'):
                self.add_button = ui.button('Add', on_click=self._handle_ok)

    def validate_start_time(self, value):
        if value is None:
            return "Start time must not be blank"
        
        if not 0 <= value < 10000:
            return "Start time must be in [0, 10,000)"

        return None

    def validate_duration(self, value):
        if value is None:
            return "Duration must not be blank"
        
        if not 0 < value < 10000:
            return "Duration must be in (0, 10,000)"

        return None
    
    def on_change(self):
        if "add_button" not in vars(self):
            return
        if None in [self.start_time, self.duration]:
            self.add_button.disable()
        elif self.start_time >= 0 and self.duration > 0:
            self.add_button.enable()
        else:
            self.add_button.disable()
    
    def _handle_ok(self):
        self.submit(
            VoltageTesterMalfunction(
                ChallengeType.VoltageTesterMalfunction,
                int(self.start_time),
                int(self.duration),
                self.tester
            )
        )

class AddHighPriorityOrder(ui.dialog):
    def __init__(self, challenge: HighPriorityOrder | None = None):
        super().__init__()

        if challenge is None:
            self.order_id = ""
            self.start_time = 0
        else:
            self.order_id = challenge.order_id
            self.start_time = challenge.start_time

        with self.classes('w-full'), ui.card():
            ui.label(f'Enter High Priority Order Info').classes('w-full text-base')

            with ui.row().classes('w-full items-center justify-between'):
                ui.label("Start time").classes('text-sm font-bold')
                ui.number(
                    on_change=self.on_change, 
                    validation=self.validate_start_time,
                    precision=1
                ).classes('text-sm font-mono w-32').bind_value(self, "start_time")
                
            with ui.row().classes('w-full justify-between items-center'):
                ui.label("Order ID").classes('text-sm font-bold')
                with ui.row().classes("ml-auto gap-2 items-center"):
                    ui.label().classes('text-sm font-mono w-32').bind_text(self, "order_id")
                    ui.button(icon="refresh", on_click=self._generate_order_id).classes('text-sm').props('flat round dense')

            with ui.row().classes('w-full justify-begin'):
                self.add_button = ui.button('Add', on_click=self._handle_ok)

                if self.order_id == "":
                    self.add_button.disable()

    def validate_start_time(self, value):
        if value is None:
            return "Start time must not be blank"
        
        if not 0 <= value < 10000:
            return "Start time must be in [0, 10,000)"

        return None

    def on_change(self):
        if "add_button" not in vars(self):
            return
        if None in [self.start_time, self.order_id]:
            self.add_button.disable()
        elif self.start_time >= 0 and self.order_id:
            self.add_button.enable()
        else:
            self.add_button.disable()
    
    def _generate_order_id(self):
        self.order_id = ''.join(random.choices(string.ascii_uppercase+string.digits, k=8))
        self.on_change()
    
    def _handle_ok(self):
        self.submit(
            HighPriorityOrder(
                ChallengeType.HighPriorityOrder,
                self.order_id,
                int(self.start_time)
            )
        )