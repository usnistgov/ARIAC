import os
from typing import Literal
from pathlib import Path

import asyncio

from nicegui import ui, events, app

from ament_index_python.packages import get_packages_with_prefixes, get_package_prefix, get_package_share_directory

from ariac_interfaces.msg import CompetitionStates

from ariac_app.theme import frame
from ariac_app.components.process_manager import ProcessManager
from ariac_app.dialogs.confirmation import Confirmation
from ariac_app import ros_globals, app_utils


@ui.page('/run')
class RunPage:
    def __init__(
            self, 
            trial: str | None = None, 
            user_config: str | None = None, 
            headless: str | None = None,
            record: str | None = None,
            db_path: str | None = None, 
            cheat: int | None = None
        ):
        if trial is None or user_config is None:
            print("Trial and user config must be selected to run a trial")
            ui.navigate.to("/")
            return
        
        if app_utils.is_gazebo_running():
            print("Gazebo is already running")
            ui.navigate.to("/")
            return
            
        self.cmd = f"ros2 launch ariac_gz ariac.launch.py trial_config:={trial} user_config:={user_config} gz_log_level:=info headless:={headless} record:={record}"
        
        if db_path is not None:
            self.cmd += f" db_path:={db_path}"

        if cheat is not None:
            self.cmd += f" cheat_selection:={cheat}"
        
        self.db_path = db_path

        self.status_display = StatusDisplay()
        self.control_panel = ControlPanel()
        self.command_select = UserCommandSelect()
        self.run_log: ProcessManager | None = None

        self.recorded = False
        if record is not None:
            self.recorded = record.lower() == "true"

        self.content()

        ui.context.client.on_disconnect(self._handle_disconnect)

        self.timer = ui.timer(1.0, self.check_process)

    def content(self):
        with frame(page_name='Run', show_menu=False):
            with ui.column().classes("w-5/6 max-w-3xl"):
                with ui.card().classes("w-full items-center"):
                    self.status_display.content()
                with ui.card().classes("w-full items-center"):
                    self.command_select.content()

            self.run_button = ui.button("Start Run", on_click=self.manage_run, color="green", icon="play_circle").classes('text-lg')
    
    async def manage_run(self):
        if self.run_log is None:
            if getattr(ros_globals, 'shutting_down', False):
                ui.notify('Shutdown in progress, not starting run', type='warning')
                return

            self.run_log = ProcessManager(self.cmd)
            self.run_button.text = "End Run"
            self.run_button.props('color=red icon=dangerous')
            return
        
        self.run_button.disable()
        self.run_button.props('loading')

        if self.run_log is None:
            await self._exit()
            return
        
        if self.run_log.is_running and not await Confirmation("Do you want to stop the current run?"):
            self.run_button.enable()
            self.run_button.props(remove='loading')
            return
        
        self.timer.deactivate()
        await self._exit()
        
    async def _exit(self):
        node = ros_globals.node

        if node is None:
            return

        if self.db_path is not None and node.run_id is not None:
            results_target = f"/run_results?run_id={node.run_id}&db_path={self.db_path}"
        else:
            results_target = None

        if node.current_state == CompetitionStates.STARTED:
            await node.end_competition(shutdown_gazebo=False)
            
        await self._stop_processes()     

        self.run_button.enable()
        self.run_button.props(remove='loading')
        
        if results_target and await Confirmation("View Results?", decline_button_lable='Go Home'):
            ui.navigate.to(results_target)
            return
        
        ui.navigate.to('/')

    async def _stop_processes(self):
        tasks = []

        if self.run_log and self.run_log.is_running:
            tasks.append(self.run_log.close())

        if self.command_select.process is not None and self.command_select.process.is_running:
            tasks.append(self.command_select.process.close())

        if tasks:
            await asyncio.gather(*tasks)

        if app_utils.is_gazebo_running():
            print("Gazebo is still running, killing process")
            app_utils.kill_gazebo()
        
        if self.recorded:
            for recorder in ["inspection", "assembly", "environment"]:
                if ros_globals.node:
                    ros_globals.node.get_logger().info(f"{recorder.upper()} video can be found at /tmp/{recorder}_recorder_run_id_{ros_globals.node.run_id}.mp4")

        node = ros_globals.node

        if node is not None:
            node.reset()

    async def check_process(self):
        if not self.run_log:
            return
        
        if not self.run_log.is_running and ros_globals.node is not None and ros_globals.node.current_state != CompetitionStates.ENDED:
            ui.notify("Gazebo shutdown before competition ended. Check logs.", type="warning")
        
            self.timer.deactivate()
            # await self._exit()
    
    async def _handle_disconnect(self):
        if not self.run_log:
            return
        
        if self.run_log.is_running or (self.command_select.process is not None and self.command_select.process.is_running):
            print('Client disconnected before all processes were ended. Stopping all processes...')
            await self._stop_processes()

class StatusDisplay:
    CHIP_OFF_PROPS = "color=light-gray text-color=black"

    def __init__(self):
        self.state_info = {
            CompetitionStates.PREPARING: {
                'label': 'Preparing',
                'on_props': 'color=yellow-500 text-color=white',
                'icon': 'build'
            },
            CompetitionStates.READY: {
                'label': 'Ready',
                'on_props': 'color=blue-600 text-color=white',
                'icon': 'check'
            },
            CompetitionStates.STARTED: {
                'label': 'Started',
                'on_props': 'color=green-600 text-color=white',
                'icon': 'play_circle'
            },
            CompetitionStates.ORDERS_COMPLETE: {
                'label': 'Complete',
                'on_props': 'color=purple-400 text-color=white',
                'icon': 'checklist'
            },
            CompetitionStates.ENDED: {
                'label': 'Ended',
                'on_props': 'color=red-600 text-color=white',
                'icon': 'stop_circle'
            }
        }

        self.chips: dict[int, ui.chip] = {}
        self.labels: dict[str, ui.label] = {}

        self.kit_progress = 0
        self.module_progress = 0

        self.kit_progress_component = None
        self.module_progress_component = None

    def content(self):
        with ui.row().classes("w-full justify-start"):
            ui.label("Competition Status").classes("text-sm text-gray-500")
        with ui.row().classes("w-full justify-center items-center"):

            for state, info in self.state_info.items():
                self.chips[state] = ui.chip(info['label'], icon=info['icon']).props(self.CHIP_OFF_PROPS)

        with ui.row().classes("w-full justify-center items-center"):
            
            with ui.column().classes("w-auto justify-center items-center"):
                ui.label("Kits").props('text-sm')
                self.kit_progress_component = ui.circular_progress(size="xl", show_value=False).bind_value(self, "kit_progress")
                with self.kit_progress_component:
                    self.kitting_progress_label = ui.label("0/0").props('text-sm')  
            
            with ui.column().classes("w-auto justify-center items-center"):
                ui.label("Modules").props('text-sm')
                self.module_progress_component = ui.circular_progress(size="xl", show_value=False).bind_value(self, "module_progress")
                with self.module_progress_component:
                    self.module_progress_label = ui.label("0/0").props('text-sm')    
        
        with ui.row().classes("w-full justify-center items-center"):
            self.time_progress = ui.linear_progress(show_value=False).classes("w-80")
            self.labels["time_label"] = ui.label('00:00 / 00:00').classes('text-base')

        ui.timer(0.1, self.update)

    def update(self):
        node = ros_globals.node
        
        if node is None:
            return  # node not ready yet

        for state, chip in self.chips.items():
            if node.current_state == state:
                chip.props(self.state_info[state]['on_props'])
            else:
                chip.props(self.CHIP_OFF_PROPS)
        
        if node.total_kits and node.kits_remaining:
            self.kit_progress = (node.total_kits - node.kits_remaining) / node.total_kits
            self.kitting_progress_label.text = f"{node.total_kits - node.kits_remaining}/{node.total_kits}"
        
        if node.total_modules and node.modules_remaining:
            self.module_progress = (node.total_modules - node.modules_remaining) / node.total_modules
            self.module_progress_label.text = f"{node.total_modules - node.modules_remaining}/{node.total_modules}"
        
        if node.time_elapsed is not None and node.time_remaining is not None:
            elapsed = node.time_elapsed
            remaining = node.time_remaining
            total = elapsed + remaining
            progress = elapsed / total if total > 0 else 0
            self.time_progress.value = progress
            self.labels["time_label"].text = f"{self.format_time(elapsed)} / {self.format_time(total)}"

    def format_time(self, seconds: float) -> str:
        m, s = divmod(int(seconds), 60)
        return f"{m:02}:{s:02}"
        
class ControlPanel:
    def __init__(self):
        self.start_button: ui.button

    def content(self):
        with ui.row().classes("w-full justify-start"):
            ui.label("Competition Controls").classes("text-sm text-gray-500")
        with ui.row().classes('w-full justify-center'):
            self.start_button = ui.button('Start', icon='play_arrow', on_click=self._start_competition).props('rounded outline')
            self.end_button = ui.button('End', icon='stop', on_click=self._end_competition).props('rounded outline')

    async def _start_competition(self):
        self.start_button.props('loading')
        self.start_button.disable()

        node = ros_globals.node
        
        if node is None:
            ui.notify('ROS node not ready')
            self.start_button.props(remove='loading')
            self.start_button.enable()
            
            return

        success, message = await node.start_competition()

        self.start_button.props(remove='loading')
        self.start_button.enable()

        ui.notify(message, type='positive' if success else 'negative')

    async def _end_competition(self):
        self.end_button.props('loading')
        self.end_button.disable()

        node = ros_globals.node
        
        if node is None:
            ui.notify('ROS node not ready')
            self.end_button.props(remove='loading')
            self.end_button.enable()
            
            return

        success, message = await node.end_competition()

        self.end_button.props(remove='loading')
        self.end_button.enable()

        ui.notify(message, type='positive' if success else 'negative')

class UserCommandSelect:
    ARIAC_PACKAGES = [
        "ariac_app",
        "ariac_db",
        "ariac_description",
        "ariac_gz",
        "ariac_interfaces",
        "ariac_plugins",
        "ariac_setup",
        "robotiq_gripper_controller"
    ]
    
    def __init__(self):
        self.packages = self.get_package_names()

        self.validate_team_storage_values()
        self.command_type: Literal["run", "launch"] = app.storage.general.get("team_command_type", "run")
        self.selected_package = app.storage.general.get("selected_team_package", "")
        self.selected_file = app.storage.general.get("selected_team_file", "")

        self.command = "No command"

        self.file_selection: ui.select | None = None
        self.button: ui.button
        self.container: ui.column
    
        self.process: ProcessManager | None = None
    
    def content(self):
        with ui.row().classes("w-full justify-start"):
            ui.label("Team Process").classes("text-sm text-gray-500")
        with ui.column().classes('w-full items-center'):
            ui.toggle(
                ["run", "launch"],
                on_change=self.update_package_select
            ).classes("text-sm").props("spread no-caps").bind_value(self, 'command_type')

            ui.select(
                self.packages, 
                label="Package", 
                on_change=self.update_package_select
            ).classes("w-64").bind_value(self, "selected_package")

            available_options = self.get_available_options()
            self.file_selection = ui.select(
                available_options,
                label="File",
                on_change=self.exe_selected,
            ).classes("w-64").bind_value(self, "selected_file")
            self.file_selection.set_options(available_options)

            with ui.row().classes("w-full items-center justify-center"):
                ui.label().classes("text-sm font-bold").bind_text(self, "command")
                self.button = ui.button(text="", icon="arrow_forward", on_click=self.start_stop_process).props("flat round dense")
                self.button.tooltip("Run this ros command")

    async def start_stop_process(self):
        if self.process is None:
            if not self.command:
                ui.notify("Command is None", type="negative")
                return 
            if getattr(ros_globals, 'shutting_down', False):
                ui.notify('Shutdown in progress, not starting process', type='warning')
                return

            self.process = ProcessManager(self.command)

            self.button.props('icon=cancel')

        else:
            self.button.props(add='loading')

            tasks = []

            if self.process and self.process.is_running:
                tasks.append(self.process.close())

            if tasks:
                await asyncio.gather(*tasks)

            self.process = None

            self.button.props(remove='loading')

            self.button.props('icon=arrow_forward')

    def get_package_names(self) -> list[str]:
        return [p for p, pre in get_packages_with_prefixes().items() if pre!="/opt/ros/jazzy" and p not in UserCommandSelect.ARIAC_PACKAGES and "moveit_config" not in p]

    def update_package_select(self):        
        app.storage.general["team_command_type"] = self.command_type
        app.storage.general["selected_team_package"] = self.selected_package

        if self.file_selection is None:
            return

        if not self.selected_package:
            self.file_selection.set_options([])
            self.selected_file = ""
            return

        match self.command_type:
            case 'run':
                dir = Path(get_package_prefix(self.selected_package)).joinpath('lib', self.selected_package)
            case 'launch':
                dir = Path(get_package_share_directory(self.selected_package), "launch")

        options = []

        if dir.exists():        
            options = [f.name for f in dir.iterdir() if os.access(f, os.X_OK) and f.is_file()]

        self.file_selection.set_options(options)
    
    def exe_selected(self):
        app.storage.general["selected_team_file"] = self.selected_file
        if self.selected_file and self.selected_package:
            self.command = f"ros2 {self.command_type} {self.selected_package} {self.selected_file}"
        else:
            self.command = "No command"

    def reset_team_storage_values(self):
        app.storage.general["team_command_type"] = "run"
        app.storage.general["selected_team_package"] = ""
        app.storage.general["selected_team_file"] = ""

    def validate_team_storage_values(self):
        if (command_type:=app.storage.general.get("team_command_type", None)) is None or \
            None in [app.storage.general.get(var, None) for var in ["selected_team_package", "selected_team_file"]]:
            self.reset_team_storage_values()
            return
        
        if app.storage.general["selected_team_package"] not in self.packages:
            self.reset_team_storage_values()
            return
        
        match command_type:
            case 'run':
                dir = Path(get_package_prefix(app.storage.general["selected_team_package"])).joinpath('lib', app.storage.general["selected_team_package"])
            case 'launch':
                dir = Path(get_package_share_directory(app.storage.general["selected_team_package"]), "launch")
        
        if not dir.exists():
            self.reset_team_storage_values()
            return
        
        if app.storage.general["selected_team_file"] not in [f.name for f in dir.iterdir() if os.access(f, os.X_OK) and f.is_file()]:
            self.reset_team_storage_values()
            return
        
    def get_available_options(self) -> list[str]:
        if app.storage.general["selected_team_package"] == "":
            return []
        
        match self.command_type:
            case 'run':
                dir = Path(get_package_prefix(app.storage.general["selected_team_package"])).joinpath('lib', app.storage.general["selected_team_package"])
            case 'launch':
                dir = Path(get_package_share_directory(app.storage.general["selected_team_package"]), "launch")
            
        if dir.exists():
            return [f.name for f in dir.iterdir() if os.access(f, os.X_OK) and f.is_file()]
        return []