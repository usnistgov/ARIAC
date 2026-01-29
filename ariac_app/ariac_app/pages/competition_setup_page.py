from pathlib import Path
from dataclasses import dataclass

from nicegui import ui, app
from jsonschema import ValidationError

from ariac_app.theme import frame
from ariac_app.dialogs.file_picker import FilePicker
from ariac_app.app_utils import is_gazebo_running
from ariac_app.pages.home_page import DatabaseSelect

from ariac_setup.yaml_validation import TrialConfigValidator

@dataclass
class TrialPath:
    path: Path
    file: str
    selected: bool = False

    def valid_trial(self) -> bool:
        val = TrialConfigValidator()

        try:
            val.validate_yaml(str(self.path))
            return True
        except ValidationError as e:
            return False
        

@ui.page("/competition_setup")
class CompetitionRunSetupPage:
    def __init__(self):
        self.available_paths = []

        self.trials_selection = TrialsSelectionFrame()
        self.db_select = DatabaseSelect()

        self.successful_runs_per_trial = 5
        self.max_runs_per_trial = 10
        self.runs_to_score = 2
        
        self.headless = "False"
        self.record = "True"
        self.save_unscored_videos = "False"

        with frame(page_name="competition Setup"):
            with ui.card().classes("w-5/6 items-center max-w-xl"):

                self.trials_selection.content()
                ui.separator()

                self.db_select.content()
                ui.separator()

                with ui.row().classes('w-full items-center'):
                    ui.number(label='Successful runs per trial').classes('w-40').bind_value(self, "successful_runs_per_trial")
                    ui.slider(min=1, max=20, step=1).classes('w-64').bind_value(self, "successful_runs_per_trial")

                with ui.row().classes('w-full items-center'):
                    ui.number(label='Maximum runs per trial').classes('w-40').bind_value(self, "max_runs_per_trial")
                    ui.slider(min=1, max=20, step=1).classes('w-64').bind_value(self, "max_runs_per_trial")

                with ui.row().classes('w-full items-center'):
                    ui.number(label='# Runs to score').classes('w-40').bind_value(self, "runs_to_score")
                    ui.slider(min=1, max=20, step=1).classes('w-64').bind_value(self, "runs_to_score")

                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Headless").classes('text-sm font-bold')
                    ui.toggle(["True", "False"]).bind_value(self, "headless")
                
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Record").classes('text-sm font-bold')
                    ui.toggle(["True", "False"]).bind_value(self, "record")
                
                with ui.row().classes('w-full items-center justify-between'):
                    ui.label("Save Unscored Videos").classes('text-sm font-bold')
                    ui.toggle(["True", "False"]).bind_value(self, "save_unscored_videos")

                ui.button(
                    "Run", icon="chevron_right", on_click=self.run
                ).props("glossy").tooltip("Confirm settings for run")

    def run(self):
        if is_gazebo_running():
            ui.notify("Gazebo is already open. Cannot start running trials", type="warning")
            return
        
        selected_trial_paths = [trial_path.path for trial_path in self.trials_selection.available_paths if trial_path.selected]

        if len(selected_trial_paths) == 0:
            ui.notify("No trials selected. Cannot start running trials", type="warning")
            return
        
        target = f"/competition?successful_runs={self.successful_runs_per_trial}&max_runs={self.max_runs_per_trial}&runs_to_score={self.runs_to_score}"
        
        target += "&trials=" + ",".join([str(path) for path in selected_trial_paths])

        target += f"&headless={self.headless}"

        target += f"&record={self.record}"

        target += f"&save_unscored_videos={self.save_unscored_videos}"

        if self.db_select.path is not None:
            target += f"&db_path={self.db_select.path}"
        ui.navigate.to(target)

class TrialsSelectionFrame:
    def __init__(self):
        self.available_paths: list[TrialPath] = []
        self.dir_path: Path | None = None
        self.column_elements: list[ui.element] = []

    def content(self):
        ui.label("Trial Selection").classes("text-xl")
        with ui.row().classes("items-center"):
            ui.button("Select", icon="folder", on_click=self._select_dir)

            with ui.column().classes("items-center h-48 overflow-y-auto w-64") as self.chip_column:
                ui.label("Available Trials").classes("text-lg")
                ui.separator()
                self.none_label = ui.label("None").classes("text-md")
                self.column_elements.append(self.none_label)
    
    async def _select_dir(self):
        path = await FilePicker("~", selection_type="directory")

        if path is None:
            ui.notify("No folder selected")
            return
        path: Path

        if len(list(path.glob("*.yaml")))==0:
            ui.notify(
                f"No yaml files found in directory {str(path)}. Aborting.",
                type="warning",
            )
            return
        
        valid_trial_paths = []
        for p in path.glob("*.yaml"):
            trial_path = TrialPath(p, p.name)
            if trial_path.valid_trial():
                valid_trial_paths.append(trial_path)
        
        if len(valid_trial_paths) > 0:
            self.available_paths.clear()
            self.available_paths = valid_trial_paths
            self.dir_path = path
            app.storage.general["trial_dir_path"] = str(path)
            await self.update_options()
        else:
            ui.notify(
                f"No valid trial files found in directory {str(path)}. Aborting.",
                type="warning",
            )
            return

    def clear_column(self):
        for element in self.column_elements:
            element.delete()
        self.column_elements.clear()
    
    async def update_options(self):
        self.clear_column()
        with self.chip_column:
            for trial_path in self.available_paths:
                self.column_elements.append(TrialChip(trial_path))
            
class TrialChip(ui.chip):
    def __init__(self, trial_path: TrialPath):
        super().__init__(trial_path.file, on_click=self.clicked)
        self.props("square outline color=grey")
        self.trial_path = trial_path
    
    def clicked(self):
        if self.trial_path.selected:
            self.trial_path.selected = False
            self.props("color=grey")
        else:
            self.trial_path.selected = True
            self.props("color=green")