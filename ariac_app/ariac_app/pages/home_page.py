from pathlib import Path

from nicegui import ui, app
from jsonschema import ValidationError
from yaml.constructor import ConstructorError

from ariac_app.theme import frame
from ariac_app.dialogs.user_config import UserConfigBuilder
from ariac_app.dialogs.trial_config import TrialConfigBuilder
from ariac_app.dialogs.file_picker import FilePicker
from ariac_app.structures import Trial, UserInfo
from ariac_app.app_utils import is_gazebo_running

from ariac_setup.yaml_validation import TrialConfigValidator, UserConfigValidator

from ariac_setup.user_config_parser import UserConfigParser
from ariac_setup.structures import Cheats

from ariac_db.manager import DatabaseError, DatabaseManager

CHEAT_OPTIONS = {
    Cheats.CELLS_IN_VOLTAGE_TESTERS: "Cells in voltage testers",
    Cheats.KIT_ON_AGV: "Kit on AGV",
    Cheats.KITS_ON_AGVS: "Kits on AGVs",
    Cheats.HIGH_PRIORITY_KIT: "High priority kit on AGV",
    Cheats.PARTIAL_MODULE: "Partial module",
    Cheats.MODULE: "Module",
    Cheats.FLIPPED_MODULE: "Flipped module",
    Cheats.MODULE_WITH_WELDS: "Module with welds",
}


@ui.page("/")
class HomePage:
    def __init__(self):
        self.trial_select = TrialSelect()
        self.user_select = UserSelect()
        self.db_select = DatabaseSelect()

        with frame(page_name="Home"):
            with ui.card().classes("w-5/6 items-center max-w-xl"):
                self.trial_select.content()
                ui.separator()

                self.user_select.content()
                ui.separator()

                self.db_select.content()
                ui.separator()

                ui.label("Cheat Selection").classes("text-xl").tooltip(
                    "Start the environment with options for easier testing"
                )
                with ui.row():
                    self.cheat_select = ui.select(
                        CHEAT_OPTIONS,
                        clearable=True,
                    )
                ui.separator()

                ui.button(
                    "Confirm", icon="chevron_right", on_click=self.run
                ).props("glossy").tooltip("Confirm settings for run")

    def run(self):
        if is_gazebo_running():
            ui.notify("Gazebo is already open. Cannot start trial", type="warning")
            return

        if self.trial_select.chip.enabled and self.user_select.chip.enabled:
            ui.notify(
                f"Starting... trial: {self.trial_select.path}, user: {self.user_select.path}:",
                type="positive",
            )

            if self.db_select.chip.enabled:
                ui.notify(f"Running with db: {self.db_select.path}", type="info")

            target = f"/run?trial={self.trial_select.path}&user_config={self.user_select.path}"

            if self.db_select.path is not None:
                target += f"&db_path={self.db_select.path}"

            if self.cheat_select.value:
                target += f"&cheat={self.cheat_select.value}"

            ui.navigate.to(target)

        else:
            ui.notify("Please select a user and trial ", type="warning")


class SelectFrame:
    def __init__(self, title):
        self.title = title
        self.chip_text = "None selected"
        self.chip: ui.chip
        self.edit_button: ui.button

        self._path: Path | None = None

    def content(self):
        ui.label(self.title).classes("text-xl")
        with ui.row():
            ui.button("Select", icon="folder", on_click=self._select)
            ui.button("Create", icon="add", on_click=self._create)

        with ui.row().classes("items-center"):
            self.chip = (
                ui.chip(color="gray", on_click=self.chip_off)
                .props("square outline")
                .bind_text(self, "chip_text")
            )
            self.edit_button = ui.button(icon="edit", on_click=self._edit).props(
                "flat round dense"
            )

            if self.path is not None:
                self.chip_on()
            else:
                self.chip.disable()
                self.edit_button.disable()

    @property
    def path(self):
        return self._path

    async def _create(self):
        raise NotImplementedError

    async def _select(self):
        raise NotImplementedError

    async def _edit(self):
        raise NotImplementedError

    def chip_on(self):
        self.chip.props("color=green")
        self.chip.enable()
        self.edit_button.enable()

    def chip_off(self):
        self._path = None
        self.chip.props("color=gray")
        self.chip.disable()
        self.edit_button.disable()


class TrialSelect(SelectFrame):
    def __init__(self):
        super().__init__("Trial")

        self.validator = TrialConfigValidator()

        path = app.storage.general.get("trial_path", None)

        self.trial = Trial(path)

        if isinstance(path, str):
            self._path = Path(path)

    def chip_on(self):
        self.chip_text = f"Trial id: {self.trial.info.trial_id}"
        self._path = self.trial.path
        app.storage.general["trial_path"] = str(self._path)

        super().chip_on()

    def chip_off(self):
        self.chip_text = "None Selected"
        app.storage.general["trial_path"] = None

        super().chip_off()

    async def _create(self):
        result = await TrialConfigBuilder(Trial(None).info)

        if result is None:
            ui.notify("Trial creation cancelled")
            return

        _, path = result

        try:
            self.validator.validate_yaml(path)
        except ValidationError as e:
            ui.notify(f"Error writing to yaml. MSG: {e.message}", type="negative")
            Path(path).unlink()
            return

        self.trial.info, self.trial.path = result

        self.chip_on()

    async def _select(self):
        path = await FilePicker("~", extension=".yaml")

        if path is None:
            ui.notify("Trial selection cancelled")
            return

        try:
            self.validator.validate_yaml(path)
        except ValidationError as e:
            ui.html(
                "<style>.multi-line-notification { white-space: pre-line; }</style>"
            )
            ui.notify(
                f"Validation error: \n{e.message}",
                type="negative",
                multi_line=True,
                classes="multi-line-notification",
            )
            return
        except ConstructorError as e:
            ui.html(
                "<style>.multi-line-notification { white-space: pre-line; }</style>"
            )
            ui.notify(
                f"Validation error: \n{e}",
                type="negative",
                multi_line=True,
                classes="multi-line-notification",
            )
            return

        self.trial = Trial(path)

        if self.trial.path is None:
            return

        self.chip_on()

    async def _edit(self):
        result = await TrialConfigBuilder(self.trial.info)

        if result is None:
            ui.notify("Trial editing cancelled")
            return

        self.trial.info, self.trial.path = result

        self.chip_on()


class UserSelect(SelectFrame):
    def __init__(self):
        super().__init__("Team Config")

        self.validator = UserConfigValidator()

        path = app.storage.general.get("user_path", None)

        if isinstance(path, str):
            self._path = Path(path)
            try:
                parser = UserConfigParser(path)

                self.info: UserInfo = UserInfo(
                    name=parser.competitor_name,
                    conveyor_speed=parser._conveyor_speed,
                    cell_feed_rate=parser.cell_feed_rate,
                    sensors=parser.sensors,
                )

                self.chip_text = f"Team name: {self.info.name}"
                return
            except:
                pass

        self.info: UserInfo = UserInfo(
            name="", conveyor_speed=0.05, cell_feed_rate=0.05, sensors=[]
        )

    def chip_on(self):
        self.chip_text = f"Team name: {self.info.name}"
        app.storage.general["user_path"] = str(self._path)

        super().chip_on()

    def chip_off(self):
        self.chip_text = "None Selected"
        app.storage.general["user_path"] = None

        super().chip_off()

    async def _create(self):
        result = await UserConfigBuilder(self.info)

        if result is None:
            ui.notify("User config creation cancelled")
            return

        self.info, self._path = result

        self.chip_on()

    async def _select(self):
        path = await FilePicker("~", extension=".yaml")

        if path is None:
            ui.notify("Trial selection cancelled")
            return

        try:
            self.validator.validate_yaml(path)
        except ValidationError as e:
            ui.html(
                "<style>.multi-line-notification { white-space: pre-line; }</style>"
            )
            ui.notify(
                f"Validation error: \n{e.message}",
                type="negative",
                multi_line=True,
                classes="multi-line-notification",
            )
            return
        except ConstructorError as e:
            ui.html(
                "<style>.multi-line-notification { white-space: pre-line; }</style>"
            )
            ui.notify(
                f"Validation error: \n{e}",
                type="negative",
                multi_line=True,
                classes="multi-line-notification",
            )
            return

        parser = UserConfigParser(path)
        self.info = UserInfo(
            parser.competitor_name,
            parser.conveyor_speed,
            parser.cell_feed_rate,
            parser.sensors,
        )

        self._path = path

        self.chip_on()

    async def _edit(self):
        result = await UserConfigBuilder(self.info)

        if result is None:
            ui.notify("Trial editing cancelled")
            return

        self.info, self._path = result

        self.chip_on()


class DatabaseSelect:
    def __init__(self):
        self.db_manager: DatabaseManager | None = None

        self.chip_text = "Disconnected"

        path = app.storage.general.get("db_path", None)

        if path is not None:
            try:
                self.db_manager = DatabaseManager(Path(path))
            except DatabaseError as e:
                del app.storage.general["db_path"]
                ui.notify(f"Unable to open Database. Error: {e}", type="negative")

    @property
    def path(self) -> Path | None:
        if self.db_manager is None:
            return None
        return self.db_manager.db_path

    def content(self):
        ui.label("Database").classes("text-xl")
        with ui.row():
            ui.button("Select", icon="folder", on_click=self._select)
            ui.button("Create", icon="add", on_click=self._create)
            # ui.button('Results', icon="bar_chart", on_click=self._go_to_results_page)

        with ui.row().classes("items-center gap-0"):
            self.chip = (
                ui.chip(icon="block", color="gray", on_click=self._chip_off)
                .props("square outline")
                .bind_text(self, "chip_text")
            )
            if self.db_manager is None:
                self.chip.disable()
            else:
                self._chip_on()

    def _go_to_results_page(self):
        if self.path is None:
            ui.notify("A database must be selected first", type="negative")
            return
        location = f"/results?db_path={str(self.path)}"
        ui.navigate.to(location)

    def _chip_on(self):
        self.chip_text = "Connected"
        self.chip.props("color=green")
        self.chip.set_icon("check")
        self.chip.enable()

    def _chip_off(self):
        self.db_manager = None

        app.storage.general["db_path"] = None

        self.chip_text = "Disconnected"
        self.chip.props("color=gray")
        self.chip.set_icon("block")
        self.chip.disable()

    async def _create(self):
        path = await FilePicker("~", selection_type="directory")

        if path is None:
            ui.notify("No folder selected")
            return
        path: Path

        if path.joinpath("ariac.db").exists():
            ui.notify(
                f"A database already exists at {str(path.joinpath("ariac.db"))}. Aborting.",
                type="warning",
            )
            return

        try:
            self.db_manager = DatabaseManager(path.joinpath("ariac.db"), create=True)
        except DatabaseError as e:
            ui.notify(f"Error: {e.message}", type="negative")
            return

        app.storage.general["db_path"] = str(path)

        ui.notify(f"Database created: {self.db_manager.db_path}")

        self._chip_on()

    async def _select(self):
        path = await FilePicker("~", extension=".db")

        if path is None:
            ui.notify("Database selection cancelled")
            return

        try:
            self.db_manager = DatabaseManager(path)
        except DatabaseError as e:
            ui.notify(f"Error: {e.message}", type="negative")
            return

        app.storage.general["db_path"] = str(path)

        ui.notify(f"Database selected: {self.db_manager.db_path}")

        self._chip_on()
