from pathlib import Path
from typing import Optional, Literal

from nicegui import events, ui


class FilePicker(ui.dialog):

    def __init__(self, directory: str, extension: Optional[str] = None, selection_type: Literal["file", "directory"] = "file"):

        super().__init__()

        self.path = Path(directory).expanduser()
        self.selected_file: str | None = None

        self.upper_limit = Path('/')

        self.selection_type = selection_type

        self.extension = extension

        with self, ui.card().classes('w-3/4 max-w-lg'):
            with ui.row().classes('w-full items-center justify-between'):
                with ui.row().classes('w-full justify-end'):
                    ui.button(icon='home', on_click=self.go_home)
                    ui.button(icon='subdirectory_arrow_left', on_click=self.go_up)
                with ui.row().classes('w-full justify-begin'):
                    ui.label().bind_text_from(self, "label_path").classes('text-xs font-mono')

            self.grid = ui.aggrid(
                {
                    'columnDefs': [
                        {'field': 'name', 'headerName': 'File', 'width': 350}
                    ],
                    'defaultColDef': {'resizable': False},
                    'rowSelection': 'single'
                },
                html_columns=[0]
            ).classes('w-full')
            
            self.grid.on('cellDoubleClicked', self.handle_double_click)
            self.grid.on('cellClicked', self.handle_selection)
            
            with ui.row().classes('w-full justify-center'):
                ui.button('Cancel', on_click=self.close).props('outline')
                self.ok_button = ui.button('Ok', on_click=self._handle_ok)
        
        self.update_grid()

    @property
    def label_path(self):
        if self.selected_file:
            return str(self.path.joinpath(self.selected_file))
        return str(self.path)

    def go_home(self):
        self.path = Path("/team_ws") if Path("/team_ws").exists() else Path.home()
        self.update_grid()

    def go_up(self):
        if self.path != self.path.parent and self.path != self.upper_limit:
            self.path = self.path.parent
            self.update_grid()

    def update_grid(self) -> None:
        if self.extension is not None:
            self.ok_button.disable()

        paths = list(self.path.glob('*'))
        paths = [p for p in paths if not p.name.startswith('.')]

        if self.extension is not None:
            paths = [p for p in paths if p.is_dir() or p.suffix == self.extension]

        paths.sort(key=lambda p: p.name.lower())
        paths.sort(key=lambda p: not p.is_dir())

        self.grid.options['rowData'] = [
            {
                'name': f'📁 <strong>{p.name}</strong>' if p.is_dir() else p.name,
                'path': str(p),
            }
            for p in paths
        ]

        self.grid.update()

    def handle_double_click(self, e: events.GenericEventArguments) -> None:
        if Path(e.args['data']['path']).is_dir():
            self.path = Path(e.args['data']['path'])
            self.update_grid()

    def handle_selection(self, e: events.GenericEventArguments) -> None:
        path = Path(e.args['data']['path'])
        self.selected_file = None

        if path.is_file() and self.selection_type == "file":
            if e.args["selected"]:
                self.ok_button.enable()
                self.selected_file = path.name
            else:
                self.ok_button.disable()
                
        elif path.is_dir() and self.selection_type == "directory":
            self.ok_button.enable()

    async def _handle_ok(self):
        row = await self.grid.get_selected_row()
        if row is not None:
            self.submit(Path(row['path']))
        else:
            self.submit(self.path)