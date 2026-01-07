from nicegui import ui


class Warning(ui.dialog):

    def __init__(self, text: str):

        super().__init__()

        with self, ui.card().classes('items-center'):
            ui.label(text).classes('text-base text-red-500')
            with ui.row().classes("items-center w-full justify-center"):
                ui.button('OK', on_click=lambda: self.submit(True))