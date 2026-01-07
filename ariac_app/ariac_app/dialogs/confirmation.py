from nicegui import ui


class Confirmation(ui.dialog):

    def __init__(self, question: str, confirm_button_lable: str = 'Yes', decline_button_lable: str = 'Cancel'):

        super().__init__()

        with self, ui.card().classes('items-center'):
            ui.label(question).classes('text-lg')
            with ui.row().classes("items-center w-full justify-center"):
                ui.button(confirm_button_lable, on_click=lambda: self.submit(True))
                ui.button(decline_button_lable, on_click=lambda: self.submit(False)).props('outline')