from contextlib import contextmanager
from nicegui import ui


def menu() -> None:
    with ui.row().classes("gap-4 items-center"):
        ui.button(
            icon='home',
            on_click=lambda: ui.navigate.to("/")
        ).props("flat dense").classes("text-white").tooltip('Home')

        ui.button(
            icon='bar_chart',
            on_click=lambda: ui.navigate.to("/results")
        ).props("flat dense").classes("text-white").tooltip('Results')
        
        ui.button(
            icon='description', 
            on_click=lambda: ui.navigate.to('https://pages.nist.gov/ARIAC_docs/en/latest/', new_tab=True)
        ).props('flat dense').classes("text-white").tooltip('Docs')
        
        ui.button(
            icon="code",
            on_click=lambda: ui.navigate.to('https://github.com/usnistgov/ARIAC', new_tab=True)
        ).props("flat dense").classes("text-white").tooltip("GitHub")

@contextmanager
def frame(page_name = '', show_menu: bool = True):
    ui.colors(primary="#2282c7", knob="#DFDFDF")
    ui.button.default_props('rounded')

    with ui.header().classes('items-center'):
        ui.label('ARIAC').classes('font-bold text-xl')
        ui.space()
        ui.label(f'{page_name}').classes('font-bold text-xl absolute-center')
        ui.space()
        if show_menu:
            menu()

    with ui.column().classes('w-full h-full items-center'):
        yield