import os

from nicegui import ui

@ui.page("/")
class RedirectionPage:
    def __init__(self):
        ui.navigate.to(os.getenv("START_PAGE", "/home_page"))