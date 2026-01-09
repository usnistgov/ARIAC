import os
import threading
import signal
from nicegui import ui, app
import rclpy
from rclpy.executors import ExternalShutdownException

from ariac_app.app_node import AppNode
from ariac_app import ros_globals
from ariac_app import app_utils

from ariac_app.pages.redirection_page import RedirectionPage
from ariac_app.pages.home_page import HomePage
from ariac_app.pages.run_page import RunPage
from ariac_app.pages.run_results_page import RunResultsPage
from ariac_app.pages.results_page import ResultsPage
from ariac_app.pages.competition_setup_page import CompetitionRunSetupPage
from ariac_app.pages.competition_page import CompetitionRunPage

def main():
    pass

def ros_main() -> None:
    rclpy.init()
    ros_globals.node = AppNode() 
    try:
        rclpy.spin(ros_globals.node)
    except ExternalShutdownException:
        pass

def _sigint_handler(signum, frame):
    ros_globals.shutting_down = True
    print('SIGINT received: marking ros_globals.shutting_down = True')

signal.signal(signal.SIGINT, _sigint_handler)

def shutdown():
    ui.navigate.to('/')
    if app_utils.is_gazebo_running():
        app_utils.kill_gazebo()

app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())
app.on_shutdown(shutdown)

try:
    os.environ["START_PAGE"]="/home_page"
    ui.run(
        title="ARIAC App",
        reload=False,
        favicon='🤖',
        show=False
    )

except KeyboardInterrupt:
    pass