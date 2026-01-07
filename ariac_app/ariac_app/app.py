import threading
from nicegui import ui, app
import rclpy
from rclpy.executors import ExternalShutdownException

from ariac_app.app_node import AppNode
from ariac_app import ros_globals
from ariac_app import app_utils

from ariac_app.pages.home_page import HomePage
from ariac_app.pages.run_page import RunPage
from ariac_app.pages.run_results_page import RunResultsPage
from ariac_app.pages.results_page import ResultsPage

def main():
    pass

def ros_main() -> None:
    rclpy.init()
    ros_globals.node = AppNode() 
    try:
        rclpy.spin(ros_globals.node)
    except ExternalShutdownException:
        pass

def shutdown():
    ui.navigate.to('/')
    if app_utils.is_gazebo_running():
        app_utils.kill_gazebo()

app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())
app.on_shutdown(shutdown)

try:
    ui.run(
        title="ARIAC App",
        reload=False,
        favicon='🤖',
        show=False
    )

except KeyboardInterrupt:
    pass