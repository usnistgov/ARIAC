from nicegui import ui, Client
from pathlib import Path

from ariac_db.manager import DatabaseManager, DatabaseError

class TrialFinished(ui.dialog):
    def __init__(
            self, 
            client: Client, 
            run_id: int | None = None, 
            db_path: str | None = None
        ):
        super().__init__()

        with self.classes('w-full'), ui.card().classes('w-full'):
                ui.label("GZ has been closed. Please select a destination or cancel to stay on this window.")
                with ui.row().classes("items-center w-full justify-center"):
                    if run_id not in [None, -1] and db_path is not None:
                        with DatabaseManager(Path(db_path)) as db:
                            if self.run_completed(db, run_id):
                                ui.button("Results", on_click=lambda: client.open(f"/run_results?run_id={run_id}&db_path={db_path}"))
                                ui.notify("Run completed, results available", type="positive")
                            else:
                                ui.notify("Run not completed. Results not available.", type="negative")
                    ui.button("Home", on_click=lambda: client.open("/"))
                    ui.button("Rerun Trial", on_click=lambda: client.run_javascript("window.location.reload();"))
                    ui.button("Cancel", on_click=self.close)

    def run_completed(self, db_manager: DatabaseManager, run_id: int | None):
        if run_id is None:
            return False
        
        run = db_manager.get_run(run_id)

        if run is None:
            return False

        return run.completed
        