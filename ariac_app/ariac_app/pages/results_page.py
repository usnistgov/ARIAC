from nicegui import ui, events, app
from pathlib import Path
from typing import Any

from ariac_app.theme import frame
from ariac_app.dialogs.warning import Warning

from ariac_db.manager import DatabaseManager, DatabaseError
from ariac_db.scoring import ARIACScorer


@ui.page("/results")
class ResultsPage:
    def __init__(self):

        self.db_path = app.storage.general.get("db_path", None)

        self.db_manager: DatabaseManager

        self.connect_timer = ui.timer(0.1, self.connect, once=True)

        self.selected_trial = ""
        self.selected_team = ""

        ui.select.default_classes('w-48')

        with frame(page_name="Results"):
            with ui.card().classes('items-center') as self.card:
                pass

    async def connect(self):
        if self.db_path is None:
            await Warning("No database found. Please create or select a database in the home menu.")
            ui.navigate.to('/')
            return
        
        try:
            self.db_manager = DatabaseManager(Path(self.db_path))
        except DatabaseError as e:
            del app.storage.general["db_path"]
            await Warning("Unable to connect to database")
            ui.navigate.to('/')
            return
        
        self.trial_ids = self.db_manager.get_all_trial_ids()
        self.competitor_names = self.db_manager.get_all_competitor_names()

        self.content()

        ui.notify("Connected to database", type="positive")
        
    def content(self):
        with self.card:
            ui.label("Trial Summary").classes('text-xl')
            self.trials_table = TrialsTable(self.db_manager)

            ui.separator()

            ui.label("Run Results").classes('text-xl')

            with ui.row():
                ui.select(
                    self.competitor_names,
                    label="Team Name",
                    on_change=self.update_tables
                ).bind_value(self, "selected_team").tooltip("Select a trial to view results for")
                
                ui.select(
                    self.trial_ids,
                    label="Trial ID",
                    on_change=self.update_tables
                ).bind_value(self, "selected_trial").tooltip("Select a trial to view results for")

            self.run_table = RunTable(self.db_manager)

    def update_tables(self):
        if self.selected_team:
            self.trials_table.update_table(self.selected_team)

        if self.selected_trial:
            self.run_table.update_table(self.selected_trial, self.selected_team)
    
class RunTable(ui.table):
    def __init__(self, db_manager: DatabaseManager):
        self.db_manager = db_manager
        self.scorer  = ARIACScorer()

        columns = [
            {'name': 'id', 'label': 'ID', 'field': 'id', 'align': 'center'},
            {'name': 'duration', 'label': 'Duration', 'field': 'duration', 'align': 'center'},
            {'name': 'score', 'label': 'Score', 'field': 'score', 'align': 'center'},
            {'name': 'link', 'label': 'Link', 'field': 'link', 'align': 'center'},
        ]

        super().__init__(columns=columns, rows=[], pagination=5)

        self.add_slot('body-cell-link', '''
            <q-td :props="props" class="flex justify-center items-center">
                <q-btn 
                    color="primary" 
                    label="View" 
                    :href="props.value" 
                    dense 
                    flat 
                    size="sm"
                    class="q-px-sm q-py-xs text-sm"
                />
            </q-td>
        ''')
    
        self.add_slot('no-data', '''
            <div class="text-center">
                No runs found for this team and trial
            </div>
        ''')

    def update_table(self, selected_trial, selected_competitor):
        rows = self.get_rows(selected_trial, selected_competitor)
        self.update_rows(rows)
    
    def view(self, e: events.GenericEventArguments):
        ui.navigate.to(f"/run_results?run_id={e.args['id']}&db_path={self.db_manager.db_path}")

    def get_rows(self, selected_trial_id: str, competitor_name: str) -> list[dict[str, Any]]:
        rows = []
        
        trial = self.db_manager.get_trial_by_id(selected_trial_id)
        if trial is None:
            return []

        run_ids = self.db_manager.get_run_ids_for_trial(selected_trial_id, competitor_name)
        for run_id in run_ids:
            run = self.db_manager.get_run(run_id)
            if run is None or not run.completed:
                continue

            penalties = self.db_manager.get_penalties_for_run(run_id)
            orders = self.db_manager.get_orders_for_run(run_id)
            score = self.scorer.score_run(run, trial, orders, penalties)

            link = f"/run_results?run_id={run_id}&db_path={self.db_manager.db_path}"

            rows.append({
                'id': run.id,
                'duration': run.duration,
                'score': round(score, 2),
                'link': link
            })

        return rows

class TrialsTable(ui.table):
    def __init__(self, db_manager: DatabaseManager):
        self.db_manager = db_manager

        columns = [
            {'name': 'id', 'label': 'ID', 'field': 'id', 'align': 'center'},
            {'name': 'kits', 'label': 'Kits', 'field': 'kits', 'align': 'center'},
            {'name': 'modules', 'label': 'Modules', 'field': 'modules', 'align': 'center'},
            {'name': 'high_priority_kits', 'label': 'High Priority Kits', 'field': 'high_priority_kits', 'align': 'center'},
            {'name': 'total_runs', 'label': 'Total Runs', 'field': 'total_runs', 'align': 'center'},
            {'name': 'team_runs', 'label': 'Team Runs', 'field': 'team_runs', 'align': 'center'}
        ]
        
        super().__init__(rows=self.get_rows(), columns=columns, pagination=5)
        
        self.classes()
    
    def get_rows(self, team_name = None) -> list:
        rows = []

        trial_ids = self.db_manager.get_all_trial_ids()

        for trial_id in trial_ids:
            trial = self.db_manager.get_trial_by_id(trial_id)

            if trial is None:
                continue

            total_runs = 0
            team_runs = "N/A"
            for team in self.db_manager.get_all_competitor_names():
                num_runs = len(self.db_manager.get_run_ids_for_trial(trial_id, team))
                total_runs += num_runs

                if team == team_name:
                    team_runs = num_runs

            rows.append(
                {
                    'id': trial_id,
                    'kits': trial.num_kits,
                    'modules': trial.num_modules,
                    'high_priority_kits': trial.num_high_priority,
                    'total_runs': total_runs,
                    'team_runs': team_runs,
                }
            )
        
        return rows

    def update_table(self, competitor_name):
        self.update_rows(self.get_rows(competitor_name))