from nicegui import ui
from pathlib import Path

from ariac_app.theme import frame
from ariac_app.components.latex_table import LatexTable

from ariac_db.manager import DatabaseManager
from ariac_db.structures import PenaltyType
from ariac_db.scoring import ARIACScorer


@ui.page('/run_results')
class RunResultsPage:
    def __init__(self, run_id: int | None = None, db_path: str | None = None):
        if run_id is None:
            ui.notify("To access the results page, a run id must be provided", type="negative")
            ui.navigate.to("/")
            return
        
        if db_path is None:
            ui.notify("To access the results page, a database path must be provided", type="negative")
            ui.navigate.to("/")
            return

        self.db_manager = DatabaseManager(Path(db_path))

        self.run = self.db_manager.get_run(run_id)

        if self.run is None:
            ui.notify(f"Could not find run with id {run_id} in database", type="negative")
            ui.navigate.to("/")
            return

        if not self.run.completed:
            ui.notify(f"The run with id {run_id} was not completed", type="negative")
            ui.navigate.to("/")
            return
        
        self.trial = self.db_manager.get_trial_for_run(self.run)

        if self.trial is None:
            ui.notify(f"Could not find trial for run with id {run_id} in database", type="negative")
            ui.navigate.to("/")
            return
        
        self.competitor_name = self.db_manager.get_competitor_name_for_run(self.run.id)
        
        self.orders = self.db_manager.get_orders_for_run(self.run.id)
        self.penalties = self.db_manager.get_penalties_for_run(self.run.id)

        self.scorer = ARIACScorer()
        
        self.kits_score = self.scorer.score_kits(self.orders, self.trial)
        self.modules_score = self.scorer.score_modules(self.orders, self.trial)
        self.bonuses = self.scorer.score_bonuses(self.run, self.trial, self.orders)
        self.penalty_results = self.scorer.score_penalties(self.run, self.penalties)
        self.run_score = self.scorer.score_run(self.run, self.trial, self.orders, self.penalties)
            
        with frame(page_name='Run Results'):
            with ui.card().classes('w-5/6 max-w-8xl items-center'):
                with ui.row().classes('w-full justify-center'):
                    with ui.card().classes('w-40'):
                        ui.label('Trial ID').classes('text-xs text-gray-500 leading-none')
                        ui.label(f'{self.trial.trial_id}').classes('w-full text-xl text-center leading-none')
                    with ui.card().classes('w-64'):
                        ui.label('Competitor Name').classes('text-xs text-gray-500 leading-none')
                        ui.label(f'{self.competitor_name}').classes('w-full text-xl text-center leading-none')
                    with ui.card().classes('w-24'):
                        ui.label('Run ID').classes('text-xs text-gray-500 leading-none')
                        ui.label(f'{self.run.id}').classes('w-full text-xl text-center leading-none')
                
                ui.separator()

                self._final_score_breakdown()
                ui.separator()
                self._details()
                ui.separator()
                self._bonuses_breakdown()
                ui.separator()
                self._penalties_breakdown()
    
    def _final_score_breakdown(self):
        score_components = {
            "Kit Score": self.kits_score,
            "Modules Score": self.modules_score,
            "Total Bonuses": self.bonuses.total(),
            "Total Penalties": self.penalty_results.total(),
        }

        with ui.card().classes('w-40 p-2'):
            ui.label('Run Score').classes('text-xs text-gray-500 leading-none')
            ui.label(f'{self.run_score:.2f}').classes('w-full text-center font-bold text-2xl text-green-600 leading-none')

        with ui.row().classes('w-full justify-center'):
            for label, val in score_components.items():
                with ui.card().classes('w-36 p-2'):
                    ui.label(label).classes('text-xs text-gray-500 leading-none')
                    ui.label(f'{val:.2f}').classes('w-full text-center text-xl leading-none')
        
        self._score_chart()

    def _details(self):
        if self.trial is None or self.run is None:
            return
        
        with ui.expansion('Details', icon='list').classes('w-3/4 text-base'):
            rows = [
                {'symbol': r'\( t_m \)', 'description': 'Time limit', 'value': self.trial.time_limit},
                {'symbol': r'\( k_d \)', 'description': 'Desired kits', 'value': self.trial.num_kits + self.trial.num_high_priority},
                {'symbol': r'\( m_d \)', 'description': 'Desired modules', 'value': self.trial.num_modules},
            ]

            columns = [
                {'name': 'symbol', 'label': 'Symbol', 'field': 'symbol', 'align': 'left'},
                {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
                {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'center'},
            ]

            LatexTable(
                title='Trial Info',
                columns=columns,
                rows=rows,
                latex_column_names=['symbol'],
            ).table.classes('w-full')


            rows = [
                {'symbol': r'\(\omega_1\)', 'description': 'Kit completion weight', 'value': self.scorer.weights.W1},
                {'symbol': r'\(\omega_2\)', 'description': 'Module completion weight', 'value': self.scorer.weights.W2},
                {'symbol': r'\(\omega_3\)', 'description': 'Trial time bonus weight', 'value': self.scorer.weights.W3},
                {'symbol': r'\(\omega_4\)', 'description': 'Inspection speed bonus weight', 'value': self.scorer.weights.W4},
                {'symbol': r'\(\omega_5\)', 'description': 'High priority order speed bonus weight', 'value': self.scorer.weights.W5},
                {'symbol': r'\(\omega_6\)', 'description': 'Sensor cost weight', 'value': self.scorer.weights.W6},
                {'symbol': r'\(\omega_7\)', 'description': 'Inspection classification weight', 'value': self.scorer.weights.W7},
            ]

            columns = [
                {'name': 'symbol', 'label': 'Symbol', 'field': 'symbol', 'align': 'left'},
                {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
                {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'center'},
            ]

            LatexTable(
                title='Weights',
                columns=columns,
                rows=rows,
                latex_column_names=['symbol'],
            ).table.classes('w-full')
    

            rows = [
                {'symbol': r'\( \gamma_n \)', 'description': 'Nominal inspection duration', 'value': self.scorer.gamma_d},
                {'symbol': r'\( \tau_n \)', 'description': 'Nominal high priority kit completion duration', 'value': self.scorer.tau_d},
                {'symbol': r'\( \sigma_b \)', 'description': 'Sensor budget', 'value': self.scorer.sigma_b},

            ]

            columns = [
                {'name': 'symbol', 'label': 'Symbol', 'field': 'symbol', 'align': 'left'},
                {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
                {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'center'},
            ]

            LatexTable(
                title='Nominal Values',
                columns=columns,
                rows=rows,
                latex_column_names=['symbol'],
            ).table.classes('w-full')

            rows = [
                {'symbol': r'\( t_e \)', 'description': 'Run execution time', 'value': self.run.duration},
                {'symbol': r'\( \sigma \)', 'description': 'Sensor cost', 'value': self.run.sensor_cost},
                {'symbol': r'\( \tau \)', 'description': 'Average report time', 'value': self.run.avg_report_time},
                {'symbol': r'\( \delta \)', 'description': 'Number of defective cells', 'value': self.run.defective_cells},
                {'symbol': r'\( \nu \)', 'description': 'Number of correctly classified reports', 'value': self.run.num_correct_report_classifications},
            ]

            columns = [
                {'name': 'symbol', 'label': 'Symbol', 'field': 'symbol', 'align': 'left'},
                {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
                {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'center'},
            ]

            LatexTable(
                title='Run Info',
                columns=columns,
                rows=rows,
                latex_column_names=['symbol'],
            ).table.classes('w-full')
            

    def _bonuses_breakdown(self):
        ui.label('Bonuses Breakdown').classes('text-xl font-bold')

        rows = [
            {'label': 'β₁', 'description': 'Trial Time Execution Bonus', 'value': f'{self.bonuses.b1.amount:.2f}'},
            {'label': 'β₂', 'description': 'Inspection Speed Bonus', 'value': f'{self.bonuses.b2.amount:.2f}'},
            {'label': 'β₃', 'description': 'High Priority Speed Bonus', 'value': f'{self.bonuses.b3.amount:.2f}'},
            {'label': 'β₄', 'description': 'Sensor Cost Bonus', 'value': f'{self.bonuses.b4.amount:.2f}'},
            {'label': 'β₅', 'description': 'Defect Classification Bonus', 'value': f'{self.bonuses.b5.amount:.2f}'},
        ]

        columns = [
            {'name': 'label', 'label': 'Label', 'field': 'label', 'align': 'center'},
            {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
            {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'center'},
        ]

        with ui.row().classes('w-full items-center justify-center items-stretch'):
            ui.table(columns=columns, rows=rows).classes('w-96')
            self._bonuses_chart()

    def _penalties_breakdown(self):
        ui.label('Penalties Breakdown').classes('text-xl font-bold')

        rows = [
            {'label': 'ρ₁', 'description': self.penalty_results.p1.description, 'deduction': f'{self.penalty_results.p1.total_deduction:.2f}'},
            {'label': 'ρ₂', 'description': self.penalty_results.p2.description, 'deduction': f'{self.penalty_results.p2.total_deduction:.2f}'},
            {'label': 'ρ₃', 'description': self.penalty_results.p3.description, 'deduction': f'{self.penalty_results.p3.total_deduction:.2f}'},
            {'label': 'ρ₄', 'description': self.penalty_results.p4.description, 'deduction': f'{self.penalty_results.p4.total_deduction:.2f}'},
            {'label': 'ρ₅', 'description': self.penalty_results.p5.description, 'deduction': f'{self.penalty_results.p5.total_deduction:.2f}'},
            {'label': 'ρ₆', 'description': self.penalty_results.p6.description, 'deduction': f'{self.penalty_results.p6.total_deduction:.2f}'},
        ]

        columns = [
            {'name': 'label', 'label': 'Symbol', 'field': 'label', 'align': 'center'},
            {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
            {'name': 'deduction', 'label': 'Total Deduction', 'field': 'deduction', 'align': 'center'},
        ]

        with ui.row().classes('w-full items-center justify-center'):
            ui.table(columns=columns, rows=rows).classes('w-120')
            self._penalties_chart()


        labels = {
            int(PenaltyType.GOOD_CELL_IN_INSPECTION_BIN): 'ρ₁',
            int(PenaltyType.CELL_IN_CONVEYOR_BIN): 'ρ₂',
            int(PenaltyType.OBJECT_ON_INVALID_SURFACE): 'ρ₃',
            int(PenaltyType.AGV_COLLISION): 'ρ₄',
            int(PenaltyType.ROBOT_COLLISION): 'ρ₅',
        }

        with ui.expansion('Penalty Occurrences', icon='list').classes('w-3/4 text-base'):
            ui.aggrid({
                'columnDefs': [
                    {'headerName': 'Type', 'field': 'type'},
                    {'headerName': 'Description', 'field': 'description'},
                    {'headerName': 'Time', 'field': 'time', 'sort': 'asc'}
                ],
                'rowData': [{'type': labels[p.type], 'description': p.description, 'time': p.time} for p in self.penalties],
            }).classes('w-full h-48')

    def _score_chart(self):
        total_bonuses = self.bonuses.total()
        total_penalties = self.penalty_results.total()

        with ui.column().classes('w-3/4'):
            ui.echart({
                'tooltip': {'trigger': 'axis'},
                'xAxis': {
                    'type': 'category',
                    'data': ['Kits', 'Modules', 'Bonuses', 'Penalties']
                },
                'yAxis': {
                    'type': 'value',
                    'name': 'Score'
                },
                'series': [{
                    'type': 'bar',
                    'data': [
                        {'value': self.kits_score, 'itemStyle': {'color': '#8ecae6'}},
                        {'value': self.modules_score, 'itemStyle': {'color': '#219ebc'}},
                        {'value': total_bonuses, 'itemStyle': {'color': '#FB8500'}},
                        {'value': -total_penalties, 'itemStyle': {'color': '#ce2d4f'}},
                    ]
                }]
            }).classes('h-96')

    def _bonuses_chart(self):
        bonus_items = [
            ('β₁', self.bonuses.b1.amount, '#8ecae6'),
            ('β₂', self.bonuses.b2.amount, '#219ebc'),
            ('β₃', self.bonuses.b3.amount, '#023047'),
            ('β₄', self.bonuses.b4.amount, '#ffb703'),
            ('β₅', self.bonuses.b5.amount, '#fb8500'),
        ]
        
        with ui.column().classes('w-1/3'):
            ui.echart({
                'title': {'show': False},
                'tooltip': {'trigger': 'item'},
                'legend': { 'orient': 'horizontal', 'bottom': 0, 'left': 'center',},
                'series': [{
                    'name': 'Bonuses',
                    'type': 'pie',
                    'radius': ['50%', '70%'],
                    'data': [{'value': val, 'name': label, 'itemStyle': {'color': color}} for label, val, color in bonus_items if val > 0]
                }]
            }).classes('h-96')

    def _penalties_chart(self):
        penalty_items = [
            ('ρ₁', self.penalty_results.p1.total_deduction, '#8ecae6'),
            ('ρ₂', self.penalty_results.p2.total_deduction, '#219ebc'),
            ('ρ₃', self.penalty_results.p3.total_deduction, '#023047'),
            ('ρ₄', self.penalty_results.p4.total_deduction, '#ffb703'),
            ('ρ₅', self.penalty_results.p5.total_deduction, '#fb8500'),
            ('ρ₆', self.penalty_results.p6.total_deduction, '#ce2d4f'),
        ]
        with ui.column().classes('w-1/3'):
            ui.echart({
                'tooltip': {'trigger': 'item'},
                'legend': { 'orient': 'horizontal', 'bottom': 0, 'left': 'center',},
                'series': [{
                    'name': 'Penalties',
                    'type': 'pie',
                    'radius': ['50%', '70%'],
                    'data': [{'value': val, 'name': label, 'itemStyle': {'color': color}} for label, val, color in penalty_items if val > 0]
                }]
            }).classes('h-96')