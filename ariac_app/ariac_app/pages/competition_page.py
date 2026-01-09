import os
import shutil
import psutil
import yaml

from pathlib import Path
from dataclasses import dataclass, field
from typing import Any

from math import inf

from rclpy.duration import Duration
from rclpy.time import Time

import asyncio

from nicegui import ui, run

from ariac_interfaces.msg import CompetitionStates, CompetitionTime

from ariac_db.manager import DatabaseManager
from ariac_db.scoring import ARIACScorer
from ariac_db.structures import OrderType

from ariac_app.theme import frame
from ariac_app.components.process_manager import ProcessManager
from ariac_app import ros_globals, app_utils
from ariac_app.pages.run_page import StatusDisplay
from ariac_app.dialogs.confirmation import Confirmation

from ariac_setup.score_logger import ScoreLogger

if "QT_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_PLUGIN_PATH"]:
    del os.environ["QT_PLUGIN_PATH"]
    
import cv2
import numpy as np
import subprocess
import sys
from tqdm import tqdm


@dataclass
class RunInfo:
    competition_log_file_path: Path | None = None
    team_log_file_path: Path | None = None
    kits_completed: int = 0
    modules_completed: int = 0

@dataclass
class TrialInfo:
    run_count: int = 0
    successful_count: int = 0
    highest_score: float | None = None
    competition_log_file_path: str | None = None
    team_log_file_path: str | None = None
    run_infos: dict[int, RunInfo] = field(default_factory=dict)


@ui.page('/competition')
class CompetitionRunPage:
    RECORDER_PATHS_TO_TARGETS = {
        "/tmp/inspection_recorder_run_id_runidhold.mp4": "inspection.mp4",
        "/tmp/assembly_recorder_run_id_runidhold.mp4": "assembly.mp4",
        "/tmp/environment_recorder_run_id_runidhold.mp4": "environment.mp4"
    }
    def __init__(
            self, 
            successful_runs: int | None = None,
            max_runs: int | None = None,
            runs_to_score: int | None = None,
            trials: str | None = None,
            headless: str | None = None,
            record: str | None = None,
            save_unscored_videos: str | None = None,
            db_path: str | None = None
          ):
                
        if successful_runs is None or max_runs is None or runs_to_score is None or trials is None or headless is None or record is None or save_unscored_videos is None:
            print("Trials, successful runs, and max runs have to be entered")
            ui.navigate.to("/")
            return
        
        if trials is None: return
        
        if app_utils.is_gazebo_running():
            print("Gazebo is already running")
            ui.navigate.to("/")
            return
        
        self.video_combiner = VideoCombiner()
        
        self.max_runs = max_runs
        self.successful_run_threshold = successful_runs
        self.runs_to_score = runs_to_score
        
        self.trials_to_run: list[str] = list(trials.split(","))
        self.trial_infos: dict[str, TrialInfo] = {trial_path: TrialInfo() for trial_path in self.trials_to_run}

        self.completed_runs_of_trial = 0
        self.completed_trials = 0
        self.total_runs = self.max_runs * len(self.trials_to_run)
        self.runs_completed_ratio = 0.0

        self.save_unscored_videos = save_unscored_videos.lower() == "true"

        self.current_trial = "None"

        self.team_config = os.getenv("TEAM_CONFIG")

        if self.team_config is None:
            print("Environment variable TEAM_CONFIG not set")
            ui.navigate.to("/")
            return
        
        self.team_cmd = os.getenv("TEAM_COMMAND")

        if self.team_cmd is None:
            print("Environment variable TEAM_COMMAND not set")
            ui.navigate.to("/")
            return

        self.cmd = f"ros2 launch ariac_gz ariac.launch.py trial_config:=holdtrial user_config:={self.team_config} gz_log_level:=info headless:={headless} record:={record}"

        self.db_manager: DatabaseManager | None = None
        self.score_logger: ScoreLogger | None = None
        if db_path is not None:
            self.db_manager = DatabaseManager(Path(db_path))
            self.cmd += f" db_path:={db_path}"
            self.score_logger = ScoreLogger(db_path)
        
        self.db_path = db_path

        self.status_display = StatusDisplay()
        self.run_info_table = RunInfoTable(self.max_runs, self.successful_run_threshold)

        self.content()

        self.client = ui.context.client

        self.competition_process: ProcessManager | None = None
        self.team_process: ProcessManager | None = None
        
        self.quitting = False

        # Prevents disconnects from timeouts
        self.heartbeat_timer = ui.timer(30.0, lambda: None)

        ui.context.client.on_disconnect(self._handle_disconnect)

        self.timer = ui.timer(1.0, self.check_process)

    def content(self):
        with frame(page_name='competition', show_menu=False):
            with ui.column().classes("w-5/6 max-w-3xl items-center justify-center"):
                with ui.card().classes("w-full items-center"):
                    self.status_display.content()
                with ui.row().classes("w-full justify-center items-center"):
                    ui.label("Current trial:").classes('text-base')
                    ui.label("None").classes('text-base').bind_text(self, "current_trial")
                with ui.row().classes("w-full justify-center items-center"):
                    ui.label("Progress:").classes('text-base')
                    ui.linear_progress(show_value=False).classes("w-80").bind_value(self, "runs_completed_ratio")
                self.trials_completed_label = ui.label("Trials completed: 0").classes('text-base')
                self.run_info_table.content(self.trial_infos)

            self.run_button = ui.button("Start Runs", on_click=self.run_comp_button_func, color="green", icon="play_circle").classes('text-lg')
            self.stop_button = ui.button("Stop", on_click=self.quit, color="red", icon="dangerous").classes('text-lg')
    
    # replace the old button handler:
    async def run_comp_button_func(self):
        asyncio.create_task(self.run_competition_async())

    # new async version of the runner (replace the old run_competition)
    async def run_competition_async(self):
        if self.max_runs is None or self.successful_run_threshold is None or ros_globals.node is None or self.team_config is None:
            return
        
        team_name = ""
        with open(self.team_config) as file:
            try:
                team_config_data: dict = yaml.safe_load(file)
                team_name = team_config_data.get("COMPETITOR_NAME", "")
            except yaml.YAMLError as exc:
                print(exc)
        
        if team_name == "":
            ui.notify("Could not find team name in team config", type="negative")

        
        if Path("/results").exists():
            results_dir = Path("/results")
        else:
            results_dir = Path("results")
        team_dir = results_dir / team_name

        if team_dir.exists():
            with self.client:
                if not await Confirmation("Results for this team already exist. Do you want to rerun this?"):
                    ui.navigate.to("/")
                else:
                    old_runs_dir = results_dir / "old_runs"
                    if not old_runs_dir.exists():
                        old_runs_dir.mkdir()
                    old_runs_team_dir = old_runs_dir / team_name
                    if not old_runs_team_dir.exists():
                        old_runs_team_dir.mkdir()
                    competition = 1
                    while (old_runs_team_dir / f"competition_run_{competition}").exists():
                        competition +=1
                    
                    old_team_competition_dir = old_runs_team_dir / f"competition_run_{competition}"
                    old_team_competition_dir.mkdir()

                    for item in os.listdir(team_dir):
                        source_item = os.path.join(team_dir, item)
                        destination_item = os.path.join(old_team_competition_dir, item)
                        
                        try:
                            shutil.move(source_item, destination_item)
                        except shutil.Error as e:
                            print(f"Error moving {item}: {e}")
                
        team_dir.mkdir(mode=0o777, exist_ok=True, parents=True)
        os.chmod(team_dir, 0o777)

        self.run_button.disable()

        for trial in self.trials_to_run:
            self.completed_runs_of_trial = 0
            self.current_trial = Path(trial).name
            trial_dir_path = team_dir / self.current_trial.split(".")[0]
            trial_dir_path.mkdir(mode=0o777, parents=True, exist_ok=True)
            os.chmod(trial_dir_path, 0o777)

            for _ in range(self.max_runs):
                if self.quitting or ros_globals.shutting_down:
                    break

                ros_globals.node.reset()
                self.trial_infos[trial].run_count += 1

                # run_trial is async, await it instead of using asyncio.run
                successful = await self.run_trial(trial)

                if successful:
                    self.trial_infos[trial].successful_count += 1
                    if self.trial_infos[trial].successful_count >= self.successful_run_threshold:
                        self.run_info_table.update(self.trial_infos)
                        break

                self.runs_completed_ratio = (self.completed_trials * self.max_runs + self.completed_runs_of_trial) / self.total_runs
                self.run_info_table.update(self.trial_infos)

            # This is blocking (file moves + video work) — run it in a threadpool
            await run.io_bound(self.organize_files, trial, trial_dir_path, team_dir)

            self.completed_runs_of_trial = 0
            self.completed_trials += 1
            self.runs_completed_ratio = (self.completed_trials * self.max_runs) / self.total_runs
            self.trials_completed_label.set_text(f"Trials completed: {self.completed_trials}")

            if self.quitting or ros_globals.shutting_down:
                print(f"self.quitting: {self.quitting}\tshutting_down: {ros_globals.shutting_down}")
                print("Quitting detected")
                break
    
    def organize_files(self, trial_path: str, trial_dir: Path, team_dir: Path):
        if len(self.trial_infos[trial_path].run_infos) == 0:
            return
        
        if self.db_manager is None:
            return
        
        score_dict: dict[int, float] = {}
        scorer = ARIACScorer()
        
        for run_id in self.trial_infos[trial_path].run_infos.keys():
            if (run := self.db_manager.get_run(run_id)) is None:
                score_dict[run_id] = -inf
                continue

            if (trial := self.db_manager.get_trial_for_run(run)) is None:
                score_dict[run_id] = -inf
                continue
            
            orders = self.db_manager.get_orders_for_run(run_id)
            penalties = self.db_manager.get_penalties_for_run(run_id)

            self.trial_infos[trial_path].run_infos[run_id].kits_completed = len(
                [order for order in orders if order.order_type in [OrderType.KIT, OrderType.HIGH_PRIORITY] and order.submission_time > 0]
            )

            self.trial_infos[trial_path].run_infos[run_id].modules_completed = len(
                [order for order in orders if order.order_type == OrderType.MODULE and order.submission_time > 0]
            )
            
            score_dict[run_id] = scorer.score_run(run, trial, orders, penalties)

        sorted_scores = sorted(score_dict.items(), key=lambda item: item[1], reverse=True)
        
        scored_scores = []
        for i, (run_id, score) in enumerate(sorted_scores, start=1):
            scored = i <= min(self.runs_to_score, self.successful_run_threshold) and score != -inf
            if scored:
                scored_scores.append(score if score > 0 else 0)
                run_dir = trial_dir / f"run_{run_id}"
            else:
                run_dir = trial_dir / "unscored" / f"run_{run_id}"
            run_dir.mkdir(mode=0o777, parents=True, exist_ok=False)
            os.chmod(run_dir, 0o777)

            logs_dir = run_dir / "logs"
            logs_dir.mkdir(mode=0o777, parents=True)
            os.chmod(logs_dir, 0o777)

            comp_log_path = self.trial_infos[trial_path].run_infos[run_id].competition_log_file_path
            team_log_path = self.trial_infos[trial_path].run_infos[run_id].team_log_file_path

            if comp_log_path is None or team_log_path is None:
                continue
            
            shutil.move(comp_log_path, logs_dir / "competition_logs.txt")
            shutil.move(team_log_path, logs_dir / "team_logs.txt")
            if self.score_logger is not None:
                if score != -inf:
                    score_output = self.score_logger.get_results_str(run_id)
                    if score_output is not None:
                        with open(logs_dir / "score.txt", "w") as f:
                            f.write(score_output)

            file_not_found = False
            if scored or self.save_unscored_videos:
                videos_dir = run_dir / "videos"
                videos_dir.mkdir(mode=0o777, parents=True)
                os.chmod(videos_dir, 0o777)
                for video_path, target in self.RECORDER_PATHS_TO_TARGETS.items():
                    current_video_path_str = video_path.replace("runidhold", str(run_id))
                    current_video_path = Path(current_video_path_str)
                    try:
                        shutil.move(current_video_path, videos_dir / target)
                    except FileNotFoundError as e:
                        print(f"Could not move file since it does not exist.\nError: {e}")
                        file_not_found = True
                        continue
                if file_not_found:
                    continue
                        
                
                if score != -inf:
                    self.video_combiner.create_video(
                        str(videos_dir / "inspection.mp4"),
                        str(videos_dir / "assembly.mp4"),
                        str(videos_dir / "environment.mp4"),
                        str(videos_dir / "combined.mp4"),
                        trial_id=Path(trial_path).name.split(".")[0],
                        run_id=str(run_id),
                        kits_completed=self.trial_infos[trial_path].run_infos[run_id].kits_completed, # type: ignore
                        modules_completed=self.trial_infos[trial_path].run_infos[run_id].modules_completed, # type: ignore
                        kits_requested=ros_globals.node.total_kits, # type: ignore
                        modules_requested=ros_globals.node.total_modules, # type: ignore
                        score=score,
                        time_limit_seconds=-1 if ros_globals.node is None or ros_globals.node.time_limit is None \
                                            else ros_globals.node.time_limit,
                    )
                
                    (videos_dir / "inspection.mp4").unlink(True)
                    (videos_dir / "assembly.mp4").unlink(True)
                    (videos_dir / "environment.mp4").unlink(True)
        
        execution_scores_path = team_dir / "execution_scores.txt"

        if not execution_scores_path.exists() or execution_scores_path.stat().st_size == 0:
            with open(execution_scores_path, "a") as file:
                file.write("Execution Scores\n")
                file.write("================\n")
        
        with open(execution_scores_path, "a") as file:
            avg = sum(scored_scores) / len(scored_scores)
            file.write(f"{trial_dir.name}: {round(avg, 2)}\n")
                
    async def cleanup_before_start(self):
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                if 'gz' in ' '.join(p.info['cmdline']):
                    p.kill()
            except Exception:
                pass

        # Clean temp files
        for path in Path('/tmp').glob('your_app_*'):
            try:
                path.unlink()
            except Exception:
                pass
    
    async def run_trial(self, trial_path: str) -> bool:
        cmd = self.cmd.replace("holdtrial", trial_path)
        if ros_globals.node is None:
            print("Unable to connect to node")
            return False
        
        run_id = -1

        if getattr(ros_globals, 'shutting_down', False):
            print("Not starting processes: application is shutting down")
            return False
        
        if self.team_cmd is None:
            print("Team command cannot be NONE in run_trial")
            return False

        await self.cleanup_before_start()

        self.competition_process = ProcessManager(cmd, self.db_manager is not None)
        self.team_process = ProcessManager(self.team_cmd, self.db_manager is not None)

        start_time = ros_globals.node.current_sim_time
        
        while ros_globals.node.current_state is None or ros_globals.node.current_state in [CompetitionStates.PREPARING]:
            if (self.competition_process and not self.competition_process.is_running) or \
               (self.team_process and not self.team_process.is_running):
                print("A process died during PREPARING. Aborting trial.")
                await self.kill_processes()
                return False
            
            if ros_globals.node.current_sim_time - start_time > Duration(seconds=100):
                print("Did not start competition within 100 seconds of preparing. Killing processes and moving on")
                await self.kill_processes()
                return False

            await asyncio.sleep(0.1)
        
        start_time = ros_globals.node.current_sim_time

        await asyncio.sleep(1.0)

        while ros_globals.node.current_state == CompetitionStates.READY:
            if (self.competition_process and not self.competition_process.is_running) or \
               (self.team_process and not self.team_process.is_running):
                print("A process died while READY. Aborting trial.")
                await self.kill_processes()
                return False

            await asyncio.sleep(0.1)
            if ros_globals.node.current_sim_time - start_time > Duration(seconds=100):
                print("Did not start competition within 100 seconds of being ready. Killing processes and moving on")
                await self.kill_processes()
                return False
            
        run_id = ros_globals.node.run_id
        if run_id is not None:
            self.trial_infos[trial_path].run_infos[run_id] = RunInfo(
                self.competition_process.log_file,
                self.team_process.log_file
            )

        prev_time: CompetitionTime | None = None
        same_count = 0

        team_process_dead_time: Time | None = None
        while ros_globals.node.current_state != CompetitionStates.ENDED:
            if prev_time is not None and ros_globals.node.competition_time is not None and \
               ros_globals.node.competition_time.elapsed == prev_time.elapsed:
                same_count += 1
                if same_count >= 15:
                    print("Simulation paused for unknown reasons. Moving on to next attempt")
                    await self.kill_processes()
                    return False
            else:
                same_count = 0

            prev_time = ros_globals.node.competition_time

            if self.competition_process and not self.competition_process.is_running:
                print("Gazebo process died during the competition. Aborting trial.")
                await self.kill_processes()
                return False
            
            if self.team_process and not self.team_process.is_running:
                if team_process_dead_time is None:
                    team_process_dead_time = ros_globals.node.current_sim_time
                if ros_globals.node.current_sim_time - team_process_dead_time > Duration(seconds=10):
                    print("Team process died during the competition. Ending competition.")
                    await ros_globals.node.end_competition(True)
                    await self.kill_processes()
                    return False

            await asyncio.sleep(0.1)

        if run_id and ros_globals.node.total_kits and ros_globals.node.kits_remaining:
            self.trial_infos[trial_path].run_infos[run_id].kits_completed = ros_globals.node.total_kits - ros_globals.node.kits_remaining

        if run_id and ros_globals.node.total_modules and ros_globals.node.modules_remaining:
            self.trial_infos[trial_path].run_infos[run_id].modules_completed = ros_globals.node.total_modules - ros_globals.node.modules_remaining

        await asyncio.sleep(5)

        if self.db_manager is None or run_id is None or run_id == -1:
            print("Database not available or run id not set")
            await self.kill_processes()
            return False
        
        if (run := self.db_manager.get_run(run_id)) is None:
            print(f"Could not get run from database with run id {run_id}")
            await self.kill_processes()
            return False
        
        await self.kill_processes()

        scorer = ARIACScorer()

        if (trial := self.db_manager.get_trial_for_run(run)) is None:
            print(f"Could not get trial from database with run using run id {run_id}")
            await self.kill_processes()
            return False
        
        orders = self.db_manager.get_orders_for_run(run_id)
        penalties = self.db_manager.get_penalties_for_run(run_id)
        
        score = scorer.score_run(run, trial, orders, penalties)

        hs = self.trial_infos[trial_path].highest_score
        if hs is None or score > hs:
            self.trial_infos[trial_path].highest_score = score
        
        return run.completed

    async def kill_processes(self):
        tasks = []

        if self.competition_process and self.competition_process.is_running:
            tasks.append(self.competition_process.close())

        if self.team_process and self.team_process.is_running:
            tasks.append(self.team_process.close())

        if tasks:
            await asyncio.gather(*tasks)

        if app_utils.is_gazebo_running():
            print("Gazebo is still running, killing process")
            app_utils.kill_gazebo()
    
    async def quit(self, event=None, navigate: bool = True):
        """Stop processes and optionally navigate back to the root page.

        The optional `event` parameter keeps the signature compatible with
        NiceGUI `on_click` handlers. Set `navigate=False` when quitting from
        a client-disconnect handler to avoid using a deleted client.
        """
        self.quitting = True
        await self.kill_processes()

        node = ros_globals.node

        if node is not None:
            node.reset()

        if navigate:
            try:
                ui.navigate.to("/")
            except Exception:
                # If the client has already been deleted (disconnect flow),
                # attempting to navigate will warn/use the client. Ignore
                # any errors here since we're already quitting.
                pass

    async def check_process(self):
        if not self.competition_process:
            return
        
        if not self.competition_process.is_running and ros_globals.node is not None and ros_globals.node.current_state != CompetitionStates.ENDED:
            ui.notify("Gazebo shutdown before competition ended. Check logs.", type="warning")
        
            self.timer.deactivate()

    async def _handle_disconnect(self):
        if not self.competition_process:
            return

        if self.competition_process.is_running or (self.team_process is not None and self.team_process.is_running):
            print('Client disconnected before all processes were ended. Stopping all processes...')
            await self.quit(navigate=False)

class RunInfoTable:
    def __init__(self, max_runs: int, successful_run_threshold):
        self.max_runs = max_runs
        self.successful_run_threshold = successful_run_threshold

        self.columns = [
            {"name": "trial", 'label': 'Trial', 'field': 'trial', 'required': True, 'align': 'left'},
            {"name": "completed_attempts", 'label': 'Completed Attempts', 'field': 'completed_attempts', 'required': True},
            {"name": "uncompleted_attempts", 'label': 'Uncompleted Attempts', 'field': 'uncompleted_attempts', 'required': True},
            {"name": "total_attempts", 'label': 'Total Attempts', 'field': 'total_attempts', 'required': True},
            {"name": "completed", 'label': 'Completed', 'field': 'completed', 'required': True},
            {"name": "highest_score", 'label': 'Highest Score', 'field': 'highest_score', 'required': True},
        ]

        self.table: ui.table | None = None

    def content(self, trial_info: dict[str, TrialInfo]):
        rows = self.generate_rows(trial_info)

        self.table = ui.table(columns=self.columns, rows=rows, row_key='name')
        self.table.add_slot('body-cell-completed', '''
            <q-td key="completed" :props="props">
                <q-badge :color="props.value == 'False' ? 'red' : 'green'">
                    {{ props.value }}
                </q-badge>
            </q-td>
        ''')
    
    def update(self, trial_info):
        if self.table is None:
            return
        
        rows = self.generate_rows(trial_info)

        self.table.update_rows(rows)
    
    def generate_rows(self, trial_info_dict: dict[str, TrialInfo]) -> list[dict[str, Any]]:
        rows = []
        for trial_path, trial_info in trial_info_dict.items():
            rows.append({
                "trial": Path(trial_path).name,
                "completed_attempts": trial_info.successful_count,
                "uncompleted_attempts": trial_info.run_count - trial_info.successful_count,
                "total_attempts": trial_info.run_count,
                "completed": str(self.successful_run_threshold == trial_info.successful_count or self.max_runs==trial_info.run_count),
                "highest_score": "N/A" if trial_info.highest_score is None else round(trial_info.highest_score)
            })
        return rows
    
class VideoCombiner:
    def __init__(
            self,
            target_width=1920, 
            target_height=1080
        ):
        self.target_width = target_width
        self.target_height = target_height

        self.bottom_bar_height = 230

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.8
        self.thickness = 1
        self.font_color = (0, 0, 0)

        self.label_font_scale = 2
        self.label_font_thickness = 2
        
    
    def create_video(
            self,
            inspection_path: str, 
            assembly_path: str, 
            environment_path: str,
            target_path: str,
            trial_id: str,
            run_id: str,
            kits_completed: int,
            modules_completed: int,
            kits_requested: int,
            modules_requested: int,
            score: float =0.0,
            time_limit_seconds=1000,
        ):
        caps = [cv2.VideoCapture(v) for v in [inspection_path, assembly_path, environment_path]]
        fps = int(caps[0].get(cv2.CAP_PROP_FPS))
        width = int(caps[0].get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_count = int(caps[0].get(cv2.CAP_PROP_FRAME_COUNT))

        ffmpeg_cmd = [
            "ffmpeg",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.target_width}x{self.target_height}",
            "-r", str(fps),
            "-i", "-",
            "-c:v", "h264_nvenc",
            "-preset", "fast",
            "-b:v", "10M",
            target_path
        ]

        process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        for frame_idx in tqdm(range(frame_count), desc="Encoding", unit="frame", file=sys.stdout, leave=False):
            frames = []
            valid = True
            for cap in caps:
                ret, frame = cap.read()
                if not ret:
                    valid = False
                    break
                frames.append(frame)
            if not valid:
                break

            current_time_sec = frame_idx / fps

            # Create full canvas
            canvas = np.ones((self.target_height, self.target_width, 3), dtype=np.uint8) * 255
            canvas[0:height, 0:width] = frames[0]                        # top-left
            canvas[height:self.target_height, 0:width] = frames[1]  # top-right

            canvas[0:self.target_height, width:self.target_width] = frames[2]

            cv2.line(canvas, (0, height), (width, height), 0, 2)
            cv2.line(canvas, (width, 0), (width, self.target_height), 0, 2)
            
            # Inspection frame
            self.display_boxed_string(canvas, "1", 0, 0)
            # Environment 1
            self.display_boxed_string(canvas, "1", 1150, 570)
            # Assembly frame
            self.display_boxed_string(canvas, "2", 0, height)
            # Environment 2
            self.display_boxed_string(canvas, "2", 1400, 270)

            # -----------------------------------
            # Add text to bottom-right white area
            # -----------------------------------
            white_y_start = int(height)
            white_height = int(height)

            lines = [
                f"Trial ID: {trial_id}",
                f"Run ID: {run_id}",
                f"Time: {self.seconds_to_minutes(int(current_time_sec))} / {self.seconds_to_minutes(time_limit_seconds)}",
                f"Kits: {kits_completed} / {kits_requested}",
                f"Modules: {modules_completed} / {modules_requested}",
                f"Score: {round(score,1)}"
            ]

            # Number of lines
            num_lines = len(lines)

            # Height of one line
            (_, text_height), _ = cv2.getTextSize(lines[0], self.font, self.font_scale, self.thickness)
            line_spacing = int(text_height * 1.5)
            block_height = num_lines * line_spacing

            # Padding from bottom of white area
            pad_bottom = 0

            # Starting y coordinate for the first line
            text_y_start = white_y_start + white_height - pad_bottom - block_height

            text_widths = [cv2.getTextSize(line_text, self.font, self.font_scale, self.thickness)[0][0] for line_text in lines]

            # Draw white background in bottom-right, but left-align text inside it
            self.bordered_rectangle(
                canvas, 
                (self.target_width - max(text_widths) - 40, text_y_start - block_height + 125),
                (self.target_width, self.target_height)
            )

            # Resize to target output resolution
            canvas_resized = cv2.resize(canvas, (self.target_width, self.target_height), interpolation=cv2.INTER_AREA)

            for i, line_text in enumerate(lines):
                # Left-align text inside the white rectangle (with padding from left edge)
                text_x = self.target_width - max(text_widths) - 25
                text_y = text_y_start + i * line_spacing
                cv2.putText(canvas_resized, line_text, (text_x, text_y), self.font, self.font_scale, self.font_color, self.thickness, cv2.LINE_AA)


            # Send to FFmpeg
            process.stdin.write(canvas_resized.tobytes()) # type: ignore

        # --------------------------
        # Cleanup
        # --------------------------
        for cap in caps:
            cap.release()
        process.stdin.close() # type: ignore
        process.wait()
        cv2.destroyAllWindows()
    
    def display_boxed_string(self, canvas, s: str, x: int, y: int):
        (text_width, text_height), _ = cv2.getTextSize(s, self.font, self.label_font_scale, self.label_font_thickness)

        self.bordered_rectangle(
            canvas,
            (x+int(text_width*0.4), y+int(text_height*0.4)),
            (x+int(text_width*1.6), y+int(text_height*1.6))
        )
        cv2.putText(canvas, s, (x+int(text_width * 0.5), y+int(text_height*1.5)), self.font, self.label_font_scale, self.font_color, self.label_font_thickness, cv2.LINE_AA)

    def bordered_rectangle(self, canvas, pt1: tuple[int, int], pt2: tuple[int, int], border_thickness=2):
        cv2.rectangle(canvas, pt1, pt2, (255, 255, 255), -1)
        cv2.rectangle(canvas, (pt1[0]-border_thickness, pt1[1]-border_thickness), (pt2[0]+border_thickness, pt2[1]+border_thickness), (0, 0, 0), border_thickness)

    def seconds_to_minutes(self, seconds: int) -> str:
        return f"{seconds//60:02d}:{int(seconds%60):02d}"