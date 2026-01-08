import asyncio
import argparse

from pathlib import Path
from tabulate import tabulate

import rclpy
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.node import Node

from ariac_interfaces.msg import CompetitionStates, CompetitionStatus

from ariac_db.manager import DatabaseManager, DatabaseError
from ariac_db.scoring import ARIACScorer

from ariac_setup.utils import ROSAsyncAdapter

def print_format(s: str, results_width=60):
    print("|"+f"{s:^{results_width}}"+"|")

class ScoreLogger:
    def __init__(self, db_path: str):
        try:
            self.db_manager = DatabaseManager(Path(db_path))
        except DatabaseError as e:
            print("Unable to connect to database")
            return

        self.competition_ended = asyncio.Event()
    
    def get_results_str(self, run_id: int) -> str|None:        
        if (run := self.db_manager.get_run(run_id)) is None:
            print(f"Could not find run with id {run_id} in database")
            return None
        
        if not run.completed:
            print("Unable to score run since run was not properly completed")
            return None

        if (trial := self.db_manager.get_trial_for_run(run)) is None:
            print(f"Could not find trial for run with id {run_id} in database")
            return None

        orders = self.db_manager.get_orders_for_run(run_id)
        penalties = self.db_manager.get_penalties_for_run(run_id)

        scorer = ARIACScorer()
        bonuses = scorer.score_bonuses(run, trial, orders)
        penalty_results = scorer.score_penalties(run, penalties)

        run_score_table = scorer.get_run_score_table(run, trial, orders, penalties)
        bonus_score_table = scorer.get_bonus_score_table(bonuses)
        penalty_score_table = scorer.get_penalty_score_table(penalty_results)

        s = ""
        s += "\n"+run_score_table+"\n\n"
        s += "\n"+bonus_score_table+"\n\n"
        s += "\n"+penalty_score_table+"\n\n"
        return s
    
class ScoreLoggerNode(Node):
    def __init__(self, db_path: str):
        super().__init__('score_logger')
        self.score_logger = ScoreLogger(db_path)
        self.subscription = self.create_subscription(CompetitionStatus, 'competition_status', self.competition_status_cb, 10)
        self.run_id = -1

        self.competition_ended = asyncio.Event()

    def competition_status_cb(self, msg: CompetitionStatus):
        self.run_id = msg.run_id
        if msg.competition_state==CompetitionStates.ENDED and self.run_id!=-1:
            self.competition_ended.set()
    
    def output_results(self):
        self.get_logger().info(self.score_logger.get_results_str(self.run_id))

async def spin_executor(executor: Executor, shutdown_event: asyncio.Event):
    while not shutdown_event.is_set():
        executor.spin_once()
        await asyncio.sleep(0)

async def run(db_path: str):
    rclpy.init()

    logger = ScoreLoggerNode(db_path)

    executor = MultiThreadedExecutor()
    executor.add_node(logger)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))
    
    try:        
        await logger.competition_ended.wait()

        await asyncio.sleep(2)

        logger.output_results()

    except Exception as e:
        print(e)
    finally:
        shutdown_event.set()
        await spin_task

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--db-path', type=str, default='')
    args, _ = parser.parse_known_args()

    asyncio.run(run(db_path=args.db_path))