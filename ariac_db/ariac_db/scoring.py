import yaml
import os

from ament_index_python import get_package_share_directory

from ariac_db.structures import (
  BonusResults,
  Trial, 
  Run, 
  OrderSubmission,
  OrderType,
  Penalty, 
  PenaltyResults,
  PenaltyType,
  PenaltyDeductions, 
  Weights
)

class ARIACScorer:
    def __init__(self):
        with open(os.path.join(get_package_share_directory("ariac_db"), "config", "scoring_weights.yaml")) as f:
            scoring_constants = yaml.safe_load(f)
        
        self.sigma_b = scoring_constants["SENSOR_BUDGET"]
        self.gamma_d = scoring_constants["NOMINAL_INSPECTION_TIME"]
        self.tau_d = scoring_constants["NOMINAL_HIGH_PRIORITY_EXECUTION_DURATION"]
        self.deductions = PenaltyDeductions(**scoring_constants["PENALTIES"])
        self.weights = Weights(**scoring_constants["WEIGHTS"])
        
    def score_penalties(self, run: Run, penalties: list[Penalty]) -> PenaltyResults:
        results = PenaltyResults()
        for penalty in penalties:
            deduction = self.deductions.get_deduction(PenaltyType(penalty.type))
            match(penalty.type):
                case PenaltyType.GOOD_CELL_IN_INSPECTION_BIN:
                    results.p1.total_deduction += deduction
                    results.p1.count +=1
                case PenaltyType.CELL_IN_CONVEYOR_BIN:
                    results.p2.total_deduction += deduction
                    results.p2.count +=1
                case PenaltyType.OBJECT_ON_INVALID_SURFACE:
                    results.p3.total_deduction += deduction
                    results.p3.count +=1
                case PenaltyType.AGV_COLLISION:
                    results.p4.total_deduction += deduction
                    results.p4.count +=1
                case PenaltyType.ROBOT_COLLISION:
                    results.p5.total_deduction += deduction
                    results.p5.count +=1

        results.p6.total_deduction = max(self.weights.W6 * ((run.sensor_cost / self.sigma_b) - 1), 0)
        results.p6.count = 0 if results.p6.total_deduction == 0 else run.sensor_cost - self.sigma_b
        
        return results

    def score_bonuses(self, run: Run, trial: Trial, orders: list[OrderSubmission]) -> BonusResults:
        results = BonusResults()

        # Set bonuses to zero if run was aborted
        if run.aborted:
            results.b1.amount = 0.0
            results.b2.amount = 0.0
            results.b3.amount = 0.0
            results.b4.amount = 0.0
            results.b5.amount = 0.0
            
            return results

        # Trial time bonus
        results.b1.amount = self.weights.W3 * (1 - (run.duration / trial.time_limit)) if not run.aborted else 0
        
        # Inspection speed bonus
        results.b2.amount = max(0, self.weights.W4 * (1 - run.avg_report_time / self.gamma_d))

        # High priority order speed bonus
        high_priority_orders = [o for o in orders if o.order_type == OrderType.HIGH_PRIORITY]
        if len(high_priority_orders) == 0:
            results.b3.amount = 0
        else:
            tau = sum([o.submission_time - o.announcement_time for o in high_priority_orders])/len(high_priority_orders)
            results.b3.amount = max(0, self.weights.W5 * (1 - tau / self.tau_d))
        
        # Sensor bonus
        results.b4.amount = max(0, self.weights.W6 * (1 - (run.sensor_cost / self.sigma_b))) if not run.aborted else 0

        # Inspection classification bonus
        if run.defective_cells == 0:
            results.b5.amount = 0
        else:
            results.b5.amount = self.weights.W7 * (run.num_correct_report_classifications / run.defective_cells)

        return results

    def score_kits(self, orders: list[OrderSubmission], trial: Trial) -> float:
        if trial.num_kits + trial. num_high_priority == 0:
            return 0
        return self.weights.W1 * len([o for o in orders if o.order_type in [OrderType.KIT, OrderType.HIGH_PRIORITY]]) / (trial.num_kits + trial. num_high_priority)
    
    def score_modules(self, orders: list[OrderSubmission], trial: Trial) -> float:
        if trial.num_modules == 0:
            return 0
        
        return self.weights.W2 * len([o for o in orders if o.order_type == OrderType.MODULE]) / trial.num_modules
    
    def score_run(self, run: Run, trial: Trial, orders: list[OrderSubmission], penalties: list[Penalty]) -> float:
        total_score = 0

        total_score += self.score_kits(orders, trial)
        
        total_score += self.score_modules(orders, trial)

        total_score += self.score_bonuses(run, trial, orders).total()

        total_score -= self.score_penalties(run, penalties).total()
        
        return total_score