from dataclasses import dataclass, field

from enum import IntEnum

@dataclass
class Trial:
    seed: int
    time_limit: int
    num_kits: int
    num_modules: int
    num_high_priority: int
    trial_id: str

@dataclass
class Run:
    id: int
    trial_id: int
    competitor_id: int
    completed: bool
    aborted: bool
    sensor_cost: float
    duration: float
    total_cells: int
    defective_cells: int
    avg_report_time: float
    num_reports_submitted: int
    num_correct_reports: int
    num_correct_report_classifications: int

@dataclass
class OrderSubmission:
    order_type: int
    submission_time: float
    announcement_time: float

@dataclass
class Penalty:
    type: int
    description: str
    time: float

class OrderType(IntEnum):
    KIT = 1
    MODULE = 2
    HIGH_PRIORITY = 3
    UNKNOWN = -1

@dataclass 
class Weights:
    W1: float
    W2: float
    W3: float
    W4: float
    W5: float
    W6: float
    W7: float

class PenaltyType(IntEnum):
    GOOD_CELL_IN_INSPECTION_BIN = 0
    CELL_IN_CONVEYOR_BIN = 1
    OBJECT_ON_INVALID_SURFACE = 2
    AGV_COLLISION = 3
    ROBOT_COLLISION = 4

@dataclass
class PenaltyDeductions:
    GOOD_CELL_IN_INSPECTION_BIN: int
    CELL_IN_CONVEYOR_BIN: int
    OBJECT_ON_INVALID_SURFACE: int
    AGV_COLLISION: int
    ROBOT_COLLISION: int

    def get_deduction(self, p_type: PenaltyType):
        return getattr(self, p_type.name)

@dataclass
class BonusResult:
    description: str
    amount: float

    def __str__(self):
        return f"{self.description}: {self.amount}"

@dataclass
class BonusResults:
    b1: BonusResult = field(default_factory=lambda: BonusResult("Trial time bonus", 0.0))
    b2: BonusResult = field(default_factory=lambda: BonusResult("Inspection speed bonus", 0.0))
    b3: BonusResult = field(default_factory=lambda: BonusResult("High priority order speed bonus", 0.0))
    b4: BonusResult = field(default_factory=lambda: BonusResult("Sensor bonus", 0.0))
    b5: BonusResult = field(default_factory=lambda: BonusResult("Inspection classification bonus", 0.0))

    def __str__(self):
        return "\n".join([str(getattr(self, f"b{i}")) for i in range(1, 6)])
    
    def total(self) -> float:
        return sum([getattr(self, f"b{i}").amount for i in range(1, 6)])

@dataclass
class PenaltyResult:
    description: str
    count: int
    total_deduction: float

    def __str__(self):
        return f"{self.description}: number of occurrences: {self.count} | total deduction: {self.total_deduction}"
@dataclass
class PenaltyResults:
    p1: PenaltyResult = field(default_factory=lambda: PenaltyResult("Non-defective cell in inspection bin", 0 , 0.0))
    p2: PenaltyResult = field(default_factory=lambda: PenaltyResult("Cell in conveyor bin", 0 , 0.0))
    p3: PenaltyResult = field(default_factory=lambda: PenaltyResult("Object on invalid surface", 0 , 0.0))
    p4: PenaltyResult = field(default_factory=lambda: PenaltyResult("AGV collision", 0 , 0.0))
    p5: PenaltyResult = field(default_factory=lambda: PenaltyResult("Robot collision", 0 , 0.0))
    p6: PenaltyResult = field(default_factory=lambda: PenaltyResult("Sensor cost over budget", 0 , 0.0))

    def __str__(self):
        return "\n".join([str(getattr(self, f"p{i}")) for i in range(1, 7)])

    def total(self) -> float:
        return sum([getattr(self, f"p{i}").total_deduction for i in range(1, 7)])