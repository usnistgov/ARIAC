from typing import cast

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from ariac_setup.utils import ROSAsyncAdapter

from ariac_interfaces.msg import (
    CompetitionTime,
    CompetitionStatus,
    CompetitionStates
)
from ariac_interfaces.srv import (
    EndCompetition,
    Trigger
)

class AppNode(Node):
    def __init__(self):
        super().__init__("app_node")

        self._current_state: int | None = None
        self._competition_time: CompetitionTime | None = None
        self._kits_remaining: int | None = None
        self._modules_remaining: int | None = None
        self._total_kits: int | None = None
        self._total_modules: int | None = None
        self._run_id: int | None = None

        # Service clients
        self.start_competition_client = self.create_client(Trigger, "/start_competition")
        self.end_competition_client = self.create_client(EndCompetition, "/end_competition")

        # Subscribers
        self.competition_state_sub = self.create_subscription(
            CompetitionStatus,
            "/competition_status",
            self.competition_status_cb,
            10
        )
    
    @property
    def current_state(self) -> int | None:
        return self._current_state
    
    @property
    def time_elapsed(self) -> float | None:
        if self._competition_time is None:
            return None
        
        return Duration.from_msg(self._competition_time.elapsed).nanoseconds / 1E9

    @property
    def time_remaining(self) -> float | None:
        if self._competition_time is None:
            return None
        
        return Duration.from_msg(self._competition_time.remaining).nanoseconds / 1E9
    
    @property
    def kits_remaining(self) -> int | None:
        return self._kits_remaining

    @property
    def modules_remaining(self) -> int | None:
        return self._modules_remaining
    
    @property
    def total_kits(self) -> int | None:
        return self._total_kits

    @property
    def total_modules(self) -> int | None:
        return self._total_modules
    
    @property
    def run_id(self) -> int | None:
        return self._run_id
    
    @property
    def current_sim_time(self) -> Time:
        return self.get_clock().now()
    
    @property
    def time_limit(self) -> int | None:
        if self._competition_time is None:
            return None
        return round(Duration.from_msg(self._competition_time.elapsed).nanoseconds / 1E9 + Duration.from_msg(self._competition_time.remaining).nanoseconds / 1E9)
    
    @property
    def competition_time(self) -> CompetitionTime | None:
        return self._competition_time

    def competition_status_cb(self, msg: CompetitionStatus):
        self._current_state = msg.competition_state
        self._competition_time = msg.time

        if msg.competition_state in [CompetitionStates.STARTED, CompetitionStates.ORDERS_COMPLETE]:
            self._kits_remaining = msg.num_kits_remaining
            self._modules_remaining = msg.num_modules_remaining
            self._total_kits = msg.num_kits
            self._total_modules = msg.num_modules
            self._run_id = msg.run_id

    async def start_competition(self) -> tuple[bool, str]:
        try:
            await ROSAsyncAdapter.await_service_ready(self.start_competition_client, timeout=2)
        except TimeoutError:
            return False, "Unable to reach start_competition service"
        
        try:
            result = await ROSAsyncAdapter.await_service_response(self.start_competition_client, Trigger.Request(), 5)
        except TimeoutError:
            return False, "Start competition request timed out"
            
        response = cast(Trigger.Response, result)
        
        if response.success:
            return True, "Competition started"
        else:
            return False, response.message        
    
    async def end_competition(self, shutdown_gazebo: bool = True) -> tuple[bool, str]:
        try:
            await ROSAsyncAdapter.await_service_ready(self.end_competition_client, timeout=2)
        
        except TimeoutError:
            return False, "Unable to reach end_competition service"
        
        req = EndCompetition.Request()
        req.shutdown_gazebo = shutdown_gazebo

        try:    
            result = await ROSAsyncAdapter.await_service_response(self.end_competition_client, req, 5)
        except TimeoutError as e:
            return False, "End competition request timed out"
            
        response = cast(Trigger.Response, result)
        
        if response.success:
            return True, "Competition ended"
        else:
            return False, response.message
        
    def reset(self):
        self._current_state = None
        self._competition_time = None
        self._kits_remaining = None
        self._modules_remaining = None
        self._total_kits = None
        self._total_modules = None
        self._run_id = None
        