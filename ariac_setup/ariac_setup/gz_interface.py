import asyncio
import rclpy
import argparse

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from gz.msgs10 import boolean_pb2, world_control_pb2
from gz.transport13 import Node as GzNode

from ariac_setup.utils import ROSAsyncAdapter
from ariac_setup.controller_starter import ControllerStarter
from ariac_setup.param_setter import ParamSetter
from ariac_setup.user_config_parser import UserConfigParser


class GZInterface(Node):
    def __init__(self):
        super().__init__('gz_interface_node')

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

        self._gz_node = GzNode()
        
        self.stats_topic_received = asyncio.Event()

        self._world_control_srv_name = "/world/ariac/control"
    
    async def unpause_simulation(self):
        # Wait for GZ service to be ready
        await ROSAsyncAdapter.await_condition(lambda: self._world_control_srv_name in self._gz_node.service_list())

        req = world_control_pb2.WorldControl()
        req.pause = False

        success, response = self._gz_node.request(
            self._world_control_srv_name,
            req,
            world_control_pb2.WorldControl,
            boolean_pb2.Boolean,
            1000
        )

        if not success:
            raise RuntimeError(f'Error when unpausing simulation success')
        elif not response.data:
            raise RuntimeError("Error response.data")

async def run(user_config_path: str):

    if not user_config_path:
        raise Exception("User config not set")

    rclpy.init()

    user_config = UserConfigParser(user_config_path)

    controller_starter = ControllerStarter()
    param_setter = ParamSetter()
    gz_interface = GZInterface()

    executor = MultiThreadedExecutor()
    executor.add_node(controller_starter)
    executor.add_node(param_setter)
    executor.add_node(gz_interface)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))

    try:
        await gz_interface.unpause_simulation()

        await controller_starter.load_controllers()
        await controller_starter.configure_controllers()
        await controller_starter.switch_controllers()

        await param_setter.set_param('inspection_conveyor/linear_conveyor_node', 'conveyor_speed', user_config.conveyor_speed)
        await param_setter.set_param('inspection_conveyor/door/linear_conveyor_node', 'conveyor_speed', user_config.conveyor_speed)
        await param_setter.set_param('inspection_conveyor/cell_feed/cell_feed_plugin', 'feed_rate', user_config.cell_feed_rate)

        await param_setter.set_param('competition_manager_plugin', 'sensors_ready', True)
        await param_setter.set_param('competition_manager_plugin', 'controllers_ready', True)

    except Exception as e:
        print(e)
    finally:
        shutdown_event.set()
        await spin_task

    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--user-config-path', type=str, default='')
    args, _ = parser.parse_known_args()

    asyncio.run(run(user_config_path=args.user_config_path))