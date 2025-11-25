import asyncio
import rclpy
import argparse

from rclpy.executors import MultiThreadedExecutor

from ariac_setup.utils import ROSAsyncAdapter
from ariac_setup.sensor_spawner import SensorSpawner
from ariac_setup.controller_starter import ControllerStarter
from ariac_setup.param_setter import ParamSetter
from ariac_setup.user_config_parser import UserConfigParser


async def run(user_config_path: str):

    if not user_config_path:
        raise Exception("User config not set")

    rclpy.init()

    user_config = UserConfigParser(user_config_path)

    sensor_spawner = SensorSpawner(user_config.sensors)
    controller_starter = ControllerStarter()
    param_setter = ParamSetter()

    executor = MultiThreadedExecutor()
    executor.add_node(sensor_spawner)
    executor.add_node(controller_starter)
    executor.add_node(param_setter)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))

    try:
        await param_setter.set_param('inspection_conveyor/linear_conveyor_node', 'conveyor_speed', user_config.conveyor_speed)
        await param_setter.set_param('inspection_conveyor/door/linear_conveyor_node', 'conveyor_speed', user_config.conveyor_speed)
        await param_setter.set_param('inspection_conveyor/cell_feed/cell_feed_plugin', 'feed_rate', user_config.cell_feed_rate)

        async def _sensor_setup():
            await sensor_spawner.spawn_sensors()
            await param_setter.set_param('competition_manager_plugin', 'sensors_ready', True)
        
        async def _controller_setup():
            await controller_starter.load_controllers()
            await controller_starter.configure_controllers()
            await controller_starter.switch_controllers()
            await param_setter.set_param('competition_manager_plugin', 'controllers_ready', True)
        
        # Run concurrently
        await asyncio.gather(_sensor_setup(), _controller_setup())

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