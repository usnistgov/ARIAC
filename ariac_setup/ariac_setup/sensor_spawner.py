import os
import asyncio

import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rclpy.parameter import Parameter

from gz.msgs10 import entity_factory_pb2, boolean_pb2, world_stats_pb2
from gz.transport13 import Node as GzNode

from ament_index_python import get_package_share_directory

from ariac_setup.user_config_parser import Sensor, ParsingError
from ariac_setup.utils import ROSAsyncAdapter
from ariac_setup.user_config_parser import UserConfigParser


class SensorSpawner(Node):
    def __init__(self, sensors: list[Sensor]):
        super().__init__('sensor_spawner_node')

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])
        
        self.sensors_to_spawn = sensors
        self._gz_node = GzNode()
        
        self.stats_topic_received = asyncio.Event()

        self._srv_name = '/world/ariac/create'
        self._stats_topic_name = "/stats"

        self._gz_node.subscribe(
            topic=self._stats_topic_name,
            msg_type=world_stats_pb2.WorldStatistics, 
            callback=self.gz_stats_cb
        )

    def gz_stats_cb(self, msg):
        self.stats_topic_received.set()

    async def spawn_sensors(self):
        # Wait for GZ service to be ready
        await ROSAsyncAdapter.await_condition(lambda: self._srv_name in self._gz_node.service_list())

        # await self.stats_topic_received.wait()

        for sensor in self.sensors_to_spawn:
            req = self._entity_factory_from_sensor(sensor)

            success, response = self._gz_node.request(
                self._srv_name,
                req,
                entity_factory_pb2.EntityFactory,
                boolean_pb2.Boolean,
                1000
            )
            
            # success, response = await ROSAsyncAdapter.call_blocking_with_timeout(
            #     self._gz_node.request,
            #     self._srv_name,
            #     req,
            #     entity_factory_pb2.EntityFactory,
            #     boolean_pb2.Boolean,
            #     1000,  
            #     timeout=2.0 
            # )

            if not success or not response.data:
                raise ParsingError(f'Error when creating sensor: {sensor.name}')

    def _entity_factory_from_sensor(self, sensor: Sensor):
        ef = entity_factory_pb2.EntityFactory()
        ef.name = sensor.name

        share_dir = get_package_share_directory("ariac_gz")
        sdf_path = os.path.join(share_dir, "models", "sensors" , str(sensor.sensor_type), "model.sdf")

        ef.sdf = sensor.get_xml(sdf_path)

        ef.pose.position.x = sensor.pose.position.x
        ef.pose.position.y = sensor.pose.position.y
        ef.pose.position.z = sensor.pose.position.z
        ef.pose.orientation.x = sensor.pose.orientation.x
        ef.pose.orientation.y = sensor.pose.orientation.y
        ef.pose.orientation.z = sensor.pose.orientation.z
        ef.pose.orientation.w = sensor.pose.orientation.w

        return ef
    
async def run(user_config_path: str):

    if not user_config_path:
        raise Exception("User config not set")

    rclpy.init()

    user_config = UserConfigParser(user_config_path)

    sensor_spawner = SensorSpawner(user_config.sensors)

    executor = MultiThreadedExecutor()
    executor.add_node(sensor_spawner)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))

    try:        
        await asyncio.sleep(3.0)

        await sensor_spawner.spawn_sensors()

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