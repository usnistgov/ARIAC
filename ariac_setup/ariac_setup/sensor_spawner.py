import os

from rclpy.node import Node

from rclpy.parameter import Parameter

from gz.msgs10 import entity_factory_pb2, boolean_pb2
from gz.transport13 import Node as GzNode

from ament_index_python import get_package_share_directory

from ariac_setup.user_config_parser import Sensor, ParsingError
from ariac_setup.utils import ROSAsyncAdapter


class SensorSpawner(Node):
    def __init__(self, sensors: list[Sensor]):
        super().__init__('sensor_spawner_node')

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])
        
        self.sensors_to_spawn = sensors
        self._gz_node = GzNode()
        
        self._srv_name = '/world/ariac/create'

    async def spawn_sensors(self):
        # Wait for GZ service to be ready
        await ROSAsyncAdapter.await_condition(lambda: self._srv_name in self._gz_node.service_list())

        for sensor in self.sensors_to_spawn:
            req = self._entity_factory_from_sensor(sensor)

            success, response = self._gz_node.request(
                self._srv_name,
                req,
                entity_factory_pb2.EntityFactory,
                boolean_pb2.Boolean,
                1000
            )
        
            if not success or not response.data:
                raise ParsingError(f'Error when creating sensor: {sensor.name}')
            
            self.get_logger().info(f"Added sensor {sensor.name} publishing to {' and '.join(sensor.topics)}")

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