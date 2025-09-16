from typing import cast

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration

from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController
)

from ariac_setup.utils import ROSAsyncAdapter


   
class ControllerStarter(Node):
    def __init__(self):
        super().__init__("controller_starter_node")

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

        base_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']
        gripper_controller = 'gripper_controller'

        self.robot_controllers = {
            'inspection_robot_1': base_controllers + [gripper_controller],
            'inspection_robot_2': base_controllers + [gripper_controller],
            'assembly_robot_1': base_controllers + [gripper_controller],
            'assembly_robot_2': base_controllers,
            'gantry_welder': base_controllers
        }

    async def load_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(LoadController, f"/{name}/controller_manager/load_controller")
            await ROSAsyncAdapter.await_service_ready(client)
            for controller in controllers:
                req = LoadController.Request()
                req.name = controller

                result = await ROSAsyncAdapter.await_service_response(client, req)

                response = cast(LoadController.Response, result)

                if not response.ok:
                    raise Exception(f"{name} {controller} failed to load")

    async def configure_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(ConfigureController, f"/{name}/controller_manager/configure_controller")
            await ROSAsyncAdapter.await_service_ready(client)
            for controller in controllers:
                req = ConfigureController.Request()
                req.name = controller

                self.get_logger().info(f"Configuring {controller} for {name}")
                result = await ROSAsyncAdapter.await_service_response(client, req)

                response = cast(ConfigureController.Response, result)

                if not response.ok:
                    raise Exception(f"{name} {controller} failed to configure")
    
    async def switch_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(SwitchController, f"/{name}/controller_manager/switch_controller")
            await ROSAsyncAdapter.await_service_ready(client)
            req = SwitchController.Request()
            req.activate_controllers = controllers
            req.strictness = 1
            req.timeout = Duration(seconds=10).to_msg()
            
            result = await ROSAsyncAdapter.await_service_response(client, req)

            response = cast(SwitchController.Response, result)

            if not response.ok:
                raise Exception(f"{name} controllers failed to activate")
