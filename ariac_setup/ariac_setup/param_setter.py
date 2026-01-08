from typing import cast, Any

from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

from ariac_setup.utils import ROSAsyncAdapter


class ParamSetter(Node):
    def __init__(self):
        super().__init__('param_setter_node')

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

    async def set_param(self, target_node_name: str, param_name: str, param_value: Any, timeout: float = 10.0) -> bool:

        # Create a client to the target node's parameter service
        client = self.create_client(srv_type=SetParameters, srv_name=f'/{target_node_name}/set_parameters')

        # Wait for service to be ready (async)
        await ROSAsyncAdapter.await_service_ready(client, timeout=timeout)

        # Create parameter message
        param = Parameter(param_name, Parameter.Type.from_parameter_value(param_value), param_value)

        # Build the request message with the parameter
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        # Call the service asynchronously and wait for response with timeout
        response = cast(SetParameters.Response, await ROSAsyncAdapter.await_service_response(client, request, timeout=timeout))
        results = cast(list[SetParametersResult], response.results)

        # Return whether the parameter was set successfully
        return all(result.successful for result in results)