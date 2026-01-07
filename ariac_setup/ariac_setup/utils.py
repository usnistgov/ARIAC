import math
import re
import asyncio

from typing import Callable, TypeVar, Any

from rclpy.task import Future
from rclpy.client import Client
from rclpy.executors import Executor

T = TypeVar('T')

class ROSAsyncAdapter:
    @staticmethod
    def ros_future_to_asyncio_future(ros_future: Future) -> asyncio.Future:
    
        loop = asyncio.get_running_loop()

        asyncio_future = loop.create_future()

        def _on_ros_future_complete(fut: Future):
            if fut.cancelled():
                asyncio_future.cancel()
            elif fut.done():
                exc = fut.exception()
                if exc is not None:
                    loop.call_soon_threadsafe(asyncio_future.set_exception, exc)
                else:
                    loop.call_soon_threadsafe(asyncio_future.set_result, fut.result())

        ros_future.add_done_callback(_on_ros_future_complete)
        
        return asyncio_future

    @staticmethod
    async def await_service_ready(client: Client, timeout: float = 20.0, poll_interval: float = 0.1):
        start_time = asyncio.get_running_loop().time()
        while not client.service_is_ready():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError(f"Timed out waiting for service {client.srv_name} to become available")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def await_service_response(client: Client, request, timeout: float = 20.0):
        future = ROSAsyncAdapter.ros_future_to_asyncio_future(client.call_async(request))

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f'Timed out waiting for response from {client.srv_name} service.')
    
    @staticmethod
    async def spin_executor(executor: Executor, shutdown_event: asyncio.Event, spin_timeout_sec: float = 0.01):
        while not shutdown_event.is_set():
            executor.spin_once(timeout_sec=spin_timeout_sec)
            await asyncio.sleep(0.001)

    @staticmethod
    async def await_condition(condition_fn: Callable[[], bool], timeout: float = 20.0, poll_interval: float = 0.1):
        start_time = asyncio.get_running_loop().time()
        while not condition_fn():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError("Condition not met within timeout.")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def call_blocking_with_timeout(func: Callable[..., T], *args: Any, timeout: float = 20.0) -> T:
        loop = asyncio.get_running_loop()
        future = loop.run_in_executor(None, func, *args)

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError("Blocking function call timed out")
        
    @staticmethod
    async def wait_for_gz_clock():
        ''' Waits for gz clock to start publishing '''
        
        process = await asyncio.create_subprocess_exec(
            'gz', 'topic', '-e', '-t', '/clock',
            stdout=asyncio.subprocess.PIPE,
        )
        
        if process.stdout is None:
            raise RuntimeError("Unable to read from gz topic stdin")
        
        await process.stdout.readline()
        process.terminate()
        await process.wait()
            


def evaluate_pi_expression(expr) -> float:
    if isinstance(expr, (int, float)):
        return float(expr)

    if isinstance(expr, str):
        try:
            # Remove spaces and prepare expression
            expr = expr.replace(' ', '')
            expr = re.sub(r'(\d)(pi)', r'\1*pi', expr)
            
            # Allow only valid characters/operators
            if not re.fullmatch(r'[\d\.\+\-\*/\(\)pi]+', expr):
                raise ValueError("Invalid characters in expression.")
            
            # Replace 'pi' with its float value
            expr = expr.replace('pi', str(math.pi))

            # Evaluate using eval with empty builtins
            result = eval(expr, {"__builtins__": {}})
            return float(result)

        except Exception as e:
            print(e)
    
    return 0.0

