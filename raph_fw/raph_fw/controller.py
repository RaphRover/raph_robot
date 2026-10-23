# Copyright 2026 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Self

import rclpy
from raph_interfaces.srv import GetControllerInfo
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

if TYPE_CHECKING:
    from types import TracebackType

GET_CONTROLLER_INFO_SERVICE = "controller/get_controller_info"
SOFT_REBOOT_SERVICE = "controller/soft_reboot"
SERVICE_TIMEOUT = 5.0
# Maximum time to wait for a single response when retrying a request
RETRY_CALL_TIMEOUT = 2.0
# Time to let graph discovery settle after the first server is found, so that servers from other
# controllers are also discovered before counting them
SERVER_DISCOVERY_SETTLE_TIME = 0.5


class MultipleControllersError(RuntimeError):
    """Raised when a service is provided by more than one controller visible in the ROS graph."""


class ControllerClient:
    """
    Short-lived ROS client for the services exposed by the RaphCore firmware.

    Each instance uses its own ROS context, so the client can be created multiple times within
    the same process. Use it as a context manager to ensure the context is shut down.
    """

    def __init__(self, node_name: str = "raph_fw_controller_client") -> None:
        """Initialize the ROS context and the node."""
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = Node(node_name, context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)

    def __enter__(self) -> Self:
        """Enter the runtime context."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Destroy the node and shut down the ROS context."""
        self.close()

    def close(self) -> None:
        """Destroy the node and shut down the ROS context."""
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()

    def get_controller_info(self, timeout: float = SERVICE_TIMEOUT) -> Any:  # noqa: ANN401
        """
        Retrieve information about the controller, including bootloader and firmware versions.

        The request is retried until a response is received or the timeout expires. This allows
        the call to succeed while the micro-ROS agent is still reconnecting to the controller,
        in which case the service may be advertised but not yet able to respond.

        :param timeout: Maximum time in seconds to wait for a response.
        :returns: The ``GetControllerInfo`` response.
        :raises TimeoutError: If no response is received within the timeout.
        :raises MultipleControllersError: If more than one controller provides the service.
        """
        return self._call(
            GetControllerInfo,
            GET_CONTROLLER_INFO_SERVICE,
            GetControllerInfo.Request(),
            timeout,
            retry=True,
        )

    def soft_reboot(self) -> None:
        """
        Request a soft reboot of the controller.

        :raises TimeoutError: If the service is not available or does not respond in time.
        :raises MultipleControllersError: If more than one controller provides the service.
        :raises RuntimeError: If the controller rejects the reboot request.
        """
        response = self._call(Trigger, SOFT_REBOOT_SERVICE, Trigger.Request(), SERVICE_TIMEOUT)
        if not response.success:
            msg = f"Controller rejected the reboot request: {response.message}"
            raise RuntimeError(msg)

    def _call(
        self,
        srv_type: Any,  # noqa: ANN401
        srv_name: str,
        request: Any,  # noqa: ANN401
        timeout: float,
        *,
        retry: bool = False,
    ) -> Any:  # noqa: ANN401
        """
        Call a service and wait for the response.

        :param timeout: Maximum time in seconds to wait for the service to become available and,
            if ``retry`` is set, for the response.
        :param retry: Whether to resend the request if no response is received, until the timeout
            expires. Only use for idempotent requests.
        :raises TimeoutError: If the service is not available or does not respond in time.
        :raises MultipleControllersError: If more than one controller provides the service, in which
            case the request is not sent, since it would reach all of them.
        """
        deadline = time.monotonic() + timeout
        client = self._node.create_client(srv_type, srv_name)
        settled = False
        try:
            while True:
                remaining = max(deadline - time.monotonic(), 0.0)
                if not client.wait_for_service(timeout_sec=remaining):
                    msg = (
                        f"{srv_name} service not available. "
                        "Check ROS and controller node visibility."
                    )
                    raise TimeoutError(msg)

                if not settled:
                    time.sleep(SERVER_DISCOVERY_SETTLE_TIME)
                    settled = True
                server_count = self._node.count_services(srv_name)
                if server_count > 1:
                    msg = (
                        f"{srv_name} service is provided by {server_count} controllers. "
                        "Cannot determine which one is the target device."
                    )
                    raise MultipleControllersError(msg)

                call_timeout = SERVICE_TIMEOUT
                if retry:
                    remaining = max(deadline - time.monotonic(), 0.0)
                    call_timeout = min(RETRY_CALL_TIMEOUT, remaining)

                future = client.call_async(request)
                self._executor.spin_until_future_complete(future, timeout_sec=call_timeout)
                response = future.result() if future.done() else None
                if response is not None:
                    return response

                client.remove_pending_request(future)
                if not retry or time.monotonic() >= deadline:
                    msg = f"{srv_name} service did not respond in time."
                    raise TimeoutError(msg)
        finally:
            self._node.destroy_client(client)
