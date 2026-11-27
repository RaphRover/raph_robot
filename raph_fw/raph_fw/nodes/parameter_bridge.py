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

from raph_interfaces.srv import ApplyTfFramePrefix
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

WAIT_TIMEOUT = 2.0


class ParameterBridge(Node):
    """Node that loads parameter overrides and forwards them to the controller node."""

    target_node: str
    apply_tf_frame_prefix_client: Client
    get_parameters_client: Client
    set_parameters_client: Client
    tf_frame_prefix: str
    params_to_set: dict[str, Parameter]
    service_cb_group: MutuallyExclusiveCallbackGroup
    cancel_timer_cb_group: MutuallyExclusiveCallbackGroup

    def __init__(self) -> None:
        """Create and initialize the controller parameter bridge node."""
        super().__init__("controller_parameter_bridge")

        self.declare_parameter(
            "target_node_name",
            "controller",
            ParameterDescriptor(read_only=True),
        )

        self.declare_parameter(
            "tf_frame_prefix",
            "",
            ParameterDescriptor(read_only=True),
        )

        self.target_node = self.get_parameter("target_node_name").value.strip("/")
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.cancel_timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.apply_tf_frame_prefix_client = self.create_client(
            ApplyTfFramePrefix,
            f"{self.target_node}/apply_tf_frame_prefix",
            callback_group=self.service_cb_group,
        )
        self.tf_frame_prefix = self.get_parameter("tf_frame_prefix").value

        self.get_parameters_client = self.create_client(
            GetParameters,
            f"{self.target_node}/get_parameters",
            callback_group=self.service_cb_group,
        )

        self.set_parameters_client = self.create_client(
            SetParameters,
            f"{self.target_node}/set_parameters",
            callback_group=self.service_cb_group,
        )
        self.params_to_set: dict[str, Parameter] = self._parse_overrides_file()
        self.params_to_set["is_configured"] = Parameter("is_configured", value=True)

        self.startup_config_timer = self.create_timer(
            0.1,
            self.startup_config_apply,
        )
        self.is_configured_timer = self.create_timer(5.0, self.check_is_configured)

        self.get_logger().info(
            f"Parameter bridge initialized for {self.target_node} target node",
        )

    async def check_is_configured(self) -> None:
        """Reapply controller configuration when firmware reports it is unconfigured."""
        if await self._should_configure():
            self.get_logger().info(
                "Controller firmware is not configured. Applying configuration...",
            )
            await self.apply_firmware_config()

    async def startup_config_apply(self) -> None:
        """Apply firmware configuration during startup."""
        self.startup_config_timer.cancel()
        self.destroy_timer(self.startup_config_timer)
        await self.apply_firmware_config()

    async def apply_firmware_config(self) -> None:
        """Apply firmware configuration."""
        await self.apply_controller_tf_frame_prefix()
        await self.apply_parameter_overrides()

    async def _should_configure(self) -> bool:
        """Determine if the controller firmware should be configured."""
        if not self.get_parameters_client.service_is_ready():
            self.get_logger().info(
                f"Service {self.get_parameters_client.srv_name} not ready, skipping the check.",
            )
            return False

        request = GetParameters.Request()
        request.names = ["is_configured"]
        future = await self._call_with_retry(self.get_parameters_client, request)

        response = future.result()
        if not response.values or response.values[0].type != Parameter.Type.BOOL.value:
            self.get_logger().info(
                f"No 'is_configured' parameter found for {self.target_node}. "
                "Check if firmware updates are available.",
            )
            return False

        return not response.values[0].bool_value

    async def apply_controller_tf_frame_prefix(self) -> bool:
        """Wait for target service and apply TF frame prefix to the controller node."""
        self.get_logger().info("Waiting for apply_tf_frame_prefix service to be available...")
        while not self.apply_tf_frame_prefix_client.wait_for_service(timeout_sec=WAIT_TIMEOUT):
            self.get_logger().info(
                f"Service {self.apply_tf_frame_prefix_client.srv_name} not available yet, "
                "retrying...",
            )
        self.get_logger().info(
            f"{self.apply_tf_frame_prefix_client.srv_name} service is available. "
            "Applying TF frame prefix...",
        )

        request = ApplyTfFramePrefix.Request()
        request.tf_frame_prefix = self.tf_frame_prefix
        future = await self._call_with_retry(self.apply_tf_frame_prefix_client, request)

        response = future.result()
        if response is None:
            self.get_logger().error("ApplyTfFramePrefix call failed or returned no result.")
            return False

        if not response.success:
            self.get_logger().error(f"Failed to apply TF frame prefix: {response.message}")
            return False

        self.get_logger().info(
            f"Applied TF frame prefix '{self.tf_frame_prefix}' to {self.target_node} node",
        )
        return True

    async def apply_parameter_overrides(self) -> bool:
        """Wait for target service and set all parsed parameters."""
        if not self.params_to_set:
            self.get_logger().info("No parameters to apply.")
            return True

        self.get_logger().info("Waiting for set_parameters service to be available...")
        while not self.set_parameters_client.wait_for_service(timeout_sec=WAIT_TIMEOUT):
            self.get_logger().info(
                f"Service {self.set_parameters_client.srv_name} not available yet, retrying...",
            )
        self.get_logger().info(
            f"{self.set_parameters_client.srv_name} service is available. Applying parameters...",
        )

        request = SetParameters.Request()
        request.parameters = [
            parameter.to_parameter_msg() for parameter in self.params_to_set.values()
        ]
        future = await self._call_with_retry(self.set_parameters_client, request)

        response = future.result()
        if response is None:
            self.get_logger().error("SetParameters call failed or returned no result.")
            return False

        results = response.results
        failed = [
            (name, result.reason)
            for name, result in zip(self.params_to_set.keys(), results, strict=True)
            if not result.successful
        ]
        if failed:
            for name, reason in failed:
                self.get_logger().error(f"Failed to set '{name}': {reason}")
            return False

        self.get_logger().info(
            f"Applied {len(self.params_to_set)} parameter(s) to {self.target_node} node",
        )
        return True

    async def _call_with_retry(
        self,
        client: Client,
        request: ApplyTfFramePrefix.Request | GetParameters.Request | SetParameters.Request,
    ) -> Future:
        """Retry timed out async service calls until the future completes."""
        while True:
            future = client.call_async(request)
            timeout_timer = self.create_timer(
                5.0,
                future.cancel,
                callback_group=self.cancel_timer_cb_group,
            )
            await future
            timeout_timer.cancel()
            self.destroy_timer(timeout_timer)

            if future.done() and future.result() is not None:
                return future

            self.get_logger().info(
                f"Service call to {client.srv_name} timed out, retrying...",
            )

    def _parse_overrides_file(self) -> dict[str, Parameter]:
        """
        Parse parameter overrides using rcl's own parameter parser.

        The bridge itself is launched with ``--params-file``, so the overrides file
        is already parsed into this process' global ROS arguments. Rather than
        reading and interpreting that YAML ourselves, a throwaway probe node is
        created with the same name/namespace the target (controller) node would
        have. This makes rcl match the node sections in the inherited global
        arguments (including ``/**`` wildcards and ``/**/<name>`` patterns) exactly
        the way it would for the real target node, so the overrides file must
        follow the standard ROS 2 parameters YAML layout, e.g.::

            /**/controller:
              ros__parameters:
                param1: 10

        Ad-hoc ``-p name:=value`` overrides are stored by rcl under a wildcard that
        matches *any* node name in the process, so they would also show up in the
        probe node's overrides even though they're meant for the bridge itself.
        Since they equally apply to this node, they're filtered out by excluding
        any name already present in this node's own ``_parameter_overrides``.

        The probe node is destroyed immediately after its parameter overrides are
        read; it never advertises services and is not spun.
        """
        try:
            probe_node = Node(
                self.target_node,
                namespace=self.get_namespace(),
                context=self.context,
                start_parameter_services=False,
                enable_rosout=False,
            )
        except Exception as exc:  # noqa: BLE001 - boundary with rcl's global args parser
            self.get_logger().error(f"Failed to parse parameter overrides: {exc}")
            return {}

        try:
            return {
                name: parameter
                for name, parameter in probe_node._parameter_overrides.items()  # noqa: SLF001
                if name not in self._parameter_overrides
            }
        finally:
            probe_node.destroy_node()
