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

import rclpy
from raph_interfaces.srv import ApplyTfFramePrefix
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParameters
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

SERVICE_TIMEOUT = 2.0


class ParameterBridge(Node):
    """Node that loads parameter overrides and forwards them to the controller node."""

    target_node: str
    apply_tf_frame_prefix_client: Client
    set_parameters_client: Client
    tf_frame_prefix: str
    params_to_set: dict[str, Parameter]

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

        self.apply_tf_frame_prefix_client = self.create_client(
            ApplyTfFramePrefix,
            f"{self.target_node}/apply_tf_frame_prefix",
        )
        self.tf_frame_prefix = self.get_parameter("tf_frame_prefix").value

        self.set_parameters_client = self.create_client(
            SetParameters,
            f"{self.target_node}/set_parameters",
        )
        self.params_to_set: dict[str, Parameter] = self._parse_overrides_file()

        self.get_logger().info(
            f"Parameter bridge initialized for {self.target_node} target node",
        )

    def run(self) -> None:
        """Perform all the tasks of the parameter bridge."""
        self.apply_controller_tf_frame_prefix()
        self.apply_parameter_overrides()

    def apply_controller_tf_frame_prefix(self) -> bool:
        """Wait for target service and apply TF frame prefix to the controller node."""
        self.get_logger().info("Waiting for apply_tf_frame_prefix service to be available...")
        while not self.apply_tf_frame_prefix_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
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
        future = self._call_with_retry(self.apply_tf_frame_prefix_client, request)

        if future.result() is None:
            self.get_logger().error("ApplyTfFramePrefix call failed or returned no result.")
            return False

        if not future.result().success:
            self.get_logger().error(f"Failed to apply TF frame prefix: {future.result().message}")
            return False

        self.get_logger().info(
            f"Applied TF frame prefix '{self.tf_frame_prefix}' to {self.target_node} node",
        )
        return True

    def apply_parameter_overrides(self) -> bool:
        """Wait for target service and set all parsed parameters."""
        if not self.params_to_set:
            self.get_logger().info("No parameters to apply. Exiting.")
            return True

        self.get_logger().info("Waiting for set_parameters service to be available...")
        while not self.set_parameters_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
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
        future = self._call_with_retry(self.set_parameters_client, request)

        if future.result() is None:
            self.get_logger().error("SetParameters call failed or returned no result.")
            return False

        results = future.result().results
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

    def _call_with_retry(
        self,
        client: Client,
        request: ApplyTfFramePrefix.Request | SetParameters.Request,
    ) -> Future:
        """Retry timed out async service calls until the future completes."""
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)

            if future.done():
                return future

            self.get_logger().info(
                f"Service call to {client.srv_name} timed out, retrying...",
            )
            future.cancel()

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
