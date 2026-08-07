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

from pathlib import Path

import rclpy
import yaml
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

    overrides_file: Path
    target_node: str
    full_node_name: str
    ns: str
    apply_tf_frame_prefix_client: Client
    set_parameters_client: Client
    apply_tf_frame_prefix_service_name: str
    set_parameters_service_name: str
    tf_frame_prefix: str
    params_to_set: dict[str, Parameter]
    type_dict: dict[type, Parameter.Type]

    def __init__(self) -> None:
        """Create and initialize the controller parameter bridge node."""
        super().__init__("controller_parameter_bridge")

        self.declare_parameter(
            "target_node_name",
            "controller",
            ParameterDescriptor(read_only=True),
        )
        self.declare_parameter(
            "overrides_param_file",
            "/etc/ros/parameter_overrides.yaml",
            ParameterDescriptor(read_only=True),
        )

        self.declare_parameter(
            "tf_frame_prefix",
            "",
            ParameterDescriptor(read_only=True),
        )

        self.type_dict = {
            str: Parameter.Type.STRING,
            int: Parameter.Type.INTEGER,
            bool: Parameter.Type.BOOL,
            float: Parameter.Type.DOUBLE,
        }

        self.overrides_file = Path(self.get_parameter("overrides_param_file").value)
        self.target_node = self.get_parameter("target_node_name").value.strip("/")
        self.ns = self.get_namespace().strip("/")
        self.full_node_name = (
            f"/{self.ns}/{self.target_node}" if self.ns else f"/{self.target_node}"
        )

        self.apply_tf_frame_prefix_service_name = f"{self.full_node_name}/apply_tf_frame_prefix"
        self.get_logger().info(
            f"Apply TF frame prefix service name: {self.apply_tf_frame_prefix_service_name}",
        )
        self.apply_tf_frame_prefix_client = self.create_client(
            ApplyTfFramePrefix,
            self.apply_tf_frame_prefix_service_name,
        )
        self.tf_frame_prefix = self.get_parameter("tf_frame_prefix").value

        self.set_parameters_service_name = f"{self.full_node_name}/set_parameters"
        self.get_logger().info(f"SetParameters service name: {self.set_parameters_service_name}")
        self.set_parameters_client = self.create_client(
            SetParameters,
            self.set_parameters_service_name,
        )
        self.params_to_set: dict[str, Parameter] = self._parse_overrides_file()

        self.get_logger().info(
            f"Parameter bridge initialized for {self.full_node_name} target node",
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
                f"Service {self.apply_tf_frame_prefix_service_name} not available yet, retrying...",
            )
        self.get_logger().info(
            "apply_tf_frame_prefix service is available. Applying TF frame prefix...",
        )

        request = ApplyTfFramePrefix.Request()
        request.tf_frame_prefix = self.tf_frame_prefix
        future = self._call_with_retry(
            self.apply_tf_frame_prefix_client,
            request,
            self.apply_tf_frame_prefix_service_name,
        )

        if future.result() is None:
            self.get_logger().error("ApplyTfFramePrefix call failed or returned no result.")
            return False

        if not future.result().success:
            self.get_logger().error(f"Failed to apply TF frame prefix: {future.result().message}")
            return False

        self.get_logger().info(
            f"Applied TF frame prefix '{self.tf_frame_prefix}' to {self.full_node_name} node",
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
                f"Service {self.set_parameters_service_name} not available yet, retrying...",
            )
        self.get_logger().info("set_parameters service is available. Applying parameters...")

        request = SetParameters.Request()
        request.parameters = [
            parameter.to_parameter_msg() for parameter in self.params_to_set.values()
        ]
        future = self._call_with_retry(
            self.set_parameters_client,
            request,
            self.set_parameters_service_name,
        )

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
            f"Applied {len(self.params_to_set)} parameter(s) to {self.full_node_name} node",
        )
        return True

    def _call_with_retry(
        self,
        client: Client,
        request: ApplyTfFramePrefix.Request | SetParameters.Request,
        service_name: str,
    ) -> Future:
        """Retry timed out async service calls until the future completes."""
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)

            if future.done():
                return future

            self.get_logger().info(
                f"Service call to {service_name} timed out, retrying...",
            )
            future.cancel()

    def _parse_overrides_file(self) -> dict[str, Parameter]:
        """Parse override YAML file and extract parameters for this bridge."""
        if not self.overrides_file.exists():
            self.get_logger().warning(f"Overrides file not found: {self.overrides_file}")
            return {}

        try:
            file_data = yaml.safe_load(self.overrides_file.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f"Failed to read overrides file {self.overrides_file}: {exc}")
            return {}

        if not isinstance(file_data, dict):
            self.get_logger().warning(
                f"Overrides file must contain a mapping at root: {self.overrides_file}",
            )
            return {}

        controller_entry = self._get_controller_entry(file_data)
        if controller_entry is None:
            self.get_logger().info(
                f"No entry for '{self.target_node}' found in overrides file.",
            )
            return {}

        ros_parameters = controller_entry.get("ros__parameters")
        if not isinstance(ros_parameters, dict):
            self.get_logger().warning(
                "Controller entry exists but has no ros__parameters mapping.",
            )
            return {}

        parsed: dict[str, Parameter] = {}

        def parse_parameters_recursive(
            parameters: dict[str, Parameter],
            param_name_prefix: str,
            values_dict: dict,
        ) -> None:
            for key, value in values_dict.items():
                if not isinstance(key, str):
                    continue

                if isinstance(value, dict):
                    parse_parameters_recursive(
                        parameters,
                        f"{param_name_prefix}{key}.",
                        value,
                    )
                    continue

                parameter = rclpy.Parameter(
                    param_name_prefix + key,
                    self.type_dict[type(value)],
                    value,
                )
                parameters[parameter.name] = parameter

        parse_parameters_recursive(parsed, "", ros_parameters)
        return parsed

    def _get_controller_entry(self, file_data: dict) -> dict | None:
        """Return bridge entry from wildcard block or direct root fallback."""
        wildcard_entry = file_data.get("/**")
        if isinstance(wildcard_entry, dict):
            controller_entry = wildcard_entry.get(self.target_node)
            if isinstance(controller_entry, dict):
                return controller_entry

        return None
