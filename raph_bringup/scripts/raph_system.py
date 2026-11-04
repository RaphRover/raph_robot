#!/usr/bin/env python3

"""A module containing the RaphSystem node implementation."""

import subprocess
import time
from pathlib import Path

import rclpy
from raph_fw.status import FirmwareStatus, compare_versions, read_target_version, write_status
from raph_interfaces.srv import GetControllerInfo, GetFirmwareStatus, GetOsVersion
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

FIRMWARE_CHECK_INTERVAL = 15 * 60
FIRMWARE_CHECK_TIMEOUT = 5.0


class RaphSystem(Node):
    """
    Raph Rover system node.

    Node responsible for system-level operations like rebooting, shutting down, and providing OS
    version information.
    """

    shutdown_scheduled: bool
    os_version: GetOsVersion.Response

    def __init__(self) -> None:
        """Create a RaphSystem node."""
        super().__init__("raph_system", start_parameter_services=False)

        self.shutdown_scheduled = False
        self.os_version = GetOsVersion.Response(version="<not_found>")

        self._read_os_version()

        self.reboot_srv = self.create_service(Trigger, "~/reboot", self._reboot_callback)
        self.shutdown_srv = self.create_service(Trigger, "~/shutdown", self._shutdown_callback)
        self.get_os_version_srv = self.create_service(
            GetOsVersion,
            "~/get_os_version",
            self._get_os_version_callback,
        )
        self.firmware_status = FirmwareStatus()
        self.firmware_status_path = Path(
            self.declare_parameter(
                "firmware_status_path",
                "/var/lib/raph/firmware-status/status.json",
            )
            .get_parameter_value()
            .string_value,
        )
        self._firmware_future: Future | None = None
        self._firmware_deadline = 0.0
        self._next_firmware_check = 0.0
        self.controller_info_client = self.create_client(
            GetControllerInfo,
            "controller/get_controller_info",
        )
        self.get_firmware_status_srv = self.create_service(
            GetFirmwareStatus,
            "~/get_firmware_status",
            self._get_firmware_status_callback,
        )
        self.firmware_check_timer = self.create_timer(
            1.0,
            self._periodic_firmware_check,
        )
        self._start_firmware_check()
        self.get_logger().info("Raph system node started!")

    def _periodic_firmware_check(self) -> None:
        if self.firmware_status.checking:
            self._check_firmware_timeout()
        elif time.monotonic() >= self._next_firmware_check:
            self._start_firmware_check()

    def _get_firmware_status_callback(
        self,
        request: GetFirmwareStatus.Request,
        response: GetFirmwareStatus.Response,
    ) -> GetFirmwareStatus.Response:
        if request.refresh:
            self._start_firmware_check()
        status = self.firmware_status
        response.status = status.status
        response.current_version = status.current_version
        response.target_version = status.target_version
        response.checking = status.checking
        response.checked_at = status.checked_at
        response.status_message = status.status_message
        response.update_command = status.update_command
        response.update_instructions = status.update_instructions
        return response

    def _start_firmware_check(self) -> None:
        if self.firmware_status.checking:
            return
        self.firmware_status = FirmwareStatus(checking=True)
        self._firmware_deadline = time.monotonic() + FIRMWARE_CHECK_TIMEOUT
        try:
            self.firmware_status.target_version = read_target_version()
        except (OSError, ValueError, LookupError) as exc:
            self._finish_firmware_check(f"Cannot read packaged firmware: {exc}")
            return
        self._request_controller_info()

    def _request_controller_info(self) -> None:
        if self._firmware_future is not None or not self.controller_info_client.service_is_ready():
            return
        try:
            self._firmware_future = self.controller_info_client.call_async(
                GetControllerInfo.Request(),
            )
        except RuntimeError as exc:
            self._finish_firmware_check(f"Cannot request controller information: {exc}")
            return
        self._firmware_future.add_done_callback(self._controller_info_done)

    def _controller_info_done(self, future: Future) -> None:
        if future is not self._firmware_future:
            return
        self._firmware_future = None
        if time.monotonic() >= self._firmware_deadline:
            self._finish_firmware_check("Timed out waiting for controller firmware information.")
            return
        error = future.exception()
        if error is not None:
            self._finish_firmware_check(f"Cannot read running firmware: {error}")
            return
        response = future.result()
        if response is None or not response.firmware_version:
            self._finish_firmware_check("Controller returned an empty firmware version.")
            return
        self.firmware_status.current_version = response.firmware_version
        self.firmware_status.status = compare_versions(
            response.firmware_version,
            self.firmware_status.target_version,
        )
        self._finish_firmware_check()

    def _check_firmware_timeout(self) -> None:
        if not self.firmware_status.checking:
            return
        if time.monotonic() >= self._firmware_deadline:
            if self._firmware_future is not None:
                future = self._firmware_future
                self._firmware_future = None
                self.controller_info_client.remove_pending_request(future)
                future.cancel()
            self._finish_firmware_check("Timed out waiting for controller firmware information.")
            return
        self._request_controller_info()

    def _finish_firmware_check(self, error: str = "") -> None:
        self.firmware_status.checking = False
        self.firmware_status.checked_at = int(time.time())
        self.firmware_status.status_message = error
        self._next_firmware_check = time.monotonic() + (30.0 if error else FIRMWARE_CHECK_INTERVAL)
        if error:
            self.firmware_status.status = "unavailable"
            self.get_logger().warning(error)
        try:
            write_status(self.firmware_status_path, self.firmware_status)
        except OSError as exc:
            self.get_logger().error(f"Cannot write firmware notification cache: {exc}")

    def _reboot_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.shutdown_scheduled:
            response.success = False
            response.message = "System is already scheduled to shutdown"
            return response

        self.shutdown_scheduled = True

        log_msg = "System scheduled to reboot in 3 seconds"
        self.get_logger().info(log_msg)
        response.success = True
        response.message = log_msg

        self.create_timer(3.0, self.reboot_system)

        return response

    def _shutdown_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.shutdown_scheduled:
            response.success = False
            response.message = "System is already scheduled to shutdown"
            return response

        self.shutdown_scheduled = True

        log_msg = "System scheduled to shut down in 3 seconds"
        self.get_logger().info(log_msg)
        response.success = True
        response.message = log_msg

        self.create_timer(3.0, self.shutdown_system)

        return response

    def _get_os_version_callback(
        self,
        _request: GetOsVersion.Request,
        response: GetOsVersion.Response,
    ) -> GetOsVersion.Response:
        response.version = self.os_version.version
        response.variant = self.os_version.variant
        response.major = self.os_version.major
        response.minor = self.os_version.minor
        response.patch = self.os_version.patch
        return response

    def reboot_system(self) -> None:
        """Reboot the system."""
        self.get_logger().info("Performing system reboot...")
        subprocess.run(["/usr/bin/systemctl", "reboot"], check=False)
        self.get_logger().info("Shutting down node...")
        rclpy.shutdown()

    def shutdown_system(self) -> None:
        """Shut down the system."""
        self.get_logger().info("Performing system shutdown...")
        subprocess.run(["/usr/bin/systemctl", "poweroff"], check=False)
        self.get_logger().info("Shutting down node...")
        rclpy.shutdown()

    def _read_os_version(self) -> None:
        """
        Fill os_version structure with the current OS version.

        Reads the /etc/custom-os-release file to extract OS_VERSION and OS_VARIANT and parses them.
        """
        try:
            release_path = Path("/etc/custom-os-release")
            if not release_path.exists():
                self.get_logger().warning("/etc/custom-os-release not found")
                return

            content = release_path.read_text(encoding="utf-8")
            version_found = False

            for line in content.splitlines():
                if line.startswith(("OS_VERSION=", "OS_VARIANT=")):
                    key, raw_value = line.split("=", 1)
                    value = raw_value.strip().strip('"')

                    if key == "OS_VERSION":
                        self.os_version.version = value
                        self._parse_os_version()
                        version_found = True
                    elif key == "OS_VARIANT":
                        self.os_version.variant = value

                    self.get_logger().info(f"Detected {key}: {value}")

            if not version_found:
                self.get_logger().warning("OS_VERSION not found in /etc/custom-os-release")
        except (OSError, UnicodeDecodeError) as exc:
            self.get_logger().error(f"Failed to read /etc/custom-os-release: {exc}")

    def _parse_os_version(self) -> None:
        """Parse the OS version string and extract the major, minor, and patch versions."""
        try:
            version_parts = self.os_version.version.split(".")
            if len(version_parts) < 3:
                self.get_logger().warning(
                    "OS_VERSION does not contain enough parts to parse major, minor, and patch",
                )
                return

            self.os_version.major = int(version_parts[0])
            self.os_version.minor = int(version_parts[1])
            self.os_version.patch = int(version_parts[2])

            self.get_logger().info(
                "Parsed OS_VERSION: "
                f"{self.os_version.major}.{self.os_version.minor}.{self.os_version.patch}",
            )
        except (AttributeError, ValueError) as exc:
            self.get_logger().error(f"Failed to parse OS_VERSION: {exc}")


def _main() -> None:
    rclpy.init()
    raph_system = RaphSystem()

    try:
        rclpy.spin(raph_system)
    except KeyboardInterrupt:
        raph_system.get_logger().info("Got Ctrl+C, shutting down.")
        raph_system.destroy_node()


if __name__ == "__main__":
    _main()
