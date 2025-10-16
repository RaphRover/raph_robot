#!/usr/bin/env python3

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from raph_interfaces.srv import GetOsVersion


class RaphSystem(Node):
    shutdown_scheduled: bool
    os_version: GetOsVersion.Response

    def __init__(self):
        super().__init__("raph_system", start_parameter_services=False)

        self.shutdown_scheduled = False
        self.os_version = GetOsVersion.Response(version="<not_found>")

        self._read_os_version()

        self.reboot_srv = self.create_service(Trigger, "~/reboot", self.reboot_callback)
        self.shutdown_srv = self.create_service(
            Trigger, "~/shutdown", self.shutdown_callback
        )
        self.get_os_version_srv = self.create_service(
            GetOsVersion, "~/get_os_version", self.get_os_version_callback
        )
        self.get_logger().info("Raph system node started!")

    def reboot_callback(self, request, response):
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

    def shutdown_callback(self, request, response):
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

    def get_os_version_callback(self, request, response):
        return self.os_version

    def reboot_system(self):
        self.get_logger().info("Performing system reboot...")
        os.system("systemctl reboot")
        self.get_logger().info("Shutting down node...")
        rclpy.shutdown()

    def shutdown_system(self):
        self.get_logger().info("Performing system shutdown...")
        os.system("systemctl poweroff")
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
                if line.startswith("OS_VERSION=") or line.startswith("OS_VARIANT="):
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
                self.get_logger().warning(
                    "OS_VERSION not found in /etc/custom-os-release"
                )
        except (OSError, UnicodeDecodeError) as exc:
            self.get_logger().error(f"Failed to read /etc/custom-os-release: {exc}")

    def _parse_os_version(self) -> None:
        """Parse the OS version string and extract the major, minor, and patch versions."""
        try:
            version_parts = self.os_version.version.split(".")
            if len(version_parts) < 3:
                self.get_logger().warning(
                    "OS_VERSION does not contain enough parts to parse major, minor, and patch"
                )
                return

            self.os_version.major = int(version_parts[0])
            self.os_version.minor = int(version_parts[1])
            self.os_version.patch = int(version_parts[2])

            self.get_logger().info(
                f"Parsed OS_VERSION: {self.os_version.major}.{self.os_version.minor}.{self.os_version.patch}"
            )
        except (AttributeError, ValueError) as exc:
            self.get_logger().error(f"Failed to parse OS_VERSION: {exc}")


def main() -> None:
    rclpy.init()
    raph_system = RaphSystem()

    try:
        rclpy.spin(raph_system)
    except KeyboardInterrupt:
        raph_system.get_logger().info("Got Ctrl+C, shutting down.")
        raph_system.destroy_node()


if __name__ == "__main__":
    main()
