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

import argparse
import sys
import time
from pathlib import Path
from typing import Protocol

from raph_fw.console import (
    get_choice_prompt,
    get_confirmation_prompt,
    get_logger,
    get_progress,
    log_step,
)
from raph_fw.controller import SERVICE_TIMEOUT, ControllerClient
from raph_fw.resolve import resolve_raphcore_name
from raph_fw.tftp import ServerRole, TFTPError, discover_server, write_binary
from raph_fw.versions import get_bootloader_version, get_firmware_version

# Maximum time for the device to reboot, apply the staged image and start the firmware again
RESTART_TIMEOUT = 60.0
RESTART_POLL_TIMEOUT = 1.0
RESTART_POLL_INTERVAL = 0.2
# Maximum time for the controller services to respond after the firmware restarts
RESTART_SERVICE_TIMEOUT = 30.0


class FlashCommand:
    """Command to flash firmware onto the RaphCore device."""

    class CommonArgs(Protocol):
        """Protocol for the common arguments expected by FlashCommand and UpdateCommand."""

        bootloader: bool
        address: str | None

    class FlashCommandArgs(CommonArgs, Protocol):
        """Protocol for the arguments expected by FlashCommand."""

        binary_path: str

    def __init__(self) -> None:
        """Initialize the FlashCommand."""
        self.logger = get_logger("FlashCommand")

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """
        Add command-line arguments for the flash command.

        :param parser: The argument parser to add arguments to.
        """
        parser.add_argument(
            "binary_path",
            type=str,
            help="Path to the binary file to be flashed.",
        )
        self.add_common_arguments(parser)

    def add_common_arguments(self, parser: argparse.ArgumentParser) -> None:
        """Add common command-line arguments for both flash and update commands."""
        parser.add_argument(
            "--bootloader",
            action="store_true",
            help="Indicates that the binary is a bootloader image.",
            default=False,
        )
        parser.add_argument(
            "--address",
            action="store",
            type=str,
            help=(
                "The IP address of the RaphCore device to flash. "
                "If not provided, the command will attempt to auto-detect the device."
            ),
        )

    def main(self, args: FlashCommandArgs) -> None:
        """
        Execute the flashing process.

        :param args: Parsed command-line arguments.
        """
        self.args = args

        self.logger.info(
            f"Will attempt to flash {'bootloader' if args.bootloader else 'firmware'} binary at: "
            f"{args.binary_path}",
        )

        self._validate_binary_path()
        self._check_binary_version_and_size()
        self._resolve_device_address()
        self._discover_flashing_server()
        if self.server_role == ServerRole.FIRMWARE:
            self._show_current_versions()
        self._perform_flash()
        if self.server_role == ServerRole.FIRMWARE:
            self._apply_staged_update()

    @property
    def _image_kind(self) -> str:
        """Human-readable name of the image type being flashed."""
        return "bootloader" if self.args.bootloader else "firmware"

    def _validate_binary_path(self) -> None:
        """Validate that the binary path exists and is a file."""
        self.binary_path = Path(self.args.binary_path)
        if not self.binary_path.is_file():
            self.logger.error(f"Binary file not found: {self.binary_path.absolute()}")
            sys.exit(1)

    def _check_binary_version_and_size(self) -> None:
        """Check and log the version of the binary being flashed."""
        try:
            if self.args.bootloader:
                with log_step("Checking version of the bootloader binary"):
                    self.version = get_bootloader_version(self.binary_path)
            else:
                with log_step("Checking version of the firmware binary"):
                    self.version = get_firmware_version(self.binary_path)
        except ValueError:
            self.logger.exception(
                "Failed to read version from the binary. Please ensure it's a valid RaphCore "
                f"{'bootloader' if self.args.bootloader else 'firmware'} binary.",
            )
            sys.exit(1)
        else:
            self.logger.info(
                f"{'Bootloader' if self.args.bootloader else 'Firmware'} version to flash: "
                f"{self.version}",
            )

        max_size = 224 * 1024 if self.args.bootloader else 800 * 1024
        actual_size = self.binary_path.stat().st_size
        if actual_size > max_size:
            self.logger.error(
                f"Binary size {actual_size} bytes exceeds the maximum allowed size of "
                f"{max_size} bytes for a {'bootloader' if self.args.bootloader else 'firmware'} "
                "image.",
            )
            sys.exit(1)

    def _resolve_device_address(self) -> None:
        """Resolve the target device's IP address."""
        if self.args.address is None:
            try:
                with log_step(
                    "No address provided, attempting to resolve RaphCore device on the network",
                ):
                    addresses = resolve_raphcore_name()
            except TimeoutError:
                self.logger.exception(
                    "Failed to resolve address of a RaphCore device on the network. "
                    "Please ensure the device is powered on and connected to the same network.",
                )
                sys.exit(1)

            if len(addresses) > 1:
                self.address = get_choice_prompt(
                    "Multiple RaphCore devices found. Please select the target device:",
                    choices=addresses,
                )
            else:
                self.address = addresses[0]
                self.logger.info(f"Resolved RaphCore device at {self.address}")
        else:
            self.address = self.args.address
            self.logger.info(f"Using provided address: {self.address}")

    def _discover_flashing_server(self) -> None:
        """Discover the flashing server on the target device and determine its role."""
        self.server_role: ServerRole | None = None
        try:
            with log_step("Discovering flashing server"):
                self.server_role = discover_server(self.address)
        except TimeoutError:
            self.logger.warning(
                "Failed to discover flashing server on the device. "
                "The device is either unreachable, running a firmware without a flashing server "
                "(in which case it needs to be put into recovery mode) or running a bootloader "
                "too old to support flashing server discovery.",
            )
            if not get_confirmation_prompt(
                "Do you want to proceed with flashing anyway? "
                "This may fail if the server is indeed unreachable.",
                default=False,
            ):
                self.logger.info("Aborting flash command.")
                sys.exit(0)
        else:
            if self.server_role == ServerRole.BOOTLOADER:
                self.logger.info(
                    "Flashing server is running on the bootloader (recovery mode). "
                    "The image will be written directly to the target memory region.",
                )
            else:
                self.logger.info(
                    "Flashing server is running on the firmware. "
                    "The image will be stored in the backup flash bank and applied by the "
                    "bootloader on the next boot.",
                )

    def _show_current_versions(self) -> None:
        """Retrieve and log the bootloader and firmware versions currently running on the device."""
        try:
            current_version = self._retrieve_current_version(
                "Retrieving current versions from the controller",
            )
        except TimeoutError:
            self.logger.warning(
                "Failed to retrieve current versions from the controller.",
                exc_info=True,
            )
            return

        if current_version == self.version:
            self.logger.info(
                f"The device is already running {self._image_kind} version {self.version}.",
            )

    def _retrieve_current_version(
        self,
        label: str,
        timeout: float = SERVICE_TIMEOUT,
    ) -> str:
        """
        Retrieve and log the versions running on the device through the controller ROS service.

        :param label: The description text shown while the step is in progress.
        :param timeout: Maximum time in seconds to wait for a response from the controller.
        :returns: The running version of the image type being flashed.
        :raises TimeoutError: If the service is not available or does not respond in time.
        """
        with log_step(label), ControllerClient() as controller:
            info = controller.get_controller_info(timeout=timeout)

        self.logger.info(f"Current bootloader version: {info.bootloader_version}")
        self.logger.info(f"Current firmware version: {info.firmware_version}")
        return str(info.bootloader_version if self.args.bootloader else info.firmware_version)

    def _perform_flash(self) -> None:
        """Perform the actual flashing process."""
        if not get_confirmation_prompt(
            f"Ready to flash {'bootloader' if self.args.bootloader else 'firmware'} version "
            f"{self.version} to device at {self.address}. Do you want to proceed?",
            default=True,
        ):
            self.logger.info("Aborting flash command.")
            sys.exit(0)

        self.logger.info("Starting flashing process...")
        try:
            with get_progress() as progress:
                task = progress.add_task("Flashing", total=None)

                def _on_progress(bytes_sent: int, total_bytes: int) -> None:
                    progress.update(task, total=total_bytes, completed=bytes_sent, refresh=True)

                write_binary(
                    self.address,
                    self.binary_path,
                    is_bootloader=self.args.bootloader,
                    progress_callback=_on_progress,
                )
        except TFTPError:
            self.logger.critical("Flashing failed with a TFTP error.", exc_info=True)
            sys.exit(1)
        except TimeoutError:
            self.logger.critical(
                "Flashing failed due to a timeout. "
                "The device may have become unresponsive during the flashing process.",
                exc_info=True,
            )
            sys.exit(1)
        except OSError:
            self.logger.critical(
                "Flashing failed due to an OS error. "
                "This may indicate a network issue or a problem with the TFTP client.",
                exc_info=True,
            )
            sys.exit(1)

        if self.server_role == ServerRole.FIRMWARE:
            self.logger.info("Image uploaded successfully.")
        else:
            self.logger.info(
                "Flashing completed successfully. "
                "Please power cycle the device to apply the update.",
            )

    def _apply_staged_update(self) -> None:
        """Reboot the device so that the bootloader applies the image staged by the firmware."""
        if not get_confirmation_prompt(
            "The update will be applied by the bootloader on the next boot. "
            "Do you want to reboot the controller now?",
            default=True,
        ):
            self.logger.info("The update will be applied on the next power cycle of the device.")
            return

        try:
            with log_step("Requesting controller reboot"), ControllerClient() as controller:
                controller.soft_reboot()
        except (TimeoutError, RuntimeError):
            self.logger.exception(
                "Failed to request controller reboot. "
                "Please power cycle the device to apply the update.",
            )
            sys.exit(1)

        try:
            with log_step("Waiting for the device to apply the update and restart"):
                role = self._wait_for_device_restart()
        except TimeoutError:
            self.logger.critical(
                f"The device did not come back online within {RESTART_TIMEOUT:.0f} seconds. "
                "Please check the device state and power cycle it if necessary.",
            )
            sys.exit(1)

        if role == ServerRole.BOOTLOADER:
            self.logger.critical(
                "The device stayed in recovery mode after the reboot, which indicates that the "
                "bootloader failed to apply the update. Please try flashing the image again.",
            )
            sys.exit(1)

        self._verify_updated_version()

    def _verify_updated_version(self) -> None:
        """Verify that the device runs the flashed version after the update has been applied."""
        try:
            current_version = self._retrieve_current_version(
                "Retrieving versions from the controller",
                timeout=RESTART_SERVICE_TIMEOUT,
            )
        except TimeoutError:
            self.logger.warning(
                "The device restarted, but its versions could not be retrieved to confirm the "
                "update.",
                exc_info=True,
            )
            return

        if current_version != self.version:
            self.logger.critical(
                f"Version mismatch after the update: expected {self.version}, "
                f"device reports {current_version}.",
            )
            sys.exit(1)

        self.logger.info("Update applied successfully.")

    def _wait_for_device_restart(self) -> ServerRole:
        """
        Wait for the flashing server to go offline and come back after a reboot.

        :return: The role of the flashing server once the device is back online.
        :raises TimeoutError: If the device does not restart within the timeout period.
        """
        deadline = time.monotonic() + RESTART_TIMEOUT
        went_offline = False
        while time.monotonic() < deadline:
            try:
                role = discover_server(self.address, timeout=RESTART_POLL_TIMEOUT)
            except OSError:
                # TimeoutError is a subclass of OSError; the server is not responding
                went_offline = True
                continue
            if went_offline:
                return role
            # The firmware is still running; the reboot has not happened yet
            time.sleep(RESTART_POLL_INTERVAL)
        raise TimeoutError
