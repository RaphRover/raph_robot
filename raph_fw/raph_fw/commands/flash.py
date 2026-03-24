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
from pathlib import Path
from typing import Protocol

from raph_fw.console import get_choice_prompt, get_logger, log_step
from raph_fw.resolve import resolve_raphcore_name


class FlashCommand:
    """Command to flash firmware onto the RaphCore device."""

    class CommonArgs(Protocol):
        """Protocol for the common arguments expected by FlashCommand and UpdateCommand."""

        bootloader: bool
        address: str | None

    class FlashCommandArgs(CommonArgs):
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
            f"{args.binary_path}"
        )

        # Validate the binary path
        binary_path = Path(args.binary_path)
        if not binary_path.is_file():
            self.logger.error(f"Binary file not found: {binary_path.absolute()}")
            sys.exit(1)

        # Resolve the target device's IP address
        if self.args.address is None:
            try:
                with log_step(
                    "No address provided, attempting to resolve RaphCore device on the network",
                ):
                    addresses = resolve_raphcore_name()
            except TimeoutError:
                self.logger.exception(
                    "Failed to resolve RaphCore device on the network. "
                    "Please ensure the device is powered on and connected to the same network.",
                )
                sys.exit(1)

            if len(addresses) > 1:
                address = get_choice_prompt(
                    "Multiple RaphCore devices found. Please select the target device:",
                    choices=addresses,
                )
            else:
                address = addresses[0]
                self.logger.info(f"Resolved RaphCore device at {address}")
        else:
            address = self.args.address
            self.logger.info(f"Using provided address: {address}")
