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

from raph_fw.console import get_logger


class FlashCommand:
    """Command to flash firmware onto the RaphCore device."""

    def __init__(self) -> None:
        """Initialize the FlashCommand."""
        self.logger = get_logger("FlashCommand")

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """Add command-line arguments for the flash command."""
        parser.add_argument(
            "binary_path",
            type=str,
            help="Path to the binary file to be flashed.",
        )
        parser.add_argument(
            "--bootloader",
            action="store_true",
            help="Indicates that the binary is a bootloader image.",
            default=False,
        )

    def main(self, args: argparse.Namespace) -> None:
        """Execute the flashing process."""
        binary_path = args.binary_path
        is_bootloader = args.bootloader

        self.logger.info("Flashing binary: %s", binary_path)
        if is_bootloader:
            self.logger.info("Flashing bootloader image.")
