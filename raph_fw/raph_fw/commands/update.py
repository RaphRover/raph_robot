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
from pathlib import Path
from typing import cast

from ament_index_python import get_package_share_directory

from raph_fw.commands import FlashCommand


class UpdateCommand(FlashCommand):
    """Command to update the RaphCore device firmware."""

    def __init__(self) -> None:
        """Initialize the UpdateCommand."""
        super().__init__()

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """
        Add command-line arguments for the update command.

        :param parser: The argument parser to add arguments to.
        """
        self.add_common_arguments(parser)

    def main(self, args: FlashCommand.CommonArgs) -> None:
        """
        Execute the update command with the provided arguments.

        :param args: The parsed command-line arguments.
        """
        share_dir = Path(get_package_share_directory("raph_fw"))
        bootloader_bin = share_dir / "data" / "bootloader" / "raphcore_bootloader_latest.bin"
        firmware_bin = share_dir / "data" / "firmware" / "raphcore_firmware_latest.bin"

        if args.bootloader:
            binary_path = str(bootloader_bin.resolve())
        else:
            binary_path = str(firmware_bin.resolve())

        args.binary_path = binary_path  # type: ignore[attr-defined]

        # The update process is the same as flashing, but with a different binary path
        super().main(cast("FlashCommand.FlashCommandArgs", args))
