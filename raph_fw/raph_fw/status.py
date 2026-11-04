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

"""Shared, read-only firmware status checks."""

import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from raph_fw.versions import get_firmware_version, get_packaged_firmware_path

FirmwareState = Literal[
    "up_to_date",
    "update_available",
    "running_newer",
    "incomparable",
    "unavailable",
]
UPDATE_COMMAND = "ros2 run raph_fw update"
UPDATE_INSTRUCTIONS = (
    "Stop driving and put RaphCore into recovery mode before updating. "
    "On the robot, run 'ros2 run raph_fw update' and confirm the flash. "
    "Power-cycle the device after flashing to apply the update. "
    "A newer image being available does not mean the device is ready to flash."
)
_RELEASE_VERSION = re.compile(r"v?(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)")


@dataclass
class FirmwareStatus:
    """Status shared by the ROS API and the SSH notification cache."""

    status: FirmwareState = "unavailable"
    current_version: str = ""
    target_version: str = ""
    checking: bool = False
    checked_at: int = 0
    status_message: str = "Firmware has not been checked yet."
    update_command: str = UPDATE_COMMAND
    update_instructions: str = UPDATE_INSTRUCTIONS


def compare_versions(current: str, target: str) -> FirmwareState:
    """Compare release tags without guessing ordering for git-describe revisions."""
    current_match = _RELEASE_VERSION.fullmatch(current)
    target_match = _RELEASE_VERSION.fullmatch(target)
    if current_match is None or target_match is None:
        return "incomparable"

    current_parts = tuple(int(part) for part in current_match.groups())
    target_parts = tuple(int(part) for part in target_match.groups())
    # 0.0.0 is also the firmware build's fallback when git information is unavailable.
    if current_parts == (0, 0, 0) or target_parts == (0, 0, 0):
        return "incomparable"
    if current_parts == target_parts:
        return "up_to_date"
    return "update_available" if current_parts < target_parts else "running_newer"


def read_target_version() -> str:
    """Read and validate the image selected by the existing update command."""
    path = get_packaged_firmware_path()
    if path.stat().st_size > 800 * 1024:
        msg = f"Packaged firmware exceeds the 800 KiB flash limit: {path}"
        raise ValueError(msg)
    version = get_firmware_version(path)
    if not version:
        msg = f"Packaged firmware version is empty: {path}"
        raise ValueError(msg)
    return version


def write_status(path: Path, status: FirmwareStatus) -> None:
    """Atomically replace the cache so login readers never see a partial write."""
    temporary = path.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(asdict(status)) + "\n", encoding="utf-8")
        temporary.chmod(0o644)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
