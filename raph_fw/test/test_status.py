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

"""Regression tests for the firmware image and notification snapshot."""

import json
from pathlib import Path

import pytest

from raph_fw import status


@pytest.mark.parametrize(
    ("current", "target", "expected"),
    [
        ("1.0.0", "v1.0.0", "up_to_date"),
        ("v1.9.0", "1.10.0", "update_available"),
        ("1.10.0", "1.9.0", "running_newer"),
        ("1.0.0", "2.0.0", "update_available"),
        ("1.0.0-3-g123abc", "1.1.0", "incomparable"),
        ("1.0.0", "1.1.0-rc.1", "incomparable"),
        ("abc123", "abc123", "incomparable"),
        ("unknown", "1.0.0", "incomparable"),
        ("", "1.0.0", "incomparable"),
        ("0.0.0", "1.0.0", "incomparable"),
        ("1.0.0", "0.0.0", "incomparable"),
        ("01.0.0", "1.0.0", "incomparable"),
    ],
)
def test_compare_versions(current: str, target: str, expected: str) -> None:
    """Only ordered releases can recommend an upgrade."""
    assert status.compare_versions(current, target) == expected


def test_read_target_version(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The target comes from the selected image's marker, not a package version."""
    binary = tmp_path / "firmware.bin"
    binary.write_bytes(b"\x01\x02__FW_VER__:v1.2.3\x00padding")
    monkeypatch.setattr(status, "get_packaged_firmware_path", lambda: binary)
    assert status.read_target_version() == "v1.2.3"


@pytest.mark.parametrize(
    ("data", "message"),
    [
        (b"invalid", "marker not found"),
        (b"__FW_VER__:", "subsection not found"),
        (b"__FW_VER__:\x00", "version is empty"),
    ],
)
def test_invalid_target(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    data: bytes,
    message: str,
) -> None:
    """Malformed images fail explicitly."""
    binary = tmp_path / "firmware.bin"
    binary.write_bytes(data)
    monkeypatch.setattr(status, "get_packaged_firmware_path", lambda: binary)
    with pytest.raises(ValueError, match=message):
        status.read_target_version()


def test_missing_target(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Missing package images must not produce an up-to-date status."""
    monkeypatch.setattr(status, "get_packaged_firmware_path", lambda: tmp_path / "missing")
    with pytest.raises(FileNotFoundError):
        status.read_target_version()


def test_image_size_limit(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The exact firmware flash-size limit is enforced."""
    binary = tmp_path / "firmware.bin"
    data = b"__FW_VER__:1.2.3\x00"
    binary.write_bytes(data.ljust(800 * 1024, b"\x00"))
    monkeypatch.setattr(status, "get_packaged_firmware_path", lambda: binary)
    assert status.read_target_version() == "1.2.3"
    binary.write_bytes(data.ljust(800 * 1024 + 1, b"\x00"))
    with pytest.raises(ValueError, match="800 KiB"):
        status.read_target_version()


def test_atomic_cache(tmp_path: Path) -> None:
    """Snapshots retain their API shape and are readable by the login user."""
    path = tmp_path / "status.json"
    snapshot = status.FirmwareStatus(
        status="update_available",
        current_version="1.0.0",
        target_version="1.1.0",
        checked_at=100,
    )
    status.write_status(path, snapshot)
    data = json.loads(path.read_text())
    assert data["status"] == "update_available"
    assert data["checked_at"] == 100
    assert data["checking"] is False
    assert data["update_command"] == "ros2 run raph_fw update"
    assert "recovery mode" in data["update_instructions"]
    assert path.stat().st_mode & 0o777 == 0o644
    assert not path.with_suffix(".tmp").exists()
    snapshot.status = "up_to_date"
    status.write_status(path, snapshot)
    assert json.loads(path.read_text())["status"] == "up_to_date"


def test_cache_failure(tmp_path: Path) -> None:
    """Unprovisioned cache directories report errors rather than hiding them."""
    with pytest.raises(FileNotFoundError):
        status.write_status(tmp_path / "missing" / "status.json", status.FirmwareStatus())
