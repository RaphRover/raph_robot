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

"""Utility functions for retrieving version information."""

import struct
import zlib
from pathlib import Path

_BOOTLOADER_INFO_OFFSET = 0x400
_BOOTLOADER_INFO_HEADER_SIZE = 68
_BOOTLOADER_MAGIC = 0x424F4F54  # 'BOOT'
_MAX_VERSION_STRING_SIZE = 51  # 50 chars + null terminator

_HEADER_FMT = "<IBH I 51s 2s I"


def get_bootloader_version(path: Path) -> str:
    """
    Get the bootloader version from the specified file.

    Reads the BootloaderInfoHeader at offset 0x400, validates the magic
    value and CRC32, then returns the human-readable version string.

    Raises
    ------
    ValueError
        If the file is too small, the magic is invalid,
        or the CRC32 check fails.

    """
    data = path.read_bytes()

    header_end = _BOOTLOADER_INFO_OFFSET + _BOOTLOADER_INFO_HEADER_SIZE
    if len(data) < header_end:
        msg = f"File too small: expected at least {header_end} bytes, got {len(data)}"
        raise ValueError(msg)

    header_bytes = data[_BOOTLOADER_INFO_OFFSET:header_end]

    (
        magic,
        _header_version,
        _header_size,
        _version_id,
        version_string_raw,
        _padding,
        crc32_stored,
    ) = struct.unpack(_HEADER_FMT, header_bytes)

    if magic != _BOOTLOADER_MAGIC:
        msg = (
            f"Invalid bootloader info magic: expected {_BOOTLOADER_MAGIC:#010x}, got {magic:#010x}"
        )
        raise ValueError(msg)

    crc32_computed = zlib.crc32(header_bytes[:-4]) & 0xFFFFFFFF
    if crc32_computed != crc32_stored:
        msg = (
            f"Bootloader info header CRC mismatch: "
            f"expected {crc32_stored:#010x}, computed {crc32_computed:#010x}"
        )
        raise ValueError(msg)

    return version_string_raw.split(b"\x00", 1)[0].decode("ascii")


_FW_VERSION_PREFIX = b"__FW_VER__:"


def get_firmware_version(path: Path) -> str:
    """
    Get the firmware version from the specified file.

    Scans the binary for the ``__FW_VER__:`` marker and returns the
    null-terminated version string that follows it.

    Raises
    ------
    ValueError
        If the version marker is not found in the file.

    """
    data = path.read_bytes()
    idx = data.find(_FW_VERSION_PREFIX)
    if idx == -1:
        msg = f"Firmware version marker not found in {path}"
        raise ValueError(msg)

    start = idx + len(_FW_VERSION_PREFIX)
    end = data.index(b"\x00", start)
    return data[start:end].decode("ascii")


if __name__ == "__main__":
    from ament_index_python.packages import get_package_share_directory

    from raph_fw.console import get_logger, log_step

    logger = get_logger()

    share_dir = Path(get_package_share_directory("raph_fw"))
    bootloader_bin = share_dir / "data" / "bootloader" / "raphcore_bootloader_latest.bin"
    firmware_bin = share_dir / "data" / "firmware" / "raphcore_firmware_latest.bin"

    try:
        with log_step("Checking bootloader version"):
            version = get_bootloader_version(bootloader_bin)
    except ValueError:
        logger.exception("Failed to read bootloader version")
    else:
        logger.info(f"Bootloader version: {version}")

    # Test failure case by trying to read bootloader version from the firmware binary
    try:
        with log_step("Checking bootloader version"):
            version = get_bootloader_version(firmware_bin)
    except ValueError:
        logger.exception("Failed to read bootloader version")
    else:
        logger.info(f"Bootloader version: {version}")

    try:
        with log_step("Checking firmware version"):
            version = get_firmware_version(firmware_bin)
    except ValueError:
        logger.exception("Failed to read firmware version")
    else:
        logger.info(f"Firmware version: {version}")

    # Test failure case by trying to read firmware version from the bootloader binary
    try:
        with log_step("Checking firmware version"):
            version = get_firmware_version(bootloader_bin)
    except ValueError:
        logger.exception("Failed to read firmware version")
    else:
        logger.info(f"Firmware version: {version}")
