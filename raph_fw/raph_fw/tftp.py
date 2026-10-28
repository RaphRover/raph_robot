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

from __future__ import annotations

import socket
import struct
import time
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path

BOOTLOADER_FILE = "bootloader"
FIRMWARE_FILE = "firmware"


class OpCode(Enum):
    """TFTP operation codes."""

    READ_REQUEST = 1
    WRITE_REQUEST = 2
    DATA = 3
    ACK = 4
    ERROR = 5

    # Custom opcodes for discovering the TFTP server; not part of the standard TFTP protocol.
    # Older bootloaders respond to DISCOVER_REQUEST by echoing the same opcode back, while newer
    # bootloaders and firmware respond with DISCOVER_RESPONSE followed by the server role.
    DISCOVER_REQUEST = 0x8000
    DISCOVER_RESPONSE = 0x8001


class ErrorCode(Enum):
    """TFTP error codes."""

    NOT_DEFINED = 0
    FILE_NOT_FOUND = 1
    DISK_FULL = 3
    ILLEGAL_OPERATION = 4


class ServerRole(Enum):
    """Role of the device software that runs the TFTP server."""

    # The server runs in the bootloader (recovery mode). Written images overwrite the target
    # memory region as the transfer progresses.
    BOOTLOADER = 0
    # The server runs in the firmware. Written images are stored in the backup flash bank and
    # applied by the bootloader on the next boot.
    FIRMWARE = 1


class TFTPError(Exception):
    """Custom exception for TFTP errors."""

    def __init__(self, error_code: ErrorCode, message: str) -> None:
        """Initialize the TFTPError."""
        super().__init__(f"TFTP error {error_code.value}: {message}")
        self.error_code = error_code
        self.message = message


def _build_discover_request() -> bytes:
    """Build a TFTP discover request packet."""
    return struct.pack("!H", OpCode.DISCOVER_REQUEST.value)


def _build_read_request(filename: str) -> bytes:
    """Build a TFTP read request packet."""
    return struct.pack("!H", OpCode.READ_REQUEST.value) + filename.encode("ascii") + b"\0octet\0"


def _build_write_request(filename: str) -> bytes:
    """Build a TFTP write request packet."""
    return struct.pack("!H", OpCode.WRITE_REQUEST.value) + filename.encode("ascii") + b"\0octet\0"


def _build_data_packet(block_number: int, data: bytes) -> bytes:
    """Build a TFTP data packet."""
    return struct.pack("!HH", OpCode.DATA.value, block_number) + data


def _send_packet_and_wait_for_ack(
    s: socket.socket,
    packet: bytes,
    address: str,
    *,
    retries: int,
    port: int,
) -> int:
    """
    Send a TFTP packet and wait for an acknowledgment.

    :return: The block number of the acknowledgment received.
    :raises TimeoutError: If no acknowledgment is received within the timeout period.
    :raises TFTPError: If a TFTP error is received.
    """
    for attempt in range(retries):
        s.sendto(packet, (address, port))
        try:
            data, _ = s.recvfrom(512)
            opcode = struct.unpack("!H", data[:2])[0]
            if opcode == OpCode.ACK.value:
                return struct.unpack("!H", data[2:4])[0]
            if opcode == OpCode.ERROR.value:
                error_code = struct.unpack("!H", data[2:4])[0]
                error_message = data[4:-1].decode("ascii")
                try:
                    tftp_error = ErrorCode(error_code)
                except ValueError:
                    tftp_error = ErrorCode.NOT_DEFINED
                raise TFTPError(tftp_error, error_message)
            # Ignore packets with unexpected opcodes and keep waiting
            continue
        except TimeoutError:
            if attempt < retries - 1:
                continue
            raise
    return 0


def _parse_discover_response(data: bytes) -> ServerRole | None:
    """
    Parse a response to the discover request.

    :return: The server role, or None if the packet is not a valid discover response.
    """
    if len(data) < 2:
        return None
    opcode = struct.unpack("!H", data[:2])[0]
    if opcode == OpCode.DISCOVER_REQUEST.value:
        # Legacy response; only the bootloader implemented the TFTP server at that time
        return ServerRole.BOOTLOADER
    if opcode == OpCode.DISCOVER_RESPONSE.value and len(data) >= 4:
        role = struct.unpack("!H", data[2:4])[0]
        try:
            return ServerRole(role)
        except ValueError:
            return None
    return None


def discover_server(address: str, timeout: float = 5.0, port: int = 69) -> ServerRole:
    """
    Discover the TFTP server on the target device and determine its role.

    :return: The role of the device software running the TFTP server.
    :raises TimeoutError: If no valid discover response is received within the timeout period.
    """
    deadline = time.monotonic() + timeout
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(_build_discover_request(), (address, port))
        while (remaining := deadline - time.monotonic()) > 0:
            s.settimeout(remaining)
            data, _ = s.recvfrom(512)
            role = _parse_discover_response(data)
            if role is not None:
                return role
            # Ignore unexpected packets and keep waiting
        raise TimeoutError
    finally:
        s.close()


def write_binary(
    address: str,
    path: Path,
    *,
    is_bootloader: bool = False,
    timeout: float = 3.0,
    retries: int = 3,
    port: int = 69,
    progress_callback: Callable[[int, int], None] | None = None,
) -> None:
    """
    Write a binary file to the target device using TFTP.

    :param progress_callback: Optional callback invoked after each block is acknowledged.
        Called with (bytes_sent, total_bytes).
    :raises TimeoutError: If the server does not respond within the timeout period.
    :raises TFTPError: If a TFTP error is received during the transfer.
    """
    filename = BOOTLOADER_FILE if is_bootloader else FIRMWARE_FILE
    file_data = path.read_bytes()
    total_bytes = len(file_data)

    if progress_callback is not None:
        progress_callback(0, total_bytes)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)

    try:
        block_number = _send_packet_and_wait_for_ack(
            s,
            _build_write_request(filename),
            address,
            retries=retries,
            port=port,
        )
        if block_number != 0:
            raise TFTPError(
                ErrorCode.ILLEGAL_OPERATION,
                f"Expected ACK for block 0, got ACK for block {block_number}",
            )

        bytes_sent = 0
        for file_index in range(0, total_bytes + 1, 512):
            block_number = block_number + 1
            chunk_size = min(512, total_bytes - file_index)
            data_chunk = file_data[file_index : file_index + chunk_size]

            ack_block_number = _send_packet_and_wait_for_ack(
                s,
                _build_data_packet(block_number, data_chunk),
                address,
                retries=retries,
                port=port,
            )
            if ack_block_number != block_number:
                raise TFTPError(
                    ErrorCode.ILLEGAL_OPERATION,
                    f"Expected ACK for block {block_number}, got ACK for block {ack_block_number}",
                )

            bytes_sent += chunk_size
            if progress_callback is not None:
                progress_callback(bytes_sent, total_bytes)
    finally:
        s.close()
