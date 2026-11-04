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

"""Firmware status checks must remain asynchronous and fail explicitly."""

import json
import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import raph_system
import rclpy
from raph_interfaces.srv import GetControllerInfo, GetFirmwareStatus, GetOsVersion
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


@pytest.fixture
def system(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[raph_system.RaphSystem]:
    """Use a separate namespace and a temporary notification cache."""
    monkeypatch.setattr(raph_system, "read_target_version", lambda: "1.1.0")
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            "__ns:=/firmware_notification_test",
            "-p",
            f"firmware_status_path:={tmp_path}/status.json",
        ],
    )
    node = raph_system.RaphSystem()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_controller_and_status_services(system: raph_system.RaphSystem) -> None:
    """A real ROS round trip publishes the same result to API and cache."""
    controller = Node("controller")
    observer = Node("observer")
    executor = SingleThreadedExecutor()

    def controller_info(
        _request: GetControllerInfo.Request,
        response: GetControllerInfo.Response,
    ) -> GetControllerInfo.Response:
        response.firmware_version = "1.0.0"
        response.bootloader_version = "1.0.0"
        return response

    controller.create_service(GetControllerInfo, "controller/get_controller_info", controller_info)
    firmware_client = observer.create_client(
        GetFirmwareStatus,
        "raph_system/get_firmware_status",
    )
    os_client = observer.create_client(GetOsVersion, "raph_system/get_os_version")
    for node in (system, controller, observer):
        executor.add_node(node)
    try:
        assert firmware_client.wait_for_service(timeout_sec=3)
        assert os_client.wait_for_service(timeout_sec=3)
        firmware_future = firmware_client.call_async(GetFirmwareStatus.Request(refresh=True))
        os_future = os_client.call_async(GetOsVersion.Request())
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if firmware_future.done() and os_future.done() and not system.firmware_status.checking:
                break
        assert firmware_future.done()
        assert os_future.done()
        assert firmware_future.result().checking is True
        assert system.firmware_status.status == "update_available"
        snapshot = json.loads(system.firmware_status_path.read_text())
        cached = firmware_client.call_async(GetFirmwareStatus.Request(refresh=False))
        executor.spin_until_future_complete(cached, timeout_sec=3)
        assert cached.done()
        result = cached.result()
        assert result.status == snapshot["status"]
        assert result.current_version == snapshot["current_version"] == "1.0.0"
        assert result.target_version == snapshot["target_version"] == "1.1.0"
        assert result.checking is False
        assert result.checked_at > 0
    finally:
        executor.shutdown()
        controller.destroy_node()
        observer.destroy_node()


def test_timeout_and_late_response(system: raph_system.RaphSystem) -> None:
    """Timeout clears pending work and ignores responses from an expired check."""
    future = Future()
    system._firmware_future = future
    system._firmware_deadline = 0.0
    system._check_firmware_timeout()
    assert system.firmware_status.status == "unavailable"
    assert not system.firmware_status.checking
    assert "Timed out" in system.firmware_status.status_message
    assert system._firmware_future is None
    system._controller_info_done(future)
    assert system.firmware_status.status == "unavailable"
    snapshot = json.loads(system.firmware_status_path.read_text())
    assert snapshot["status"] == "unavailable"


def test_transport_failure(system: raph_system.RaphSystem) -> None:
    """Future errors become explicit unavailable status."""
    future = Future()
    system._firmware_future = future
    future.set_exception(RuntimeError("transport failed"))
    system._controller_info_done(future)
    assert system.firmware_status.status == "unavailable"
    assert "transport failed" in system.firmware_status.status_message


def test_empty_controller_version(system: raph_system.RaphSystem) -> None:
    """Empty responses never recommend an upgrade."""
    future = Future()
    system._firmware_future = future
    future.set_result(GetControllerInfo.Response())
    system._controller_info_done(future)
    assert system.firmware_status.status == "unavailable"
    assert "empty firmware version" in system.firmware_status.status_message


def test_response_after_deadline(system: raph_system.RaphSystem) -> None:
    """A late response is rejected even before the timeout timer fires."""
    future = Future()
    system._firmware_future = future
    system._firmware_deadline = 0.0
    future.set_result(GetControllerInfo.Response(firmware_version="1.0.0"))
    system._controller_info_done(future)
    assert system.firmware_status.status == "unavailable"
    assert "Timed out" in system.firmware_status.status_message


def test_periodic_check_interval(
    system: raph_system.RaphSystem,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Successful checks wait 15 minutes and failures retry after 30 seconds."""
    system._finish_firmware_check()
    next_check = system._next_firmware_check
    assert 899 <= next_check - time.monotonic() <= 900
    system._periodic_firmware_check()
    assert not system.firmware_status.checking
    monkeypatch.setattr(raph_system.time, "monotonic", lambda: next_check)
    system._periodic_firmware_check()
    assert system.firmware_status.checking
    system._finish_firmware_check("Unavailable")
    assert system._next_firmware_check == next_check + 30


def test_missing_image(
    system: raph_system.RaphSystem,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A failed target read is returned to API clients and persisted for SSH."""
    system._firmware_deadline = 0.0
    system._check_firmware_timeout()

    def missing_image() -> str:
        message = "firmware.bin"
        raise FileNotFoundError(message)

    monkeypatch.setattr(raph_system, "read_target_version", missing_image)
    response = system._get_firmware_status_callback(
        GetFirmwareStatus.Request(refresh=True),
        GetFirmwareStatus.Response(),
    )
    assert response.status == "unavailable"
    assert response.checking is False
    assert "firmware.bin" in response.status_message


def test_cache_failure_keeps_ros_status(
    system: raph_system.RaphSystem,
    tmp_path: Path,
) -> None:
    """An unprovisioned cache does not disable the ROS status API."""
    system.firmware_status_path = tmp_path / "missing" / "status.json"
    system._finish_firmware_check("Controller unavailable")
    assert system.firmware_status.status == "unavailable"
    assert system.firmware_status.status_message == "Controller unavailable"
