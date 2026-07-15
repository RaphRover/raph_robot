import argparse
import math
import time
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from raph_interfaces.msg import PowerSystemState
from raph_interfaces.srv import GetControllerInfo, GetOsVersion
from rclpy.node import Node
from sensor_msgs.msg import Imu

from raph_fw.console import get_logger, log_step
from raph_fw.resolve import resolve_raphcore_name
from raph_fw.versions import get_bootloader_version, get_firmware_version

MESSAGE_TIMEOUT = 5.0
SERVICE_TIMEOUT = 5.0
IMU_TEST_TIMEOUT = 10.0
IMU_SAMPLES = 20
IMU_ANGULAR_TOLERANCE = 0.0025
IMU_LINEAR_XY_TOLERANCE = 0.045
IMU_GRAVITY_TOLERANCE = 0.08
BATTERY_MIN_VOLTAGE = 18.0
BATTERY_MAX_VOLTAGE = 24.86
BOOTLOADER_BINARY_NAME = "raphcore_bootloader_latest.bin"
FIRMWARE_BINARY_NAME = "raphcore_firmware_latest.bin"
EXPECTED_RAPH_OS_VERSION = "1.0.0"


class HardwareTestCommand:
    """Command to validate basic Raph robot hardware behavior."""

    def __init__(self) -> None:
        self.logger = get_logger("HardwareTestCommand")
        self.node: Node | None = None
        self.latest_imu: Imu | None = None
        self.new_imu_received: bool = False
        self.latest_power_state: PowerSystemState | None = None

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """Add command-line arguments for the hardware test command."""
        # parser.add_argument("--config", type=str, help="Path to the hardware test config file.")
        pass

    def main(self, args: argparse.Namespace) -> None:
        """Execute the hardware tests."""
        self.logger.info("Starting hardware tests.")
        self._check_raphcore_network()
        self.exit_code = 0
        results: list[tuple[str, bool]] = []

        self._setup_ros()
        results.append(("RaphCore firmware and bootloader version", self._test_controller_info()))
        results.append(("IMU", self._test_imu()))
        results.append(("Battery voltage", self._test_battery_voltage()))
        results.append(("RaphOS version", self._test_raph_os_version()))
        #TODO battery disconnection test
        #TODO wheel firmware version test
        #TODO wheel motor test

        self._shutdown_ros()
        self._parse_results(results)

        sys.exit(self.exit_code)
    
    def _parse_results(self, results: list[tuple[str, bool]]) -> None:
        failed = [result for result in results if not result[1]]

        if failed:
            self.exit_code = 1
            failed_names = ", ".join(result[0] for result in failed)
            self.logger.error(
                f"Hardware test finished with {len(failed)} failing check(s): {failed_names}."
            )
        else:
            self.logger.info(f"Hardware test finished successfully. {len(results)} check(s) passed.")

    def _check_raphcore_network(self) -> None:
        try:
            with log_step(
                "Attempting to resolve RaphCore on the network",
            ):
                addresses = resolve_raphcore_name()
        except TimeoutError:
            self.logger.exception(
                "Failed to resolve address of the RaphCore on the network. "
                "Ensure that RaphCore is powered and connected to the router properly.",
            )
            sys.exit(1)
        if len(addresses) >= 1:
            self.logger.info(f"Resolved RaphCore device at {addresses[0]}")
            

    def _setup_ros(self) -> None:
        rclpy.init(args=None)
        self.node = Node("raph_hardware_tester")

        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT

        self.imu_sub = self.node.create_subscription(
            Imu,
            "controller/imu/data",
            self._imu_callback,
            qos_profile,
        )
        self.power_system_state_sub = self.node.create_subscription(
            PowerSystemState,
            "controller/power_system_state",
            self._power_system_state_callback,
            qos_profile,
        )
        self.controller_info_client = self.node.create_client(
            GetControllerInfo,
            "controller/get_controller_info",
        )
        self.raph_os_version_client = self.node.create_client(
            GetOsVersion,
            "raph_system/get_os_version",
        )
    def _shutdown_ros(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _imu_callback(self, msg: Imu) -> None:
        self.latest_imu = msg
        self.new_imu_received = True

    def _power_system_state_callback(self, msg: PowerSystemState) -> None:
        self.latest_power_state = msg

    def _spin_once(self, timeout_sec: float) -> None:
        if self.node and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _test_controller_info(self) -> bool:
        try:
            with log_step("Checking controller firmware and bootloader versions"):
                if not self.controller_info_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
                    raise TimeoutError("Timed out waiting for GetControllerInfo service server to start.")
                expected_bootloader_version, expected_firmware_version = (
                    self._get_expected_controller_versions()
                )

                future = self.controller_info_client.call_async(GetControllerInfo.Request())
                rclpy.spin_until_future_complete(self.node, future, None, SERVICE_TIMEOUT)
                if not future.done() or future.result() is None:
                    raise TimeoutError("Controller info service did not respond in time")

                response = future.result()
                if response.bootloader_version != expected_bootloader_version:
                    raise ValueError(
                        "Controller bootloader version mismatch: "
                        f"expected {expected_bootloader_version}, "
                        f"got {response.bootloader_version}"
                    )
                if response.firmware_version != expected_firmware_version:
                    raise ValueError(
                        "Controller firmware version mismatch: "
                        f"expected {expected_firmware_version}, "
                        f"got {response.firmware_version}"
                    )
        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            return False
        else:
            self.logger.info(f"Bootloader version: {response.bootloader_version}, Firmware version: {response.firmware_version}")
            return True

    def _get_expected_controller_versions(self) -> tuple[str, str]:
        share_dir = Path(get_package_share_directory("raph_fw"))
        bootloader_bin = share_dir / "data" / "bootloader" / BOOTLOADER_BINARY_NAME
        firmware_bin = share_dir / "data" / "firmware" / FIRMWARE_BINARY_NAME
        return (
            get_bootloader_version(bootloader_bin),
            get_firmware_version(firmware_bin),
        )

    def _test_imu(self) -> bool:
        try:
            with log_step(f"Validating IMU data"):
                samples = 0
                deadline = time.monotonic() + IMU_TEST_TIMEOUT

                while samples < IMU_SAMPLES:
                    self._spin_once(0.1)
                    imu = self.latest_imu
                    if imu is None or not self.new_imu_received:
                        continue

                    if abs(imu.angular_velocity.x) > IMU_ANGULAR_TOLERANCE:
                        raise ValueError(
                            f"IMU angular_velocity.x={imu.angular_velocity.x:.3f} rad/s exceeds tolerance"
                        )
                    if abs(imu.angular_velocity.y) > IMU_ANGULAR_TOLERANCE:
                        raise ValueError(
                            f"IMU angular_velocity.y={imu.angular_velocity.y:.3f} rad/s exceeds tolerance"
                        )
                    if abs(imu.angular_velocity.z) > IMU_ANGULAR_TOLERANCE:
                        raise ValueError(
                            f"IMU angular_velocity.z={imu.angular_velocity.z:.3f} rad/s exceeds tolerance"
                        )
                    if abs(imu.linear_acceleration.x) > IMU_LINEAR_XY_TOLERANCE:
                        raise ValueError(
                            f"IMU linear_acceleration.x={imu.linear_acceleration.x:.3f} m/s^2 exceeds tolerance"
                        )
                    if abs(imu.linear_acceleration.y) > IMU_LINEAR_XY_TOLERANCE:
                        raise ValueError(
                            f"IMU linear_acceleration.y={imu.linear_acceleration.y:.3f} m/s^2 exceeds tolerance"
                        )
                    if abs(imu.linear_acceleration.z - 9.81) > IMU_GRAVITY_TOLERANCE:
                        raise ValueError(
                            f"IMU linear_acceleration.z={imu.linear_acceleration.z:.3f} m/s^2 exceeds tolerance"
                        )
                    samples += 1
                    self.new_imu_received = False
                    if time.monotonic() > deadline:
                        raise TimeoutError("IMU test timed out before receiving enough valid samples.")
        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            self.logger.error("IMU test failed. Ensure that the robot is stationary and IMU data is being published.")
            return False
        return True

    def _test_battery_voltage(self) -> bool:
        try:
            with log_step(
                "Checking battery connection and voltage levels",
            ):
                if self.latest_power_state is None:
                    self._wait_for_power_state_message()
                
                power_state = self.latest_power_state
                failures: list[str] = []

                if not power_state.bat1_connected:
                    failures.append("bat1 is not connected")
                if not power_state.bat2_connected:
                    failures.append("bat2 is not connected")

                for battery_name, connected, voltage in (
                    ("bat1", power_state.bat1_connected, power_state.bat1_state.voltage),
                    ("bat2", power_state.bat2_connected, power_state.bat2_state.voltage),
                ):
                    if not connected:
                        continue
                    if voltage < BATTERY_MIN_VOLTAGE:
                        failures.append(
                            f"{battery_name} voltage {voltage:.2f} V is below {BATTERY_MIN_VOLTAGE:.2f} V"
                        )
                    if voltage > BATTERY_MAX_VOLTAGE:
                        failures.append(
                            f"{battery_name} voltage {voltage:.2f} V is above {BATTERY_MAX_VOLTAGE:.2f} V"
                        )

                if failures:
                    raise ValueError("; ".join(failures))
        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            return False
        return True

    def _wait_for_power_state_message(self) -> None:
        deadline = time.monotonic() + MESSAGE_TIMEOUT
        while self.latest_power_state is None and time.monotonic() < deadline:
            self._spin_once(0.1)
        if self.latest_power_state is None:
            raise TimeoutError("Failed to receive battery state message in time. Make sure that ROS is running and the controller topics are visible.")
        
    def _test_raph_os_version(self) -> bool:
        try:
            with log_step(
                "Checking RaphOS version",
            ):
                if not self.raph_os_version_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
                    raise TimeoutError("Timed out waiting for GetOsVersion service server to start.")

                future = self.raph_os_version_client.call_async(GetOsVersion.Request())
                rclpy.spin_until_future_complete(self.node, future, None, SERVICE_TIMEOUT)
                if not future.done() or future.result() is None:
                    raise TimeoutError("RaphOS version service did not respond in time")

                response = future.result()
                if response.version != EXPECTED_RAPH_OS_VERSION:
                    raise ValueError(
                        "RaphOS version mismatch: "
                        f"expected {EXPECTED_RAPH_OS_VERSION}, "
                        f"got {response.version}"
                    )
        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            return False
        else:
            self.logger.info(f"RaphOS version: {response.version}")
            return True