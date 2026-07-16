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
import time
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from raph_interfaces.msg import MotorDiagnostics, PowerSystemState
from raph_interfaces.srv import GetControllerInfo, GetOsVersion
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from raph_fw.console import get_logger, log_step
from raph_fw.resolve import resolve_raphcore_name
from raph_fw.versions import get_bootloader_version, get_firmware_version

MESSAGE_TIMEOUT = 5.0
SERVICE_TIMEOUT = 5.0
IMU_TEST_TIMEOUT = 10.0
IMU_SAMPLES = 20
IMU_ANGULAR_TOLERANCE = 0.0035
IMU_LINEAR_XY_TOLERANCE = 0.05
IMU_GRAVITY_TOLERANCE = 0.08
BATTERY_MIN_VOLTAGE = 18.0
BATTERY_MAX_VOLTAGE = 24.86
MOTOR_DIAGNOSTIC_SAMPLES = 10
MOTOR_DIAGNOSTIC_TIMEOUT = 10.0
BOOTLOADER_BINARY_NAME = "raphcore_bootloader_latest.bin"
FIRMWARE_BINARY_NAME = "raphcore_firmware_latest.bin"
EXPECTED_RAPH_OS_VERSION = "1.0.0"
EXPECTED_MOTOR_FIRMWARE = "hw34 sw3.3 24.06.03"
#TODO: currently this is the wrong order but it needs to be fixed in the msg or the firmware
MOTOR_NAMES = (
    "rear left wheel",
    "rear right wheel",
    "right servo",
    "left servo",
    "front left wheel",
    "front right wheel",
)
MOTOR_FIRMWARE_KEYS = (
    "wheel_rl_firmware",
    "wheel_rr_firmware",
    "wheel_fl_firmware",
    "wheel_fr_firmware",
    "servo_l_firmware",
    "servo_r_firmware",
)


class HardwareTestCommand:
    """Command to validate basic Raph robot hardware behavior."""

    def __init__(self) -> None:
        """Initialize the HardwareTestCommand."""
        self.logger = get_logger("HardwareTestCommand")
        self.node: Node | None = None
        self.latest_imu: Imu | None = None
        self.new_imu_received: bool = False
        self.latest_power_state: PowerSystemState | None = None
        self.latest_motor_diagnostics: MotorDiagnostics | None = None
        self.new_motor_diagnostics_received: bool = False

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """
        Add command-line arguments for the hardware test command.

        :param parser: The argument parser to add arguments to.
        """
        # parser.add_argument("--config", type=str, help="Path to the hardware test config file.")
        # TODO: figure out what needs to be parametrized for hardware test
        pass

    def main(self, args: argparse.Namespace) -> None:
        """
        Execute the hardware tests.

        :param args: Parsed command-line arguments.
        """
        self.logger.info("Starting hardware tests.")
        self._check_raphcore_network()
        self.exit_code = 0
        results: list[tuple[str, bool]] = []

        self._setup_ros()
        results.append(("Motor firmware, RaphCore firmware and bootloader version", self._test_controller_info()))
        results.append(("IMU", self._test_imu()))
        results.append(("Battery voltage", self._test_battery_voltage()))
        results.append(("RaphOS version", self._test_raph_os_version()))
        results.append(("Motor diagnostics and servo calibration", self._test_motors()))

        self._shutdown_ros()
        self._parse_results(results)

        sys.exit(self.exit_code)
    
    def _parse_results(self, results: list[tuple[str, bool]]) -> None:
        """
        Parse and report the final hardware test results.

        :param results: A list of tuples containing test names and pass/fail status.
        """
        failed = [result for result in results if not result[1]]

        if failed:
            self.exit_code = 1
            failed_names = "; ".join(result[0] for result in failed)
            self.logger.error(
                f"Hardware test finished with {len(failed)} failing check(s): {failed_names}."
            )
        else:
            self.logger.info(f"Hardware test finished successfully. {len(results)} check(s) passed.")

    def _check_raphcore_network(self) -> None:
        """Resolve the RaphCore device on the network and abort on failure."""
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
        """Initialize ROS, create the node, and register subscriptions and service clients."""
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
        self.motor_diagnostics_sub = self.node.create_subscription(
            MotorDiagnostics,
            "controller/diagnostics/motors",
            self._motor_diagnostics_callback,
            qos_profile,
        )
        self.controller_info_client = self.node.create_client(
            GetControllerInfo,
            "controller/get_controller_info",
        )
        self.calibrate_servos_client = self.node.create_client(
            Trigger,
            "controller/calibrate_servos",
        )
        self.raph_os_version_client = self.node.create_client(
            GetOsVersion,
            "raph_system/get_os_version",
        )
    def _shutdown_ros(self) -> None:
        """Destroy the ROS node and shut down rclpy if it is still running."""
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

    def _motor_diagnostics_callback(self, msg: MotorDiagnostics) -> None:
        self.latest_motor_diagnostics = msg
        self.new_motor_diagnostics_received = True

    def _spin_once(self, timeout_sec: float) -> None:
        """
        Spin the ROS node once to process callbacks.

        :param timeout_sec: Maximum time in seconds to wait for one spin iteration.
        """
        if self.node and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _test_controller_info(self) -> bool:
        """
        Validate controller, bootloader, and motor firmware versions.

        :returns: True if all version checks pass, otherwise False.
        """
        try:
            with log_step("Checking motor firmware, controller firmware and bootloader versions"):
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
                extra_info = {kv.key: kv.value for kv in response.extra_information}
                motor_mismatches: list[str] = []
                for key in MOTOR_FIRMWARE_KEYS:
                    actual = extra_info.get(key)
                    if actual is None:
                        motor_mismatches.append(f"{key}: missing from response")
                    elif actual != EXPECTED_MOTOR_FIRMWARE:
                        motor_mismatches.append(
                            f"{key}: expected '{EXPECTED_MOTOR_FIRMWARE}', got '{actual}'"
                        )
                if motor_mismatches:
                    raise ValueError(
                        "Motor firmware version mismatch(es): " + "; ".join(motor_mismatches)
                    )

        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            return False
        else:
            self.logger.info(
                f"Bootloader version: {response.bootloader_version}, "
                f"Firmware version: {response.firmware_version}, "
                f"Motor firmware (all motors): {EXPECTED_MOTOR_FIRMWARE}"
            )
            return True

    def _get_expected_controller_versions(self) -> tuple[str, str]:
        """
        Read expected bootloader and firmware versions from packaged binaries.

        :returns: A tuple of (bootloader_version, firmware_version).
        """
        share_dir = Path(get_package_share_directory("raph_fw"))
        bootloader_bin = share_dir / "data" / "bootloader" / BOOTLOADER_BINARY_NAME
        firmware_bin = share_dir / "data" / "firmware" / FIRMWARE_BINARY_NAME
        return (
            get_bootloader_version(bootloader_bin),
            get_firmware_version(firmware_bin),
        )

    def _test_imu(self) -> bool:
        """
        Validate IMU readings while the robot is stationary.

        :returns: True if all IMU checks pass, otherwise False.
        """
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
        """
        Validate battery connectivity and voltage ranges.

        :returns: True if battery checks pass, otherwise False.
        """
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
        """Wait for a power system state message until timeout."""
        deadline = time.monotonic() + MESSAGE_TIMEOUT
        while self.latest_power_state is None and time.monotonic() < deadline:
            self._spin_once(0.1)
        if self.latest_power_state is None:
            raise TimeoutError("Failed to receive battery state message in time. Make sure that ROS is running and the controller topics are visible.")
        
    def _test_raph_os_version(self) -> bool:
        """
        Validate the reported RaphOS version.

        :returns: True if the OS version matches the expected value, otherwise False.
        """
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

    def _test_motors(self) -> bool:
        """
        Validate motor diagnostics before and after servo calibration.

        :returns: True if diagnostics and calibration checks pass, otherwise False.
        """
        try:
            with log_step("Checking motor diagnostics and calibrating servos"):
                pre_calibration_samples = self._collect_motor_diagnostics_samples(
                    MOTOR_DIAGNOSTIC_SAMPLES,
                    MOTOR_DIAGNOSTIC_TIMEOUT,
                )
                self._validate_motor_diagnostics_samples(
                    pre_calibration_samples,
                    "before servo calibration",
                )

                if not self.calibrate_servos_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
                    raise TimeoutError("Timed out waiting for calibrate_servos service server to start.")

                calibration_future = self.calibrate_servos_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self.node, calibration_future, None, SERVICE_TIMEOUT)
                if not calibration_future.done() or calibration_future.result() is None:
                    raise TimeoutError("Servo calibration service did not respond in time")

                calibration_response = calibration_future.result()
                if not calibration_response.success:
                    raise ValueError(f"Servo calibration failed: {calibration_response.message}")

                post_calibration_samples = self._collect_motor_diagnostics_samples(
                    MOTOR_DIAGNOSTIC_SAMPLES,
                    MOTOR_DIAGNOSTIC_TIMEOUT,
                )
                self._validate_motor_diagnostics_samples(
                    post_calibration_samples,
                    "after servo calibration",
                )
        except (TimeoutError, ValueError) as exc:
            self.logger.error(str(exc))
            return False
        return True

    def _collect_motor_diagnostics_samples(
        self,
        sample_count: int,
        timeout_sec: float,
    ) -> list[MotorDiagnostics]:
        """
        Collect a fixed number of fresh motor diagnostics messages.

        :param sample_count: Number of diagnostics messages to collect.
        :param timeout_sec: Maximum time in seconds allowed for collection.
        :returns: A list of collected motor diagnostics messages.
        """
        deadline = time.monotonic() + timeout_sec
        samples: list[MotorDiagnostics] = []

        while len(samples) < sample_count and time.monotonic() < deadline:
            self._spin_once(0.1)
            if self.latest_motor_diagnostics is None or not self.new_motor_diagnostics_received:
                continue
            samples.append(self.latest_motor_diagnostics)
            self.new_motor_diagnostics_received = False

        if len(samples) < sample_count:
            raise TimeoutError(
                "Timed out waiting for motor diagnostics messages: "
                f"received {len(samples)}/{sample_count} different messages"
            )
        return samples

    def _validate_motor_diagnostics_samples(
        self,
        samples: list[MotorDiagnostics],
        stage: str,
    ) -> None:
        """
        Validate motor diagnostics samples for active motor and communication faults.

        :param samples: Motor diagnostics messages to validate.
        :param stage: A label describing validation stage used in error messages.
        """

        active_fault_mask_by_motor: dict[str, int] = {}
        communication_fault_active_by_motor: set[str] = set()

        for sample in samples:
            for motor_index, motor_name in enumerate(MOTOR_NAMES):
                active_mask = sample.fault_mask_active[motor_index]
                comm_active = sample.communication_fault_active[motor_index]

                if active_mask != 0:
                    active_fault_mask_by_motor[motor_name] = (
                        active_fault_mask_by_motor.get(motor_name, 0) | active_mask
                    )
                if comm_active:
                    communication_fault_active_by_motor.add(motor_name)

        faults: list[str] = []
        for motor_name in MOTOR_NAMES:
            active_mask = active_fault_mask_by_motor.get(motor_name, 0)
            comm_active = motor_name in communication_fault_active_by_motor

            if active_mask != 0:
                faults.append(f"{motor_name} has active motor fault mask {active_mask}")
            if comm_active:
                faults.append(f"{motor_name} has active communication fault")

        if faults:
            raise ValueError(f"Motor diagnostics check failed {stage}: " + "; ".join(faults))
