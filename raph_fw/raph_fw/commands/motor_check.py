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

import sys
import time

import rclpy
from raph_interfaces.msg import ServoCommand, WheelCommand
from rclpy.node import Node
from std_srvs.srv import Trigger

from raph_fw.console import get_choice_prompt, get_logger, log_step

SERVICE_TIMEOUT = 5.0

WHEEL_TARGET_VELOCITY = 0.5
WHEEL_ACCELERATION_DIVIDER = 2

SERVO_TARGET_VELOCITY = 1.5
SERVO_ACCELERATION_DIVIDER = 10

MOVE_DURATION = 3.0
PUBLISH_RATE = 20.0

_WHEEL_MOTORS = [
    ("Front-left wheel", "controller/cmd_wheel_fl"),
    ("Front-right wheel", "controller/cmd_wheel_fr"),
    ("Rear-left wheel", "controller/cmd_wheel_rl"),
    ("Rear-right wheel", "controller/cmd_wheel_rr"),
]

_SERVO_MOTORS = [
    ("Left servo", "controller/cmd_servo_l", -1.57),
    ("Right servo", "controller/cmd_servo_r", 1.57),
]

_CHOICE_ALL = "All motors (sequential)"
_CHOICE_EXIT = "Exit"

_CHOICES = (
    [name for name, _ in _WHEEL_MOTORS]
    + [name for name, _, _t in _SERVO_MOTORS]
    + [_CHOICE_ALL, _CHOICE_EXIT]
)


class MotorCheckCommand:
    """Command to interactively test individual motors on the Raph robot."""

    def __init__(self) -> None:
        """Initialize the MotorCheckCommand."""
        self.logger = get_logger("MotorCheckCommand")
        self.node: Node | None = None

    def main(self) -> None:
        """Execute the interactive motor check."""
        self._setup_ros()

        try:
            self._calibrate_servos()
            self._run_interactive_loop()
        finally:
            self._shutdown_ros()

    def _setup_ros(self) -> None:
        """Initialize ROS, create the node, and register publishers and service clients."""
        rclpy.init(args=None)
        self.node = Node("raph_motor_checker")

        self._wheel_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for name, topic in _WHEEL_MOTORS:
            self._wheel_pubs[name] = self.node.create_publisher(WheelCommand, topic, 10)

        self._servo_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._servo_targets: dict[str, float] = {}
        for name, topic, target in _SERVO_MOTORS:
            self._servo_pubs[name] = self.node.create_publisher(ServoCommand, topic, 10)
            self._servo_targets[name] = target

        self._calibrate_servos_client = self.node.create_client(
            Trigger,
            "controller/calibrate_servos",
        )

    def _shutdown_ros(self) -> None:
        """Destroy the ROS node and shut down rclpy."""
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _calibrate_servos(self) -> None:
        """Call the calibrate_servos service and abort on failure."""
        with log_step("Waiting for calibrate_servos service"):
            if not self._calibrate_servos_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
                self.logger.error(
                    "calibrate_servos service not available. "
                    "Ensure the robot controller is running.",
                )
                sys.exit(1)

        with log_step("Calibrating servos"):
            future = self._calibrate_servos_client.call_async(Trigger.Request())
            assert self.node is not None
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=SERVICE_TIMEOUT)
            if future.result() is None:
                self.logger.error("calibrate_servos service call timed out.")
                sys.exit(1)
            result: Trigger.Response = future.result()
            if not result.success:
                self.logger.error(f"Servo calibration failed: {result.message}")
                sys.exit(1)

    def _publish_for_duration(
        self,
        pub: rclpy.publisher.Publisher,
        msg: object,
        duration: float,
    ) -> None:
        """Publish *msg* on *pub* at PUBLISH_RATE Hz for *duration* seconds."""
        interval = 1.0 / PUBLISH_RATE
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            pub.publish(msg)
            time.sleep(interval)

    def _move_all_motors(self) -> None:
        """Move each motor one by one in sequence."""
        for name, _ in _WHEEL_MOTORS:
            msg = WheelCommand(
                enabled=True,
                target_velocity=WHEEL_TARGET_VELOCITY,
                acceleration_divider=WHEEL_ACCELERATION_DIVIDER,
            )
            self.logger.info(f"Moving {name} at {WHEEL_TARGET_VELOCITY} rad/s.")
            self._publish_for_duration(self._wheel_pubs[name], msg, MOVE_DURATION)
            self._wheel_pubs[name].publish(
                WheelCommand(
                    enabled=False,
                    target_velocity=0.0,
                    acceleration_divider=WHEEL_ACCELERATION_DIVIDER,
                ),
            )
            self.logger.info(f"Disabling {name}.")

        for name, _, target in _SERVO_MOTORS:
            msg = ServoCommand(
                enabled=True,
                target_position=target,
                target_velocity=SERVO_TARGET_VELOCITY,
                acceleration_divider=SERVO_ACCELERATION_DIVIDER,
            )
            self.logger.info(f"Moving {name} to {target} rad at {SERVO_TARGET_VELOCITY} rad/s.")
            self._publish_for_duration(self._servo_pubs[name], msg, MOVE_DURATION / 2)
            msg.target_position = 0.0
            self.logger.info(f"Moving {name} to 0 rad at {SERVO_TARGET_VELOCITY} rad/s.")
            self._publish_for_duration(self._servo_pubs[name], msg, MOVE_DURATION / 2)
            msg.enabled = False
            self.logger.info(f"Disabling {name}.")
            self._servo_pubs[name].publish(msg)

    def _run_interactive_loop(self) -> None:
        """Present the motor selection menu and dispatch commands until Exit is chosen."""
        self.logger.info("Remember to have the robot placed on a stand before moving the motors!")
        while True:
            choice = get_choice_prompt("Select a motor to move:", _CHOICES)

            if choice == _CHOICE_EXIT:
                break

            if choice == _CHOICE_ALL:
                self._move_all_motors()
            elif choice in self._wheel_pubs:
                msg = WheelCommand(
                    enabled=True,
                    target_velocity=WHEEL_TARGET_VELOCITY,
                    acceleration_divider=WHEEL_ACCELERATION_DIVIDER,
                )
                self.logger.info(f"Moving {choice} at {WHEEL_TARGET_VELOCITY} rad/s.")
                self._publish_for_duration(self._wheel_pubs[choice], msg, MOVE_DURATION)
                stop_msg = WheelCommand(
                    enabled=False,
                    target_velocity=0.0,
                    acceleration_divider=WHEEL_ACCELERATION_DIVIDER,
                )
                self._wheel_pubs[choice].publish(stop_msg)
                self.logger.info(f"Disabling {choice}.")
            elif choice in self._servo_pubs:
                target = self._servo_targets[choice]
                msg = ServoCommand(
                    enabled=True,
                    target_position=target,
                    target_velocity=SERVO_TARGET_VELOCITY,
                    acceleration_divider=SERVO_ACCELERATION_DIVIDER,
                )
                self.logger.info(
                    f"Moving {choice} to {target} rad at {SERVO_TARGET_VELOCITY} rad/s.",
                )
                self._publish_for_duration(self._servo_pubs[choice], msg, MOVE_DURATION / 2)
                msg.target_position = 0.0
                self.logger.info(f"Moving {choice} to 0 rad at {SERVO_TARGET_VELOCITY} rad/s.")
                self._publish_for_duration(self._servo_pubs[choice], msg, MOVE_DURATION / 2)
                msg.enabled = False
                self.logger.info(f"Disabling {choice}.")
                self._servo_pubs[choice].publish(msg)
