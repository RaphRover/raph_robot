#!/usr/bin/env python3

"""A module containing the PDSPipeGuide node implementation."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quaternion_to_roll(x: float, y: float, z: float, w: float) -> float:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    return math.atan2(sinr_cosp, cosr_cosp)


class PDSPipeGuide(Node):
    kp: float
    kd: float
    max_steering_angle: float
    inverse_steering_angle: bool

    def __init__(self) -> None:
        super().__init__("pds_pipe_guide")

        self.declare_parameter(
            "kp",
            1.0,
            ParameterDescriptor(
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.0)]
            ),
        )
        self.declare_parameter(
            "kd",
            0.0,
            ParameterDescriptor(
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.0)]
            ),
        )
        self.declare_parameter(
            "max_steering_angle",
            0.54,
            ParameterDescriptor(
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.54, step=0.0)]
            ),
        )
        self.declare_parameter("inverse_steering_angle", value=False)

        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.max_steering_angle = (
            self.get_parameter("max_steering_angle").get_parameter_value().double_value
        )
        self.inverse_steering_angle = (
            self.get_parameter("inverse_steering_angle").get_parameter_value().bool_value
        )

        self.add_post_set_parameters_callback(self._post_parameters_changed)

        self.imu_sub = self.create_subscription(
            Imu,
            "controller/imu/data",
            self._imu_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.cmd_sub = self.create_subscription(
            AckermannDrive,
            "~/cmd_ackermann",
            self._cmd_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(
            AckermannDrive,
            "controller/cmd_ackermann",
            10,
        )

        self.roll = 0.0
        self.roll_rate = 0.0

        self.get_logger().info("PDS Pipe Guide node started!")

    def _post_parameters_changed(self, params: list[rclpy.Parameter]) -> None:
        for param in params:
            if param.name == "kp":
                self.kp = param.get_parameter_value().double_value
            elif param.name == "kd":
                self.kd = param.get_parameter_value().double_value
            elif param.name == "max_steering_angle":
                self.max_steering_angle = param.get_parameter_value().double_value
            elif param.name == "inverse_steering_angle":
                self.inverse_steering_angle = param.get_parameter_value().bool_value

    def _imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.roll = quaternion_to_roll(q.x, q.y, q.z, q.w)
        self.roll_rate = msg.angular_velocity.x

    def _cmd_callback(self, msg: AckermannDrive) -> None:
        steering_angle = self.kp * self.roll + self.kd * self.roll_rate
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        if self.inverse_steering_angle:
            steering_angle = -steering_angle

        out = AckermannDrive()
        out.speed = msg.speed
        out.acceleration = msg.acceleration
        out.jerk = msg.jerk
        out.steering_angle = steering_angle
        out.steering_angle_velocity = msg.steering_angle_velocity
        self.cmd_pub.publish(out)


def _main() -> None:
    rclpy.init()
    node = PDSPipeGuide()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Got Ctrl+C, shutting down.")
        node.destroy_node()


if __name__ == "__main__":
    _main()
