// Copyright 2026 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "raph_oak/conversions.hpp"
#include "raph_oak/frame_clock.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace raph_oak
{
namespace
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

// ROS time 100 s corresponds to the steady clock epoch
const FrameClock Clock(
  rclcpp::Time(100, 0, RCL_SYSTEM_TIME), std::chrono::steady_clock::time_point{});

constexpr double AccelerationCovariance = 0.001;
constexpr double VelocityCovariance = 0.00001;

template <typename Report>
Report report(int32_t sequence, std::chrono::milliseconds time, float x)
{
  Report result;
  result.sequence = sequence;
  result.timestamp.sec = time.count() / 1000;
  result.timestamp.nsec = (time.count() % 1000) * 1000000;
  result.x = x;
  result.y = 2 * x;
  result.z = -x;
  return result;
}

dai::IMUPacket packet(
  int32_t accelerometer_sequence, std::chrono::milliseconds accelerometer_time, float acceleration,
  int32_t gyroscope_sequence, std::chrono::milliseconds gyroscope_time, float velocity)
{
  dai::IMUPacket result;
  result.acceleroMeter =
    report<dai::IMUReportAccelerometer>(accelerometer_sequence, accelerometer_time, acceleration);
  result.gyroscope = report<dai::IMUReportGyroscope>(gyroscope_sequence, gyroscope_time, velocity);
  return result;
}

dai::IMUData imu_data(std::vector<dai::IMUPacket> packets)
{
  dai::IMUData data;
  data.packets = std::move(packets);
  return data;
}

double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp, RCL_SYSTEM_TIME).seconds();
}

TEST(FrameClock, AddsTheElapsedSteadyTimeToTheRosTime)
{
  const auto time = Clock.to_ros_time(std::chrono::steady_clock::time_point{} + 1500ms);
  EXPECT_DOUBLE_EQ(time.seconds(), 101.5);
}

TEST(ImuConverter, InterpolatesTheGyroscopeAtTheAccelerometerTimes)
{
  ImuConverter converter("imu", AccelerationCovariance, VelocityCovariance);
  const auto messages = converter.convert(
    imu_data(
      {packet(1, 5ms, 9.0F, 1, 0ms, 0.0F), packet(2, 15ms, 9.5F, 2, 10ms, 1.0F),
       packet(2, 15ms, 9.5F, 3, 20ms, 2.0F)}),
    Clock);

  ASSERT_EQ(messages.size(), 2U);
  EXPECT_NEAR(messages[0].angular_velocity.x, 0.5, 1e-9);
  EXPECT_NEAR(messages[0].angular_velocity.y, 1.0, 1e-9);
  EXPECT_NEAR(messages[0].angular_velocity.z, -0.5, 1e-9);
  EXPECT_NEAR(messages[1].angular_velocity.x, 1.5, 1e-9);

  EXPECT_DOUBLE_EQ(messages[0].linear_acceleration.x, 9.0);
  EXPECT_DOUBLE_EQ(messages[1].linear_acceleration.x, 9.5);
  EXPECT_NEAR(seconds(messages[0].header.stamp), 100.005, 1e-9);
  EXPECT_NEAR(seconds(messages[1].header.stamp), 100.015, 1e-9);
}

TEST(ImuConverter, FillsTheHeaderAndCovariances)
{
  ImuConverter converter("oak_imu_frame", AccelerationCovariance, VelocityCovariance);
  const auto messages = converter.convert(
    imu_data({packet(1, 5ms, 1.0F, 1, 0ms, 0.0F), packet(1, 5ms, 1.0F, 2, 10ms, 1.0F)}), Clock);

  ASSERT_EQ(messages.size(), 1U);
  const auto & message = messages[0];
  EXPECT_EQ(message.header.frame_id, "oak_imu_frame");
  EXPECT_DOUBLE_EQ(message.linear_acceleration_covariance[0], AccelerationCovariance);
  EXPECT_DOUBLE_EQ(message.linear_acceleration_covariance[4], AccelerationCovariance);
  EXPECT_DOUBLE_EQ(message.linear_acceleration_covariance[8], AccelerationCovariance);
  EXPECT_DOUBLE_EQ(message.angular_velocity_covariance[0], VelocityCovariance);
  EXPECT_DOUBLE_EQ(message.angular_velocity_covariance[8], VelocityCovariance);
  EXPECT_DOUBLE_EQ(message.orientation_covariance[0], -1.0);
}

TEST(ImuConverter, RepeatedReportsAreUsedOnce)
{
  ImuConverter converter("imu", AccelerationCovariance, VelocityCovariance);
  const auto messages = converter.convert(
    imu_data(
      {packet(1, 5ms, 1.0F, 1, 0ms, 0.0F), packet(1, 5ms, 1.0F, 1, 0ms, 0.0F),
       packet(1, 5ms, 1.0F, 2, 10ms, 1.0F), packet(1, 5ms, 1.0F, 2, 10ms, 1.0F)}),
    Clock);
  EXPECT_EQ(messages.size(), 1U);
}

TEST(ImuConverter, AccelerometerBeforeTheFirstGyroscopeReportIsDropped)
{
  ImuConverter converter("imu", AccelerationCovariance, VelocityCovariance);
  const auto messages = converter.convert(
    imu_data({packet(1, 0ms, 1.0F, 1, 2ms, 0.0F), packet(2, 6ms, 1.0F, 2, 12ms, 1.0F)}), Clock);
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_NEAR(seconds(messages[0].header.stamp), 100.006, 1e-9);
}

TEST(ImuConverter, AccelerometerAfterTheLastGyroscopeReportWaitsForTheNextBatch)
{
  ImuConverter converter("imu", AccelerationCovariance, VelocityCovariance);
  auto messages = converter.convert(
    imu_data({packet(1, 5ms, 1.0F, 1, 0ms, 0.0F), packet(2, 15ms, 1.0F, 2, 10ms, 1.0F)}), Clock);
  ASSERT_EQ(messages.size(), 1U);

  messages = converter.convert(imu_data({packet(2, 15ms, 1.0F, 3, 20ms, 2.0F)}), Clock);
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_NEAR(messages[0].angular_velocity.x, 1.5, 1e-9);
  EXPECT_NEAR(seconds(messages[0].header.stamp), 100.015, 1e-9);
}

TEST(ImuConverter, DoesNotInterpolateOverAGap)
{
  ImuConverter converter("imu", AccelerationCovariance, VelocityCovariance);
  // The gate was closed between 10 ms and 1 s
  const auto messages = converter.convert(
    imu_data(
      {packet(1, 5ms, 1.0F, 1, 0ms, 0.0F), packet(1, 5ms, 1.0F, 2, 10ms, 1.0F),
       packet(2, 500ms, 1.0F, 3, 1000ms, 5.0F), packet(3, 1005ms, 1.0F, 4, 1010ms, 6.0F)}),
    Clock);

  ASSERT_EQ(messages.size(), 2U);
  EXPECT_NEAR(seconds(messages[0].header.stamp), 100.005, 1e-9);
  EXPECT_NEAR(seconds(messages[1].header.stamp), 101.005, 1e-9);
  EXPECT_NEAR(messages[1].angular_velocity.x, 5.5, 1e-9);
}

TEST(PointCloudConverter, CopiesThePointsAsFloats)
{
  dai::PointCloudData data;
  data.setPoints({dai::Point3f(1.0F, 2.0F, 3.0F), dai::Point3f(-1.0F, 0.5F, 4.0F)});
  data.setWidth(2);
  data.setHeight(1);
  data.setTimestamp(std::chrono::steady_clock::time_point{} + 2s);

  const PointCloudConverter converter("oak_stereo_camera_optical_frame");
  const auto message = converter.convert(data, Clock);

  EXPECT_EQ(message.header.frame_id, "oak_stereo_camera_optical_frame");
  EXPECT_DOUBLE_EQ(seconds(message.header.stamp), 102.0);
  EXPECT_EQ(message.width, 2U);
  EXPECT_EQ(message.height, 1U);
  EXPECT_EQ(message.point_step, 12U);
  EXPECT_EQ(message.row_step, 24U);
  ASSERT_EQ(message.fields.size(), 3U);
  EXPECT_EQ(message.fields[0].name, "x");
  EXPECT_EQ(message.fields[2].name, "z");
  EXPECT_EQ(message.fields[2].offset, 8U);
  EXPECT_EQ(message.fields[1].datatype, sensor_msgs::msg::PointField::FLOAT32);

  ASSERT_EQ(message.data.size(), 24U);
  std::array<float, 6> values{};
  std::memcpy(values.data(), message.data.data(), message.data.size());
  EXPECT_EQ(values, (std::array<float, 6>{1.0F, 2.0F, 3.0F, -1.0F, 0.5F, 4.0F}));
}

}  // namespace
}  // namespace raph_oak
