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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "raph_oak/conversions.hpp"
#include "raph_oak/frame_clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

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

TEST(ImageConversion, ConvertsPlanarAndRawFrames)
{
  dai::ImgFrame frame;
  frame.setWidth(2).setHeight(1).setType(dai::ImgFrame::Type::RGB888p);
  frame.setData(std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6});
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;
  auto image = to_ros_image(frame, header, rclcpp::get_logger("test"));
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->header, header);
  EXPECT_EQ(image->encoding, "rgb8");
  EXPECT_EQ(image->step, 6U);
  EXPECT_EQ(image->data, (std::vector<std::uint8_t>{1, 3, 5, 2, 4, 6}));

  frame.setType(dai::ImgFrame::Type::RAW16);
  frame.setData(std::vector<std::uint8_t>{1, 2, 3, 4});
  image = to_ros_image(frame, header, rclcpp::get_logger("test"));
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "16UC1");
  EXPECT_EQ(image->is_bigendian, 0U);
  EXPECT_EQ(image->step, 4U);
  EXPECT_EQ(image->data, (std::vector<std::uint8_t>{1, 2, 3, 4}));
}

TEST(ImageConversion, ConvertsNv12AndRejectsUnsupportedTypes)
{
  dai::ImgFrame frame;
  frame.setWidth(2).setHeight(2).setType(dai::ImgFrame::Type::NV12);
  frame.setData(std::vector<std::uint8_t>{16, 16, 16, 16, 128, 128});
  const std_msgs::msg::Header header;
  auto image = to_ros_image(frame, header, rclcpp::get_logger("test"));
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "bgr8");
  EXPECT_EQ(image->data, std::vector<std::uint8_t>(12, 0));

  frame.setType(dai::ImgFrame::Type::GRAYF16);
  EXPECT_EQ(to_ros_image(frame, header, rclcpp::get_logger("test")), nullptr);
}

TEST(CompressedImageConversion, CopiesHeaderAndEncodedBytes)
{
  dai::EncodedFrame frame;
  frame.setData(std::vector<std::uint8_t>{0xff, 0xd8, 0xff, 0xd9});
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;
  const auto image = to_ros_compressed_image(frame, header);
  EXPECT_EQ(image->header, header);
  EXPECT_EQ(image->format, "jpeg");
  EXPECT_EQ(image->data, (std::vector<std::uint8_t>{0xff, 0xd8, 0xff, 0xd9}));
}

TEST(DetectionConversion, ScalesBoxesAndPreservesLabelsAndHeader)
{
  dai::ImgDetections data;
  data.transformation.emplace().setSize(200, 100);
  dai::ImgDetection named;
  named.labelName = "person";
  named.confidence = 0.75F;
  named.setOuterBoundingBox(0.25F, 0.1F, 0.75F, 0.9F);
  auto numbered = named;
  numbered.labelName.clear();
  numbered.label = 42;
  data.detections = {named, numbered};
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;

  auto message = to_ros_detections(data, header);
  ASSERT_EQ(message.detections.size(), 2U);
  EXPECT_EQ(message.header, header);
  EXPECT_EQ(message.detections[0].header, header);
  EXPECT_EQ(message.detections[0].id, "");
  EXPECT_EQ(message.detections[1].id, "");
  ASSERT_EQ(message.detections[0].results.size(), 1U);
  EXPECT_EQ(message.detections[0].results[0].hypothesis.class_id, "person");
  EXPECT_DOUBLE_EQ(message.detections[0].results[0].hypothesis.score, 0.75);
  EXPECT_NEAR(message.detections[0].bbox.size_x, 100.0, 1e-5);
  EXPECT_NEAR(message.detections[0].bbox.size_y, 80.0, 1e-5);
  EXPECT_NEAR(message.detections[0].bbox.center.position.x, 100.0, 1e-5);
  EXPECT_NEAR(message.detections[0].bbox.center.position.y, 50.0, 1e-5);

  data.transformation.reset();
  message = to_ros_detections(data, header);
  EXPECT_NEAR(message.detections[0].bbox.size_x, 0.5, 1e-5);
  EXPECT_NEAR(message.detections[0].bbox.size_y, 0.8, 1e-5);
}

TEST(ClassificationConversion, KeepsTheBestClassesInOrder)
{
  dai::beta::Classifications data;
  data.classes = {"cat", "dog", "bird"};
  data.scores = {0.7F, 0.2F, 0.1F};
  std_msgs::msg::Header header;
  header.frame_id = "camera";

  const auto message = to_ros_classification(data, header, 2);
  EXPECT_EQ(message.header, header);
  ASSERT_EQ(message.results.size(), 2U);
  EXPECT_EQ(message.results[0].class_id, "cat");
  EXPECT_NEAR(message.results[0].score, 0.7, 1e-6);
  EXPECT_EQ(message.results[1].class_id, "dog");

  EXPECT_EQ(to_ros_classification(data, header, 10).results.size(), 3U);
  data.scores.pop_back();
  EXPECT_EQ(to_ros_classification(data, header, 10).results.size(), 2U);
}

TEST(SegmentationConversion, PublishesTheClassIndicesAsMono8)
{
  const dai::SegmentationMask data(std::vector<std::uint8_t>{0, 1, 255, 2, 255, 0}, 3, 2);
  std_msgs::msg::Header header;
  header.frame_id = "camera";

  const auto message = to_ros_segmentation(data, header);
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->header, header);
  EXPECT_EQ(message->encoding, "mono8");
  EXPECT_EQ(message->width, 3U);
  EXPECT_EQ(message->height, 2U);
  EXPECT_EQ(message->step, 3U);
  EXPECT_EQ(message->data, (std::vector<std::uint8_t>{0, 1, 255, 2, 255, 0}));

  EXPECT_FALSE(to_ros_segmentation(dai::SegmentationMask(), header).has_value());
}

TEST(SegmentationDrawing, BlendsOnlyTheClassesNotTheBackground)
{
  // A 2x1 mask stretched over a 4x2 image: the left half is class 0, the right half background
  sensor_msgs::msg::Image mask;
  mask.width = 2;
  mask.height = 1;
  mask.step = 2;
  mask.encoding = "mono8";
  mask.data = {0, 255};

  sensor_msgs::msg::Image image;
  image.width = 4;
  image.height = 2;
  image.step = 12;
  image.encoding = "bgr8";
  image.data.assign(24, 100);

  draw_segmentation(image, mask);
  // Class 0 has a colour, so the pixel changed. The background pixel is untouched
  EXPECT_NE(image.data[0], 100);
  EXPECT_EQ(image.data[3 * 3], 100);
  EXPECT_EQ(image.data[3 * 3 + 1], 100);
  EXPECT_EQ(image.data[3 * 3 + 2], 100);
  // The colour of one class is the same everywhere
  EXPECT_EQ(image.data[0], image.data[3]);
  EXPECT_EQ(image.data[12], image.data[0]);
}

TEST(SegmentationDrawing, IgnoresAMaskItCannotUse)
{
  sensor_msgs::msg::Image mask;
  mask.width = 2;
  mask.height = 1;
  mask.step = 2;
  mask.encoding = "mono8";
  mask.data = {0};  // too short

  sensor_msgs::msg::Image image;
  image.width = 1;
  image.height = 1;
  image.step = 3;
  image.encoding = "bgr8";
  image.data.assign(3, 100);

  draw_segmentation(image, mask);
  EXPECT_EQ(image.data, (std::vector<std::uint8_t>{100, 100, 100}));
}

TEST(InstanceMaskConversion, KeepsTheDetectionIndicesAsMono8)
{
  dai::ImgDetections data;
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  EXPECT_FALSE(to_ros_instance_mask(data, header).has_value());

  data.setSegmentationMask(std::vector<std::uint8_t>{0, 1, 255, 1, 255, 0}, 3, 2);
  const auto message = to_ros_instance_mask(data, header);
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->header, header);
  EXPECT_EQ(message->encoding, "mono8");
  EXPECT_EQ(message->width, 3U);
  EXPECT_EQ(message->height, 2U);
  EXPECT_EQ(message->step, 3U);
  EXPECT_EQ(message->data, (std::vector<std::uint8_t>{0, 1, 255, 1, 255, 0}));
}

namespace
{
vision_msgs::msg::Detection2D make_detection(const std::string & class_id)
{
  vision_msgs::msg::Detection2D detection;
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = class_id;
  detection.results.push_back(result);
  // Outside the image, so that only the mask is drawn
  detection.bbox.size_x = 0.0;
  return detection;
}
}  // namespace

TEST(InstanceDrawing, ColoursTheInstancesByTheClassOfTheirDetection)
{
  // Detections 0 and 1 are of the same class, 2 of another one. The 4x1 mask has one instance
  // in each pixel and a background pixel at the end
  vision_msgs::msg::Detection2DArray detections;
  detections.detections = {make_detection("cat"), make_detection("cat"), make_detection("dog")};

  sensor_msgs::msg::Image mask;
  mask.width = 4;
  mask.height = 1;
  mask.step = 4;
  mask.encoding = "mono8";
  mask.data = {0, 1, 2, 255};

  sensor_msgs::msg::Image image;
  image.width = 4;
  image.height = 1;
  image.step = 12;
  image.encoding = "bgr8";
  image.data.assign(12, 100);

  draw_instances(image, detections, mask);
  const std::vector<std::uint8_t> first(image.data.begin(), image.data.begin() + 3);
  const std::vector<std::uint8_t> second(image.data.begin() + 3, image.data.begin() + 6);
  const std::vector<std::uint8_t> third(image.data.begin() + 6, image.data.begin() + 9);
  const std::vector<std::uint8_t> background(image.data.begin() + 9, image.data.end());
  EXPECT_EQ(first, second);
  EXPECT_NE(first, third);
  EXPECT_NE(first, (std::vector<std::uint8_t>{100, 100, 100}));
  EXPECT_EQ(background, (std::vector<std::uint8_t>{100, 100, 100}));
}

TEST(InstanceDrawing, AMaskValueWithoutADetectionStillGetsAColour)
{
  const vision_msgs::msg::Detection2DArray detections;

  sensor_msgs::msg::Image mask;
  mask.width = 1;
  mask.height = 1;
  mask.step = 1;
  mask.encoding = "mono8";
  mask.data = {5};

  sensor_msgs::msg::Image image;
  image.width = 1;
  image.height = 1;
  image.step = 3;
  image.encoding = "bgr8";
  image.data.assign(3, 100);

  draw_instances(image, detections, mask);
  EXPECT_NE(image.data, (std::vector<std::uint8_t>{100, 100, 100}));
}

TEST(ClassificationDrawing, WritesTheBestClassOnTheImage)
{
  sensor_msgs::msg::Image image;
  image.width = 64;
  image.height = 32;
  image.step = 192;
  image.encoding = "bgr8";
  image.data.assign(64 * 32 * 3, 0);

  vision_msgs::msg::Classification classification;
  vision_msgs::msg::ObjectHypothesis result;
  result.class_id = "cat";
  result.score = 0.9;
  classification.results.push_back(result);

  draw_classification(image, classification);
  // The text is anti-aliased, so only some pixel is no longer black
  EXPECT_TRUE(
    std::any_of(image.data.begin(), image.data.end(), [](auto value) { return value != 0; }));
}

TEST(LabelInfoConversion, IndexesTheClassesByPosition)
{
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  const auto message = to_ros_label_info({"road", "sky"}, header);
  EXPECT_EQ(message.header, header);
  ASSERT_EQ(message.class_map.size(), 2U);
  EXPECT_EQ(message.class_map[0].class_id, 0);
  EXPECT_EQ(message.class_map[0].class_name, "road");
  EXPECT_EQ(message.class_map[1].class_id, 1);
  EXPECT_EQ(message.class_map[1].class_name, "sky");
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
