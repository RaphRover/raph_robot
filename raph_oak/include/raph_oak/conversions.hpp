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

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "raph_oak/frame_clock.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/label_info.hpp"

namespace raph_oak
{

std::unique_ptr<sensor_msgs::msg::Image> to_ros_image(
  dai::ImgFrame & data, const std_msgs::msg::Header & header, const rclcpp::Logger & logger);

std::unique_ptr<sensor_msgs::msg::CompressedImage> to_ros_compressed_image(
  dai::EncodedFrame & data, const std_msgs::msg::Header & header);

/// Draws the bounding boxes and labels of the detections on a bgr8 or rgb8 image, in place
void draw_detections(
  sensor_msgs::msg::Image & image, const vision_msgs::msg::Detection2DArray & detections);

vision_msgs::msg::Detection2DArray to_ros_detections(
  const dai::ImgDetections & data, const std_msgs::msg::Header & header);

/// The best max_results classes of a classification result, best first
vision_msgs::msg::Classification to_ros_classification(
  const dai::beta::Classifications & data, const std_msgs::msg::Header & header,
  std::size_t max_results);

/// Draws the best classes on a bgr8 or rgb8 image, in place
void draw_classification(
  sensor_msgs::msg::Image & image, const vision_msgs::msg::Classification & classification);

/// A mono8 image of the class index of every pixel, 255 for the background. Empty if the mask has
/// no data
std::optional<sensor_msgs::msg::Image> to_ros_segmentation(
  const dai::SegmentationMask & data, const std_msgs::msg::Header & header);

/// Blends the classes of a segmentation mask (see to_ros_segmentation) over a bgr8 or rgb8 image,
/// in place. The mask is stretched to the image
void draw_segmentation(sensor_msgs::msg::Image & image, const sensor_msgs::msg::Image & mask);

/// The mask of an instance segmentation model that comes with its detections, as mono8 like
/// to_ros_segmentation(): the value i is the pixels of detection i, 255 is the background. Empty if
/// the detections have no mask
std::optional<sensor_msgs::msg::Image> to_ros_instance_mask(
  const dai::ImgDetections & data, const std_msgs::msg::Header & header);

/// Blends the mask of an instance segmentation model (see to_ros_instance_mask) over a bgr8 or
/// rgb8 image and draws the boxes, in place. The pixels of a detection get a colour of its class
void draw_instances(
  sensor_msgs::msg::Image & image, const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & mask);

/// The names of the classes of a segmentation mask, indexed by the value in the mask
vision_msgs::msg::LabelInfo to_ros_label_info(
  const std::vector<std::string> & labels, const std_msgs::msg::Header & header);

/**
 * Converts the IMU reports of the camera to sensor_msgs/Imu.
 *
 * The accelerometer and the gyroscope report at different rates and times, so every accelerometer
 * report becomes one message, with the angular velocity linearly interpolated between the two
 * gyroscope reports around it and the timestamp of the accelerometer report. Reports arrive in
 * batches and may repeat between batches (same sequence number), which is why the converter keeps
 * the reports it could not use yet. The orientation is marked as unknown.
 */
class ImuConverter
{
public:
  /// Gyroscope reports further apart than this are not interpolated between (the IMU gate was
  /// closed in between, for example)
  static constexpr std::chrono::milliseconds kMaxGyroscopeGap{50};

  ImuConverter(
    std::string frame_id, double linear_acceleration_covariance,
    double angular_velocity_covariance);

  std::vector<sensor_msgs::msg::Imu> convert(const dai::IMUData & data, const FrameClock & clock);

private:
  sensor_msgs::msg::Imu make_message(
    const dai::IMUReportAccelerometer & accelerometer, const dai::IMUReportGyroscope & gyroscope0,
    const dai::IMUReportGyroscope & gyroscope1, double alpha, const FrameClock & clock) const;

  std::string frame_id_;
  double linear_acceleration_covariance_;
  double angular_velocity_covariance_;

  std::deque<dai::IMUReportAccelerometer> accelerometer_reports_;
  std::deque<dai::IMUReportGyroscope> gyroscope_reports_;
};

/**
 * Converts point clouds computed by DepthAI to sensor_msgs/PointCloud2 with x, y and z float
 * fields. The point coordinates are copied as they are, so the PointCloud node has to be set to
 * output metres.
 */
class PointCloudConverter
{
public:
  explicit PointCloudConverter(std::string frame_id);

  sensor_msgs::msg::PointCloud2 convert(
    const dai::PointCloudData & data, const FrameClock & clock) const;

private:
  std::string frame_id_;
};

}  // namespace raph_oak
