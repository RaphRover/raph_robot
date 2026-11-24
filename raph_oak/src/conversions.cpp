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

#include "raph_oak/conversions.hpp"

#include <opencv2/core/hal/interface.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "raph_oak/frame_clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace raph_oak
{

std::unique_ptr<sensor_msgs::msg::Image> to_ros_image(
  dai::ImgFrame & data, const std_msgs::msg::Header & header, const rclcpp::Logger & logger)
{
  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header = header;
  image->width = data.getWidth();
  image->height = data.getHeight();
  image->is_bigendian = 1U;

  if (data.getType() == dai::ImgFrame::Type::NV12) {
    image->encoding = "bgr8";
    image->step = image->width * 3;
    image->data.resize(image->width * image->height * 3);

    cv::Mat const in_mat(
      cv::Size(data.getWidth(), data.getHeight() * 3 / 2), CV_8UC1, data.getData().data());
    cv::Mat out_mat(cv::Size(data.getWidth(), data.getHeight()), CV_8UC3, image->data.data());
    cv::cvtColor(in_mat, out_mat, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
  } else if (
    data.getType() == dai::ImgFrame::Type::BGR888i ||
    data.getType() == dai::ImgFrame::Type::RGB888i) {
    image->encoding = data.getType() == dai::ImgFrame::Type::BGR888i ? "bgr8" : "rgb8";
    image->step = image->width * 3;
    image->data.assign(data.getData().begin(), data.getData().end());
  } else if (
    data.getType() == dai::ImgFrame::Type::BGR888p ||
    data.getType() == dai::ImgFrame::Type::RGB888p) {
    // Planar frames (what RVC2 networks take) need the channels interleaved for ROS
    image->encoding = data.getType() == dai::ImgFrame::Type::BGR888p ? "bgr8" : "rgb8";
    image->step = image->width * 3;
    image->data.resize(static_cast<size_t>(image->width) * image->height * 3);

    const cv::Size plane_size(static_cast<int>(image->width), static_cast<int>(image->height));
    const auto plane_bytes = static_cast<size_t>(image->width) * image->height;
    auto * in_planes = data.getData().data();

    const std::array<cv::Mat, 3> planes{
      cv::Mat(plane_size, CV_8UC1, in_planes),
      cv::Mat(plane_size, CV_8UC1, in_planes + plane_bytes),
      cv::Mat(plane_size, CV_8UC1, in_planes + (2 * plane_bytes))};
    cv::Mat out_mat(plane_size, CV_8UC3, image->data.data());
    cv::merge(planes.data(), planes.size(), out_mat);
  } else if (
    data.getType() == dai::ImgFrame::Type::RAW8 || data.getType() == dai::ImgFrame::Type::GRAY8) {
    image->encoding = "mono8";
    image->step = image->width;
    image->data.assign(data.getData().begin(), data.getData().end());
  } else if (data.getType() == dai::ImgFrame::Type::RAW16) {
    image->encoding = "16UC1";
    image->is_bigendian = 0U;
    image->step = image->width * 2;
    image->data.assign(data.getData().begin(), data.getData().end());
  } else {
    RCLCPP_WARN_STREAM(
      logger, "Unsupported image frame type: " << static_cast<int>(data.getType()));
    return nullptr;
  }
  return image;
}

std::unique_ptr<sensor_msgs::msg::CompressedImage> to_ros_compressed_image(
  dai::EncodedFrame & data, const std_msgs::msg::Header & header)
{
  auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
  image->header = header;
  image->format = "jpeg";
  image->data.assign(data.getData().begin(), data.getData().end());
  return image;
}

void draw_detections(
  sensor_msgs::msg::Image & image, const vision_msgs::msg::Detection2DArray & detections)
{
  if (image.encoding != "bgr8" && image.encoding != "rgb8") {
    return;
  }

  cv::Mat mat(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3, image.data.data(),
    image.step);
  // Green in both channel orders
  const cv::Scalar color(0, 255, 0);
  constexpr double CoordinateLimit = 1000000.0;

  for (const auto & detection : detections.detections) {
    const auto & box = detection.bbox;
    const double center_x = box.center.position.x;
    const double center_y = box.center.position.y;
    if (
      !std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(box.size_x) ||
      !std::isfinite(box.size_y) || std::abs(center_x) > CoordinateLimit ||
      std::abs(center_y) > CoordinateLimit || box.size_x <= 0.0 || box.size_y <= 0.0 ||
      box.size_x > CoordinateLimit || box.size_y > CoordinateLimit) {
      continue;
    }

    const cv::Point top_left(
      static_cast<int>(std::lround(center_x - (box.size_x / 2.0))),
      static_cast<int>(std::lround(center_y - (box.size_y / 2.0))));
    const cv::Point bottom_right(
      static_cast<int>(std::lround(center_x + (box.size_x / 2.0))),
      static_cast<int>(std::lround(center_y + (box.size_y / 2.0))));
    cv::rectangle(mat, top_left, bottom_right, color, 2);

    if (!detection.results.empty()) {
      const auto & hypothesis = detection.results.front().hypothesis;
      std::string label = hypothesis.class_id;
      if (std::isfinite(hypothesis.score)) {
        label +=
          " " + std::to_string(std::lround(std::clamp(hypothesis.score, 0.0, 1.0) * 100.0)) + "%";
      }
      const int label_x = std::clamp(top_left.x, 0, std::max(mat.cols - 1, 0));
      const int label_y = std::clamp(top_left.y - 5, 12, std::max(mat.rows - 1, 12));
      cv::putText(
        mat, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
        cv::LINE_AA);
    }
  }
}

vision_msgs::msg::Detection2DArray to_ros_detections(
  const dai::ImgDetections & data, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;

  // Detections are normalized to the network input frame. This may be a letterboxed version of
  // the RGB stream, so bounding boxes do not line up with that stream without remapping.
  double width = 1.0;
  double height = 1.0;
  if (data.transformation) {
    const auto size = data.transformation->getSize();
    width = static_cast<double>(size.first);
    height = static_cast<double>(size.second);
  }

  msg.detections.reserve(data.detections.size());
  for (const auto & detection : data.detections) {
    vision_msgs::msg::Detection2D ros_detection;
    ros_detection.header = msg.header;
    ros_detection.id =
      detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = ros_detection.id;
    hypothesis.hypothesis.score = detection.confidence;
    ros_detection.results.push_back(hypothesis);

    const auto bbox = detection.getOuterBoundingBox();
    ros_detection.bbox.size_x = (bbox[2] - bbox[0]) * width;
    ros_detection.bbox.size_y = (bbox[3] - bbox[1]) * height;
    ros_detection.bbox.center.position.x = (bbox[0] * width) + (ros_detection.bbox.size_x / 2.0);
    ros_detection.bbox.center.position.y = (bbox[1] * height) + (ros_detection.bbox.size_y / 2.0);
    msg.detections.push_back(ros_detection);
  }
  return msg;
}

ImuConverter::ImuConverter(
  std::string frame_id, double linear_acceleration_covariance, double angular_velocity_covariance)
: frame_id_(std::move(frame_id)),
  linear_acceleration_covariance_(linear_acceleration_covariance),
  angular_velocity_covariance_(angular_velocity_covariance)
{
}

std::vector<sensor_msgs::msg::Imu> ImuConverter::convert(
  const dai::IMUData & data, const FrameClock & clock)
{
  for (const auto & packet : data.packets) {
    if (
      accelerometer_reports_.empty() ||
      accelerometer_reports_.back().sequence != packet.acceleroMeter.sequence) {
      accelerometer_reports_.push_back(packet.acceleroMeter);
    }
    if (
      gyroscope_reports_.empty() ||
      gyroscope_reports_.back().sequence != packet.gyroscope.sequence) {
      gyroscope_reports_.push_back(packet.gyroscope);
    }
  }

  std::vector<sensor_msgs::msg::Imu> messages;
  while (gyroscope_reports_.size() >= 2 && !accelerometer_reports_.empty()) {
    const auto & gyroscope0 = gyroscope_reports_[0];
    const auto & gyroscope1 = gyroscope_reports_[1];
    const auto & accelerometer = accelerometer_reports_.front();

    const auto time0 = gyroscope0.getTimestamp();
    const auto time1 = gyroscope1.getTimestamp();
    const auto time = accelerometer.getTimestamp();

    if (time <= time0) {
      // Older than any gyroscope report that is left: cannot be interpolated
      accelerometer_reports_.pop_front();
    } else if (time > time1 || time1 - time0 > MaxGyroscopeGap) {
      gyroscope_reports_.pop_front();
    } else {
      const double alpha = std::chrono::duration<double>(time - time0).count() /
                           std::chrono::duration<double>(time1 - time0).count();
      messages.push_back(make_message(accelerometer, gyroscope0, gyroscope1, alpha, clock));
      accelerometer_reports_.pop_front();
    }
  }
  return messages;
}

sensor_msgs::msg::Imu ImuConverter::make_message(
  const dai::IMUReportAccelerometer & accelerometer, const dai::IMUReportGyroscope & gyroscope0,
  const dai::IMUReportGyroscope & gyroscope1, double alpha, const FrameClock & clock) const
{
  const auto lerp = [alpha](float value0, float value1) {
    return static_cast<double>(value0) * (1.0 - alpha) + static_cast<double>(value1) * alpha;
  };

  sensor_msgs::msg::Imu message;
  message.header.frame_id = frame_id_;
  message.header.stamp = clock.to_ros_time(accelerometer.getTimestamp());

  message.linear_acceleration.x = accelerometer.x;
  message.linear_acceleration.y = accelerometer.y;
  message.linear_acceleration.z = accelerometer.z;
  message.linear_acceleration_covariance = {linear_acceleration_covariance_, 0.0, 0.0, 0.0,
                                            linear_acceleration_covariance_, 0.0, 0.0, 0.0,
                                            linear_acceleration_covariance_};

  message.angular_velocity.x = lerp(gyroscope0.x, gyroscope1.x);
  message.angular_velocity.y = lerp(gyroscope0.y, gyroscope1.y);
  message.angular_velocity.z = lerp(gyroscope0.z, gyroscope1.z);
  message.angular_velocity_covariance = {angular_velocity_covariance_, 0.0, 0.0, 0.0,
                                         angular_velocity_covariance_, 0.0, 0.0, 0.0,
                                         angular_velocity_covariance_};

  // Orientation unknown
  message.orientation_covariance[0] = -1.0;
  return message;
}

PointCloudConverter::PointCloudConverter(std::string frame_id) : frame_id_(std::move(frame_id)) {}

sensor_msgs::msg::PointCloud2 PointCloudConverter::convert(
  const dai::PointCloudData & data, const FrameClock & clock) const
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id_;
  message.header.stamp = clock.to_ros_time(data.getTimestamp());
  message.width = data.getWidth();
  message.height = data.getHeight();
  // Kept from depthai_bridge: an organized cloud is reported as dense
  message.is_dense = data.isOrganized();
  message.is_bigendian = false;

  constexpr std::uint32_t FieldSize = sizeof(float);
  const std::array<const char *, 3> names{"x", "y", "z"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<std::uint32_t>(i) * FieldSize;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    message.fields.push_back(field);
  }
  message.point_step = 3 * FieldSize;
  message.row_step = message.point_step * message.width;

  const auto points = data.getPoints();
  message.data.resize(points.size() * message.point_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::array<float, 3> xyz{points[i].x, points[i].y, points[i].z};
    std::memcpy(&message.data[i * message.point_step], xyz.data(), message.point_step);
  }
  return message;
}

}  // namespace raph_oak
