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

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <variant>

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace raph_oak
{

/// The detections of an instance segmentation model with their mask (see to_ros_instance_mask())
struct InstanceResult
{
  vision_msgs::msg::Detection2DArray detections;
  sensor_msgs::msg::Image mask;
};

/// What the network found in a frame. A segmentation mask is a mono8 image, see
/// to_ros_segmentation()
using NnResult = std::variant<
  vision_msgs::msg::Detection2DArray, vision_msgs::msg::Classification, sensor_msgs::msg::Image,
  InstanceResult>;

/**
 * Draws the results of the network on the frame they belong to. The frame and its result arrive
 * on separate queues, in either order. Whichever comes first waits here, matched by sequence
 * number, until the other one arrives. Safe to call from several threads.
 */
class NnOverlay
{
public:
  /// Returns the finished overlay once the result of the same sequence is here, null before
  std::unique_ptr<sensor_msgs::msg::Image> set_image(
    const sensor_msgs::msg::Image & image, std::int64_t sequence);

  /// Returns the finished overlay once the frame of the same sequence is here, null before
  std::unique_ptr<sensor_msgs::msg::Image> set_result(NnResult result, std::int64_t sequence);

private:
  // The mutex has to be held
  std::unique_ptr<sensor_msgs::msg::Image> take_if_ready();

  std::mutex mutex_;
  std::optional<sensor_msgs::msg::Image> image_;
  std::int64_t image_sequence_ = -1;
  std::optional<NnResult> result_;
  std::int64_t result_sequence_ = -1;
};

}  // namespace raph_oak
