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

#include "raph_oak/nn_overlay.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <variant>

#include "raph_oak/conversions.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace raph_oak
{
namespace
{
class Drawer
{
public:
  explicit Drawer(sensor_msgs::msg::Image & image) : image_(image) {}

  void operator()(const vision_msgs::msg::Detection2DArray & detections) const
  {
    draw_detections(image_, detections);
  }
  void operator()(const vision_msgs::msg::Classification & classification) const
  {
    draw_classification(image_, classification);
  }
  void operator()(const sensor_msgs::msg::Image & mask) const { draw_segmentation(image_, mask); }
  void operator()(const InstanceResult & instances) const
  {
    draw_instances(image_, instances.detections, instances.mask);
  }

private:
  sensor_msgs::msg::Image & image_;
};
}  // namespace

std::unique_ptr<sensor_msgs::msg::Image> NnOverlay::set_image(
  const sensor_msgs::msg::Image & image, std::int64_t sequence)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  image_ = image;
  image_sequence_ = sequence;
  return take_if_ready();
}

std::unique_ptr<sensor_msgs::msg::Image> NnOverlay::set_result(
  NnResult result, std::int64_t sequence)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  result_ = std::move(result);
  result_sequence_ = sequence;
  return take_if_ready();
}

std::unique_ptr<sensor_msgs::msg::Image> NnOverlay::take_if_ready()
{
  if (!image_ || !result_ || image_sequence_ != result_sequence_) {
    return nullptr;
  }

  auto overlay = std::make_unique<sensor_msgs::msg::Image>(std::move(*image_));
  std::visit(Drawer(*overlay), *result_);
  image_.reset();
  result_.reset();
  return overlay;
}

}  // namespace raph_oak
