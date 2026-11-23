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

#include "raph_oak/demand.hpp"

#include <algorithm>
#include <chrono>

#include "raph_oak/control.hpp"
#include "raph_oak/streams.hpp"

namespace raph_oak
{

ImageDemand::ImageDemand(const ImageSubscriberCounts & subscribers, bool has_compressed)
{
  if (!has_compressed) {
    raw_ = subscribers.image > 0 || subscribers.camera_info > 0;
    return;
  }

  raw_ = subscribers.image > 0;
  // The raw image publishes the camera info while it is requested
  encoded_ = subscribers.compressed > 0 || (subscribers.camera_info > 0 && !raw_);
}

Demand::Demand(const SubscriberCounts & counts)
: pointcloud_(counts.pointcloud > 0), imu_(counts.imu > 0)
{
  for (const auto & spec : ImageStreamSpecs) {
    const auto index = get_image_index(spec.id);
    images_[index] = ImageDemand(counts.images[index], spec.has_compressed());
  }
}

bool Demand::depth_needed() const { return image(ImageStreamId::Depth).raw() || pointcloud_; }

bool Demand::stereo_needed() const
{
  if (depth_needed()) {
    return true;
  }
  return std::any_of(ImageStreamSpecs.begin(), ImageStreamSpecs.end(), [this](const auto & spec) {
    return spec.stereo && image(spec.id).any();
  });
}

GateStates Demand::gate_states() const
{
  GateStates states;
  for (const auto & spec : ImageStreamSpecs) {
    const auto & image_demand = image(spec.id);
    states.set(spec.raw_gate, image_demand.raw());
    if (!spec.enc_gate.is_none()) {
      states.set(spec.enc_gate, image_demand.encoded());
    }
    // The rotation, and the encoder behind it, only run while one of its outputs is wanted
    if (!spec.in_gate.is_none()) {
      states.set(spec.in_gate, image_demand.any());
    }
  }

  // The point cloud is computed from the same rotated depth, so it keeps the depth input open
  states.set(Control::DepthIn, depth_needed());
  states.set(Control::PointCloud, pointcloud_);
  states.set(Control::Imu, imu_);
  return states;
}

StereoSensorAction stereo_sensor_action(
  bool streaming, bool needed, std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point stopped_at, std::chrono::milliseconds restart_delay)
{
  if (needed && !streaming) {
    return now - stopped_at >= restart_delay ? StereoSensorAction::Start : StereoSensorAction::None;
  }
  if (!needed && streaming) {
    return StereoSensorAction::Stop;
  }
  return StereoSensorAction::None;
}

float laser_intensity(bool lazy, bool depth_is_needed, double configured_intensity)
{
  return !lazy || depth_is_needed ? static_cast<float>(configured_intensity) : 0.0F;
}

}  // namespace raph_oak
