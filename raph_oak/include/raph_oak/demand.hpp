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

#include <array>
#include <chrono>
#include <cstddef>

#include "raph_oak/control.hpp"
#include "raph_oak/streams.hpp"

/**
 * What the camera has to do for the current subscribers.
 *
 * The node counts the subscribers, builds a Demand from them and then applies the decisions made
 * here, which are the part of the node that is easy to get wrong and that can be tested without a
 * camera, a ROS graph or any timing.
 */
namespace raph_oak
{

/// Subscribers of the topics of one image stream
struct ImageSubscriberCounts
{
  std::size_t image{0};
  std::size_t compressed{0};
  std::size_t camera_info{0};
};

struct SubscriberCounts
{
  std::array<ImageSubscriberCounts, kImageStreamCount> images{};
  std::size_t pointcloud{0};
  std::size_t imu{0};
  /// Subscribers of ~/nn/detections and ~/nn/overlay, 0 when nn.enabled is false. The subscribers
  /// of the passthrough topics are counted with the other image streams
  std::size_t nn{0};
};

/// Which gates are open. Only the gates that follow the demand are meaningful, see
/// Control::follows_demand()
class GateStates
{
public:
  bool is_open(Control gate) const { return open_[gate.index()]; }
  void set(Control gate, bool open) { open_[gate.index()] = open; }

private:
  std::array<bool, Control::kCount> open_{};
};

/// What one image stream needs from the camera
class ImageDemand
{
public:
  ImageDemand() = default;

  /// The camera info alone is served from the compressed stream if the stream has one, which is
  /// much cheaper to transfer than the raw image
  ImageDemand(const ImageSubscriberCounts & subscribers, bool has_compressed);

  bool raw() const { return raw_; }
  bool encoded() const { return encoded_; }
  bool any() const { return raw_ || encoded_; }

private:
  bool raw_{false};
  bool encoded_{false};
};

/// What the camera has to do for all the streams
class Demand
{
public:
  Demand() = default;
  explicit Demand(const SubscriberCounts & counts);

  const ImageDemand & image(ImageStreamId stream_id) const
  {
    return images_[get_image_index(stream_id)];
  }
  bool pointcloud() const { return pointcloud_; }
  bool imu() const { return imu_; }
  /// The network has to run: detections, the overlay or either passthrough (both come out of the
  /// network) are wanted
  bool nn() const { return nn_ || image(ImageStreamId::NN_PASSTHROUGH).any(); }

  /// The depth is needed for the depth image and for the point cloud
  bool depth_needed() const;

  /// The stereo sensors are needed for everything derived from them
  bool stereo_needed() const;

  /// The gates that have to be open, see Control::follows_demand()
  GateStates gate_states() const;

private:
  std::array<ImageDemand, kImageStreamCount> images_{};
  bool pointcloud_{false};
  bool imu_{false};
  bool nn_{false};
};

enum class StereoSensorAction
{
  NONE,
  START,
  STOP
};

/**
 * Decide whether the stereo sensors have to be started or stopped.
 *
 * A start right after a stop is lost by the camera and its stereo streams stay silent, so the
 * sensors are only started again restart_delay after they were stopped. Until then the answer is
 * None and the next check asks again.
 */
StereoSensorAction stereo_sensor_action(
  bool streaming, bool needed, std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point stopped_at, std::chrono::milliseconds restart_delay);

/// Intensity of the IR laser dot projector: always the configured one, or with lazy activation only
/// while the depth is needed
float laser_intensity(bool lazy, bool depth_is_needed, double configured_intensity);

}  // namespace raph_oak
