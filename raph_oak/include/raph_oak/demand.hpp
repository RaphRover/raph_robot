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
 * What the camera has to do for the current subscribers, as pure functions.
 *
 * The node counts the subscribers (compute_demand) and then applies the decisions made here, which
 * are the part of the node that is easy to get wrong and that can be tested without a camera, a
 * ROS graph or any timing.
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
  std::array<ImageSubscriberCounts, ImageStreamCount> images{};
  std::size_t pointcloud{0};
  std::size_t imu{0};
};

/// What one image stream needs from the camera
struct ImageDemand
{
  bool raw{false};
  bool encoded{false};
};

struct Demand
{
  std::array<ImageDemand, ImageStreamCount> images{};
  bool pointcloud{false};
  bool imu{false};
};

/// Which gates are open. Only the gates that follow the demand are meaningful, see
/// Control::follows_demand()
class GateStates
{
public:
  bool is_open(Control gate) const { return open_[gate.index()]; }
  void set(Control gate, bool open) { open_[gate.index()] = open; }

private:
  std::array<bool, Control::Count> open_{};
};

enum class StereoSensorAction
{
  None,
  Start,
  Stop
};

/// Demand of one image stream. The camera info alone is served from the compressed stream if the
/// stream has one, which is much cheaper to transfer than the raw image.
ImageDemand compute_image_demand(const ImageSubscriberCounts & subscribers, bool has_compressed);

Demand compute_demand(const SubscriberCounts & counts);

/// The depth is needed for the depth image and for the point cloud
bool depth_needed(const Demand & demand);

/// The stereo sensors are needed for everything derived from them
bool stereo_needed(const Demand & demand);

GateStates compute_gate_states(const Demand & demand);

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

/// Whole frames per second of the point cloud gate, at least 1
int pointcloud_rate(double fps);

}  // namespace raph_oak
