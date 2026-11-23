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

#include <chrono>
#include <set>
#include <string>

#include "raph_oak/control.hpp"
#include "raph_oak/demand.hpp"
#include "raph_oak/streams.hpp"

namespace raph_oak
{
namespace
{

using std::chrono_literals::operator""ms;
using Clock = std::chrono::steady_clock;

SubscriberCounts subscribers(ImageStreamId id, ImageSubscriberCounts counts)
{
  SubscriberCounts result;
  result.images[image_index(id)] = counts;
  return result;
}

bool gate(const GateStates & states, Control control) { return states[control_index(control)]; }

std::size_t open_gates(const GateStates & states)
{
  std::size_t count = 0;
  for (std::size_t i = 0; i < ControlCount; ++i) {
    if (is_demand_gate(static_cast<Control>(i)) && states[i]) {
      ++count;
    }
  }
  return count;
}

TEST(StreamTable, IsIndexedById)
{
  for (std::size_t i = 0; i < ImageStreamCount; ++i) {
    EXPECT_EQ(image_index(ImageStreamSpecs[i].id), i);
  }
}

TEST(StreamTable, NamesTopicsAndGatesAreUnique)
{
  std::set<std::string> names;
  std::set<std::string> topics;
  std::set<Control> gates;
  for (const auto & spec : ImageStreamSpecs) {
    EXPECT_TRUE(names.insert(spec.name).second) << spec.name;
    EXPECT_TRUE(topics.insert(spec.topic).second) << spec.topic;
    EXPECT_TRUE(topics.insert(spec.info_topic).second) << spec.info_topic;

    EXPECT_NE(spec.raw_gate, NoControl) << spec.name;
    EXPECT_TRUE(gates.insert(spec.raw_gate).second) << spec.name;
    if (spec.enc_gate != NoControl) {
      EXPECT_TRUE(gates.insert(spec.enc_gate).second) << spec.name;
    }
    if (spec.in_gate != NoControl) {
      EXPECT_TRUE(gates.insert(spec.in_gate).second) << spec.name;
    }
  }
}

TEST(StreamTable, OnlyTheDepthHasNoCompressedVariant)
{
  for (const auto & spec : ImageStreamSpecs) {
    EXPECT_EQ(spec.has_compressed(), spec.id != ImageStreamId::Depth) << spec.name;
  }
}

TEST(StreamTable, StereoStreamsHaveAnInputGate)
{
  // The stereo outputs are rotated by an ImageManip, which is what the input gate keeps idle
  for (const auto & spec : ImageStreamSpecs) {
    EXPECT_EQ(spec.stereo, spec.in_gate != NoControl) << spec.name;
  }
}

TEST(Control, EveryTargetHasAName)
{
  for (std::size_t i = 0; i < ControlCount; ++i) {
    EXPECT_STRNE(control_name(static_cast<Control>(i)), "unknown") << i;
  }
}

TEST(ImageDemand, NothingWithoutSubscribers)
{
  const auto demand = compute_image_demand({}, true);
  EXPECT_FALSE(demand.raw);
  EXPECT_FALSE(demand.encoded);
}

TEST(ImageDemand, RawImage)
{
  const auto demand = compute_image_demand({1, 0, 0}, true);
  EXPECT_TRUE(demand.raw);
  EXPECT_FALSE(demand.encoded);
}

TEST(ImageDemand, CompressedImage)
{
  const auto demand = compute_image_demand({0, 2, 0}, true);
  EXPECT_FALSE(demand.raw);
  EXPECT_TRUE(demand.encoded);
}

TEST(ImageDemand, CameraInfoAloneIsServedFromTheCompressedStream)
{
  const auto demand = compute_image_demand({0, 0, 1}, true);
  EXPECT_FALSE(demand.raw);
  EXPECT_TRUE(demand.encoded);
}

TEST(ImageDemand, CameraInfoIsServedFromTheRawImageWhenItIsRequested)
{
  const auto demand = compute_image_demand({1, 0, 1}, true);
  EXPECT_TRUE(demand.raw);
  EXPECT_FALSE(demand.encoded);
}

TEST(ImageDemand, CameraInfoAndCompressedImage)
{
  const auto demand = compute_image_demand({0, 1, 1}, true);
  EXPECT_FALSE(demand.raw);
  EXPECT_TRUE(demand.encoded);
}

TEST(ImageDemand, WithoutCompressedVariantCameraInfoNeedsTheRawImage)
{
  const auto demand = compute_image_demand({0, 0, 1}, false);
  EXPECT_TRUE(demand.raw);
  EXPECT_FALSE(demand.encoded);
}

TEST(Demand, NoSubscribersNeedNothing)
{
  const auto demand = compute_demand({});
  EXPECT_FALSE(depth_needed(demand));
  EXPECT_FALSE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_EQ(open_gates(states), 0U);
}

TEST(Demand, RgbDoesNotNeedTheStereoSensors)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::Rgb, {1, 1, 0}));
  EXPECT_FALSE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::RgbRaw));
  EXPECT_TRUE(gate(states, Control::RgbEnc));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, RgbRectDoesNotNeedTheStereoSensors)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::RgbRect, {0, 0, 1}));
  EXPECT_FALSE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_FALSE(gate(states, Control::RgbRectRaw));
  EXPECT_TRUE(gate(states, Control::RgbRectEnc));
  EXPECT_EQ(open_gates(states), 1U);
}

TEST(Demand, RawStereoImageOpensItsInputAndRawGate)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::LeftRect, {1, 0, 0}));
  EXPECT_TRUE(stereo_needed(demand));
  EXPECT_FALSE(depth_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::LeftRectIn));
  EXPECT_TRUE(gate(states, Control::LeftRectRaw));
  EXPECT_FALSE(gate(states, Control::LeftRectEnc));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, CompressedStereoImageOpensItsInputAndEncoderGate)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::Right, {0, 1, 0}));
  EXPECT_TRUE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::RightIn));
  EXPECT_FALSE(gate(states, Control::RightRaw));
  EXPECT_TRUE(gate(states, Control::RightEnc));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, DepthCameraInfoAloneNeedsTheDepth)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::Depth, {0, 0, 1}));
  EXPECT_TRUE(depth_needed(demand));
  EXPECT_TRUE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::DepthIn));
  EXPECT_TRUE(gate(states, Control::DepthRaw));
  EXPECT_FALSE(gate(states, Control::PointCloud));
}

TEST(Demand, PointCloudAloneKeepsTheDepthInputOpenButNotTheDepthOutput)
{
  SubscriberCounts counts;
  counts.pointcloud = 1;
  const auto demand = compute_demand(counts);
  EXPECT_TRUE(depth_needed(demand));
  EXPECT_TRUE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::DepthIn));
  EXPECT_FALSE(gate(states, Control::DepthRaw));
  EXPECT_TRUE(gate(states, Control::PointCloud));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, ImuDoesNotNeedTheStereoSensors)
{
  SubscriberCounts counts;
  counts.imu = 3;
  const auto demand = compute_demand(counts);
  EXPECT_FALSE(stereo_needed(demand));

  const auto states = compute_gate_states(demand);
  EXPECT_TRUE(gate(states, Control::Imu));
  EXPECT_EQ(open_gates(states), 1U);
}

TEST(Demand, EverythingSubscribed)
{
  SubscriberCounts counts;
  for (auto & image : counts.images) {
    image = {1, 1, 1};
  }
  counts.pointcloud = 1;
  counts.imu = 1;
  const auto states = compute_gate_states(compute_demand(counts));

  // 6 streams with compressed variant: 2 gates each, the 4 stereo ones also an input gate, plus the
  // depth input and output, the point cloud and the imu
  EXPECT_EQ(open_gates(states), 6U * 2U + 4U + 2U + 1U + 1U);
}

TEST(Demand, TheGatesOfDifferentStreamsAreIndependent)
{
  const auto demand = compute_demand(subscribers(ImageStreamId::Left, {1, 0, 0}));
  const auto states = compute_gate_states(demand);
  EXPECT_FALSE(gate(states, Control::LeftRectIn));
  EXPECT_FALSE(gate(states, Control::RightIn));
  EXPECT_FALSE(gate(states, Control::DepthIn));
}

TEST(StereoSensors, NeededWhileStreamingChangesNothing)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, true, now, now, 3000ms), StereoSensorAction::None);
}

TEST(StereoSensors, NotNeededWhileStoppedChangesNothing)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(false, false, now, now, 3000ms), StereoSensorAction::None);
}

TEST(StereoSensors, StopsWhenNotNeeded)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, false, now, now, 3000ms), StereoSensorAction::Stop);
}

TEST(StereoSensors, DoesNotStartRightAfterTheyWereStopped)
{
  const auto stopped_at = Clock::now();
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 100ms, stopped_at, 3000ms),
    StereoSensorAction::None);
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 2999ms, stopped_at, 3000ms),
    StereoSensorAction::None);
}

TEST(StereoSensors, StartsOnceTheRestartDelayPassed)
{
  const auto stopped_at = Clock::now();
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 3000ms, stopped_at, 3000ms),
    StereoSensorAction::Start);
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 10000ms, stopped_at, 3000ms),
    StereoSensorAction::Start);
}

TEST(StereoSensors, StopIsNotDelayed)
{
  // Stopping only depends on the demand, a stop shortly after a start is fine
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, false, now, now - 1ms, 3000ms), StereoSensorAction::Stop);
}

TEST(Laser, AlwaysOnWithoutLazyActivation)
{
  EXPECT_FLOAT_EQ(laser_intensity(false, false, 0.3), 0.3F);
  EXPECT_FLOAT_EQ(laser_intensity(false, true, 0.3), 0.3F);
}

TEST(Laser, OnlyWhileDepthIsNeededWithLazyActivation)
{
  EXPECT_FLOAT_EQ(laser_intensity(true, false, 0.3), 0.0F);
  EXPECT_FLOAT_EQ(laser_intensity(true, true, 0.3), 0.3F);
}

TEST(PointCloudRate, RoundsToWholeFrames)
{
  EXPECT_EQ(pointcloud_rate(5.0), 5);
  EXPECT_EQ(pointcloud_rate(4.6), 5);
  EXPECT_EQ(pointcloud_rate(4.4), 4);
}

TEST(PointCloudRate, IsAtLeastOne)
{
  EXPECT_EQ(pointcloud_rate(0.2), 1);
  EXPECT_EQ(pointcloud_rate(-3.0), 1);
}

}  // namespace
}  // namespace raph_oak
