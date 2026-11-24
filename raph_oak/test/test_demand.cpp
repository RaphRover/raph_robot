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
  result.images[get_image_index(id)] = counts;
  return result;
}

bool gate(const GateStates & states, Control control) { return states.is_open(control); }

std::size_t open_gates(const GateStates & states)
{
  std::size_t count = 0;
  for (const auto control : Control::all()) {
    if (control.follows_demand() && states.is_open(control)) {
      ++count;
    }
  }
  return count;
}

TEST(StreamTable, IsIndexedById)
{
  for (std::size_t i = 0; i < kImageStreamCount; ++i) {
    EXPECT_EQ(get_image_index(kImageStreamSpecs[i].id), i);
  }
}

TEST(StreamTable, NamesTopicsAndGatesAreUnique)
{
  std::set<std::string> names;
  std::set<std::string> topics;
  std::set<Control> gates;
  for (const auto & spec : kImageStreamSpecs) {
    EXPECT_TRUE(names.insert(spec.name).second) << spec.name;
    EXPECT_TRUE(topics.insert(spec.topic).second) << spec.topic;
    EXPECT_TRUE(topics.insert(spec.info_topic).second) << spec.info_topic;

    EXPECT_FALSE(spec.raw_gate.is_none()) << spec.name;
    EXPECT_TRUE(gates.insert(spec.raw_gate).second) << spec.name;
    if (!spec.enc_gate.is_none()) {
      EXPECT_TRUE(gates.insert(spec.enc_gate).second) << spec.name;
    }
    if (!spec.in_gate.is_none()) {
      EXPECT_TRUE(gates.insert(spec.in_gate).second) << spec.name;
    }
  }
}

TEST(StreamTable, OnlyThePassthroughNeedsTheNetwork)
{
  for (const auto & spec : kImageStreamSpecs) {
    EXPECT_EQ(spec.needs_nn, spec.id == ImageStreamId::NN_PASSTHROUGH) << spec.name;
  }
}

TEST(StreamTable, OnlyUndistortedStreamsAreRectified)
{
  // A stream that is not rectified takes the camera info of its calibration, so it cannot be
  // undistorted. The passthrough is resized but keeps the distortion
  for (const auto & spec : kImageStreamSpecs) {
    EXPECT_TRUE(!spec.undistorted || spec.rectified) << spec.name;
  }
  EXPECT_TRUE(get_image_stream_spec(ImageStreamId::NN_PASSTHROUGH).rectified);
  EXPECT_FALSE(get_image_stream_spec(ImageStreamId::NN_PASSTHROUGH).undistorted);
}

TEST(StreamTable, OnlyTheDepthHasNoCompressedVariant)
{
  for (const auto & spec : kImageStreamSpecs) {
    EXPECT_EQ(spec.has_compressed(), spec.id != ImageStreamId::DEPTH) << spec.name;
  }
}

TEST(StreamTable, StereoStreamsHaveAnInputGate)
{
  // The stereo outputs are rotated by an ImageManip, which is what the input gate keeps idle
  for (const auto & spec : kImageStreamSpecs) {
    EXPECT_EQ(spec.stereo, !spec.in_gate.is_none()) << spec.name;
  }
}

TEST(Control, EveryTargetHasAName)
{
  for (const auto control : Control::all()) {
    EXPECT_STRNE(control.name(), "none") << control.index();
  }
}

TEST(Control, AllListsEveryTargetInOrder)
{
  const auto all = Control::all();
  for (std::size_t i = 0; i < Control::kCount; ++i) {
    EXPECT_EQ(all[i].index(), i);
  }
  EXPECT_EQ(Control(Control::NONE).index(), Control::kCount);
}

TEST(Control, OnlyTheGatesFollowTheDemand)
{
  EXPECT_TRUE(Control(Control::LEFT_RAW).follows_demand());
  EXPECT_TRUE(Control(Control::POINT_CLOUD).follows_demand());
  EXPECT_FALSE(Control(Control::STILL).follows_demand());
  EXPECT_FALSE(Control(Control::CAMERA_B).follows_demand());
  EXPECT_FALSE(Control(Control::CAMERA_C).follows_demand());
  EXPECT_FALSE(Control(Control::NONE).follows_demand());
}

TEST(ImageDemand, NothingWithoutSubscribers)
{
  const auto demand = ImageDemand({}, true);
  EXPECT_FALSE(demand.raw());
  EXPECT_FALSE(demand.encoded());
}

TEST(ImageDemand, RawImage)
{
  const auto demand = ImageDemand({1, 0, 0}, true);
  EXPECT_TRUE(demand.raw());
  EXPECT_FALSE(demand.encoded());
}

TEST(ImageDemand, CompressedImage)
{
  const auto demand = ImageDemand({0, 2, 0}, true);
  EXPECT_FALSE(demand.raw());
  EXPECT_TRUE(demand.encoded());
}

TEST(ImageDemand, CameraInfoAloneIsServedFromTheCompressedStream)
{
  const auto demand = ImageDemand({0, 0, 1}, true);
  EXPECT_FALSE(demand.raw());
  EXPECT_TRUE(demand.encoded());
}

TEST(ImageDemand, CameraInfoIsServedFromTheRawImageWhenItIsRequested)
{
  const auto demand = ImageDemand({1, 0, 1}, true);
  EXPECT_TRUE(demand.raw());
  EXPECT_FALSE(demand.encoded());
}

TEST(ImageDemand, CameraInfoAndCompressedImage)
{
  const auto demand = ImageDemand({0, 1, 1}, true);
  EXPECT_FALSE(demand.raw());
  EXPECT_TRUE(demand.encoded());
}

TEST(ImageDemand, WithoutCompressedVariantCameraInfoNeedsTheRawImage)
{
  const auto demand = ImageDemand({0, 0, 1}, false);
  EXPECT_TRUE(demand.raw());
  EXPECT_FALSE(demand.encoded());
}

TEST(Demand, NoSubscribersNeedNothing)
{
  const Demand demand;
  EXPECT_FALSE(demand.depth_needed());
  EXPECT_FALSE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_EQ(open_gates(states), 0U);
}

TEST(Demand, RgbDoesNotNeedTheStereoSensors)
{
  const auto demand = Demand(subscribers(ImageStreamId::RGB, {1, 1, 0}));
  EXPECT_FALSE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::RGB_RAW));
  EXPECT_TRUE(gate(states, Control::RGB_ENC));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, RgbRectDoesNotNeedTheStereoSensors)
{
  const auto demand = Demand(subscribers(ImageStreamId::RGB_RECT, {0, 0, 1}));
  EXPECT_FALSE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_FALSE(gate(states, Control::RGB_RECT_RAW));
  EXPECT_TRUE(gate(states, Control::RGB_RECT_ENC));
  EXPECT_EQ(open_gates(states), 1U);
}

TEST(Demand, RawStereoImageOpensItsInputAndRawGate)
{
  const auto demand = Demand(subscribers(ImageStreamId::LEFT_RECT, {1, 0, 0}));
  EXPECT_TRUE(demand.stereo_needed());
  EXPECT_FALSE(demand.depth_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::LEFT_RECT_IN));
  EXPECT_TRUE(gate(states, Control::LEFT_RECT_RAW));
  EXPECT_FALSE(gate(states, Control::LEFT_RECT_ENC));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, CompressedStereoImageOpensItsInputAndEncoderGate)
{
  const auto demand = Demand(subscribers(ImageStreamId::RIGHT, {0, 1, 0}));
  EXPECT_TRUE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::RIGHT_IN));
  EXPECT_FALSE(gate(states, Control::RIGHT_RAW));
  EXPECT_TRUE(gate(states, Control::RIGHT_ENC));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, DepthCameraInfoAloneNeedsTheDepth)
{
  const auto demand = Demand(subscribers(ImageStreamId::DEPTH, {0, 0, 1}));
  EXPECT_TRUE(demand.depth_needed());
  EXPECT_TRUE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::DEPTH_IN));
  EXPECT_TRUE(gate(states, Control::DEPTH_RAW));
  EXPECT_FALSE(gate(states, Control::POINT_CLOUD));
}

TEST(Demand, PointCloudAloneKeepsTheDepthInputOpenButNotTheDepthOutput)
{
  SubscriberCounts counts;
  counts.pointcloud = 1;
  const auto demand = Demand(counts);
  EXPECT_TRUE(demand.depth_needed());
  EXPECT_TRUE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::DEPTH_IN));
  EXPECT_FALSE(gate(states, Control::DEPTH_RAW));
  EXPECT_TRUE(gate(states, Control::POINT_CLOUD));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, ImuDoesNotNeedTheStereoSensors)
{
  SubscriberCounts counts;
  counts.imu = 3;
  const auto demand = Demand(counts);
  EXPECT_FALSE(demand.stereo_needed());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::IMU));
  EXPECT_EQ(open_gates(states), 1U);
}

TEST(Demand, NnDoesNotNeedTheStereoSensors)
{
  SubscriberCounts counts;
  counts.nn = 2;
  const auto demand = Demand(counts);
  EXPECT_FALSE(demand.stereo_needed());
  EXPECT_TRUE(demand.nn());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::NN));
  EXPECT_FALSE(gate(states, Control::NN_ENC));
  EXPECT_EQ(open_gates(states), 1U);
}

TEST(Demand, CompressedNnOpensTheNetworkAndTheEncoder)
{
  SubscriberCounts counts;
  counts.images[get_image_index(ImageStreamId::NN_PASSTHROUGH)].compressed = 1;
  const auto demand = Demand(counts);
  // The compressed passthrough comes out of the network too
  EXPECT_TRUE(demand.nn());
  EXPECT_TRUE(demand.image(ImageStreamId::NN_PASSTHROUGH).encoded());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::NN));
  EXPECT_TRUE(gate(states, Control::NN_ENC));
  EXPECT_EQ(open_gates(states), 2U);
}

TEST(Demand, NnCameraInfoAloneOpensTheNetworkAndTheEncoder)
{
  SubscriberCounts counts;
  counts.images[get_image_index(ImageStreamId::NN_PASSTHROUGH)].camera_info = 1;
  const auto demand = Demand(counts);
  EXPECT_FALSE(demand.image(ImageStreamId::NN_PASSTHROUGH).raw());
  EXPECT_TRUE(demand.image(ImageStreamId::NN_PASSTHROUGH).encoded());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::NN));
  EXPECT_TRUE(gate(states, Control::NN_ENC));
}

TEST(Demand, DetectionsAloneDoNotOpenTheEncoder)
{
  SubscriberCounts counts;
  counts.nn = 1;
  const auto states = Demand(counts).gate_states();
  EXPECT_TRUE(gate(states, Control::NN));
  EXPECT_FALSE(gate(states, Control::NN_ENC));
}

TEST(Demand, RawNnPassthroughNeedsTheNetwork)
{
  SubscriberCounts counts;
  counts.images[get_image_index(ImageStreamId::NN_PASSTHROUGH)].image = 1;
  const auto demand = Demand(counts);
  EXPECT_TRUE(demand.nn());
  EXPECT_FALSE(demand.image(ImageStreamId::NN_PASSTHROUGH).encoded());

  const auto states = demand.gate_states();
  EXPECT_TRUE(gate(states, Control::NN));
  EXPECT_FALSE(gate(states, Control::NN_ENC));
}

TEST(Demand, EverythingSubscribed)
{
  SubscriberCounts counts;
  for (auto & image : counts.images) {
    image = {1, 1, 1};
  }
  counts.pointcloud = 1;
  counts.imu = 1;
  counts.nn = 1;
  const auto states = Demand(counts).gate_states();

  // 7 streams with compressed variant (the passthrough of the network among them, whose gates are
  // the network and its encoder): 2 gates each, the 4 stereo ones also an input gate, plus the
  // depth input and output, the point cloud and the imu
  EXPECT_EQ(open_gates(states), 7U * 2U + 4U + 2U + 1U + 1U);
}

TEST(Demand, TheGatesOfDifferentStreamsAreIndependent)
{
  const auto demand = Demand(subscribers(ImageStreamId::LEFT, {1, 0, 0}));
  const auto states = demand.gate_states();
  EXPECT_FALSE(gate(states, Control::LEFT_RECT_IN));
  EXPECT_FALSE(gate(states, Control::RIGHT_IN));
  EXPECT_FALSE(gate(states, Control::DEPTH_IN));
}

TEST(StereoSensors, NeededWhileStreamingChangesNothing)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, true, now, now, 3000ms), StereoSensorAction::NONE);
}

TEST(StereoSensors, NotNeededWhileStoppedChangesNothing)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(false, false, now, now, 3000ms), StereoSensorAction::NONE);
}

TEST(StereoSensors, StopsWhenNotNeeded)
{
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, false, now, now, 3000ms), StereoSensorAction::STOP);
}

TEST(StereoSensors, DoesNotStartRightAfterTheyWereStopped)
{
  const auto stopped_at = Clock::now();
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 100ms, stopped_at, 3000ms),
    StereoSensorAction::NONE);
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 2999ms, stopped_at, 3000ms),
    StereoSensorAction::NONE);
}

TEST(StereoSensors, StartsOnceTheRestartDelayPassed)
{
  const auto stopped_at = Clock::now();
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 3000ms, stopped_at, 3000ms),
    StereoSensorAction::START);
  EXPECT_EQ(
    stereo_sensor_action(false, true, stopped_at + 10000ms, stopped_at, 3000ms),
    StereoSensorAction::START);
}

TEST(StereoSensors, StopIsNotDelayed)
{
  // Stopping only depends on the demand, a stop shortly after a start is fine
  const auto now = Clock::now();
  EXPECT_EQ(stereo_sensor_action(true, false, now, now - 1ms, 3000ms), StereoSensorAction::STOP);
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

}  // namespace
}  // namespace raph_oak
