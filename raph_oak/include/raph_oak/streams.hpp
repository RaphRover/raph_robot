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
#include <cstddef>
#include <cstdint>

#include "raph_oak/control.hpp"

namespace raph_oak
{

/**
 * The image streams of the camera. Every one of them is described once, in kImageStreamSpecs, and the
 * pipeline, the publishers, the callbacks and the subscriber based activation are all driven by
 * that table. Adding an image stream means adding an entry here and building its nodes in
 * create_dai_pipeline().
 *
 * This header does not depend on DepthAI, so that the logic using it can be unit tested.
 */
enum class ImageStreamId : uint8_t
{
  RGB,
  RGB_RECT,
  LEFT,
  LEFT_RECT,
  RIGHT,
  RIGHT_RECT,
  DEPTH,
  /// The image the network takes, only present when the network is in the pipeline
  NN_PASSTHROUGH,
  COUNT
};

constexpr std::size_t kImageStreamCount = static_cast<std::size_t>(ImageStreamId::COUNT);

constexpr std::size_t get_image_index(ImageStreamId stream_id)
{
  return static_cast<std::size_t>(stream_id);
}

// Stream specifications are intentionally public aggregates.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
struct ImageStreamSpec
{
  ImageStreamId id;
  /// Name of the stream, also the prefix of its queue names
  const char * name;
  /// Topic of the image, relative to the node. The compressed image is published on
  /// <topic>/compressed
  const char * topic;
  const char * info_topic;
  /// TF frame id, without the tf_frame_prefix parameter
  const char * frame_id;
  /// Gate in front of the ImageManip of the stream, Control::NONE if the stream has no ImageManip
  Control in_gate;
  /// Gate in front of the raw image
  Control raw_gate;
  /// Gate in front of the JPEG encoder, Control::NONE if the stream has no compressed variant
  Control enc_gate;
  /// The stream comes from the stereo sensors, which have to be streaming for it
  bool stereo;
  /// Camera info comes from the intrinsics DepthAI attaches to each frame, not from the factory
  /// calibration
  bool rectified;
  /// The frames were undistorted, so the camera info has no distortion. A resized frame that was
  /// not undistorted keeps the distortion of the calibration. Only used with rectified
  bool undistorted;
  /// The frames were rotated by the sensor, not by an ImageManip: DepthAI does not record that in
  /// the intrinsics of the frame
  bool rotated_at_sensor;
  /// The stream only exists when the neural network made it into the pipeline
  bool needs_nn;

  constexpr bool has_compressed() const { return !enc_gate.is_none(); }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

// Left is the physically right camera and the other way round, because the camera is mounted
// upside down: see create_dai_pipeline()
// clang-format off
inline constexpr std::array<ImageStreamSpec, kImageStreamCount> kImageStreamSpecs{{
  {ImageStreamId::RGB, "rgb", "rgb/image_raw", "rgb/camera_info",
   "oak_rgb_camera_optical_frame", Control::NONE, Control::RGB_RAW, Control::RGB_ENC,
   false, false, false, false, false},
  {ImageStreamId::RGB_RECT, "rgb_rect", "rgb_rect/image_rect", "rgb_rect/camera_info",
   "oak_rgb_camera_optical_frame", Control::NONE, Control::RGB_RECT_RAW, Control::RGB_RECT_ENC,
   false, true, true, true, false},
  {ImageStreamId::LEFT, "left", "left/image_raw", "left/camera_info",
   "oak_left_camera_optical_frame", Control::LEFT_IN, Control::LEFT_RAW, Control::LEFT_ENC,
   true, false, false, false, false},
  {ImageStreamId::LEFT_RECT, "left_rect", "left_rect/image_rect", "left_rect/camera_info",
   "oak_left_camera_optical_frame", Control::LEFT_RECT_IN, Control::LEFT_RECT_RAW,
   Control::LEFT_RECT_ENC, true, true, true, false, false},
  {ImageStreamId::RIGHT, "right", "right/image_raw", "right/camera_info",
   "oak_right_camera_optical_frame", Control::RIGHT_IN, Control::RIGHT_RAW, Control::RIGHT_ENC,
   true, false, false, false, false},
  {ImageStreamId::RIGHT_RECT, "right_rect", "right_rect/image_rect", "right_rect/camera_info",
   "oak_right_camera_optical_frame", Control::RIGHT_RECT_IN, Control::RIGHT_RECT_RAW,
   Control::RIGHT_RECT_ENC, true, true, true, false, false},
  {ImageStreamId::DEPTH, "depth", "stereo/image_raw", "stereo/camera_info",
   "oak_stereo_camera_optical_frame", Control::DEPTH_IN, Control::DEPTH_RAW, Control::NONE,
   true, true, true, false, false},
  {ImageStreamId::NN_PASSTHROUGH, "nn_passthrough", "nn/passthrough",
   "nn/passthrough/camera_info",
   "oak_rgb_camera_optical_frame", Control::NONE, Control::NN, Control::NN_ENC,
   false, true, false, true, true},
}};
// clang-format on

constexpr const ImageStreamSpec & get_image_stream_spec(ImageStreamId stream_id)
{
  return kImageStreamSpecs[get_image_index(stream_id)];
}

}  // namespace raph_oak
