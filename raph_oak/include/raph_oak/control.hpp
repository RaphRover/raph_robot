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
#include <utility>

namespace raph_oak
{

/**
 * A target of the single device control channel.
 *
 * All Gate nodes and the stereo cameras are controlled through one XLink input stream: the host
 * sends the target id as a one byte dai::Buffer immediately followed by the actual control message
 * (dai::GateControl or dai::CameraControl), and a Script node forwards the message to the target.
 *
 * Gates come in up to three stages per image, so that nothing is computed or transferred unless it
 * is needed:
 *  - *In gates sit in front of the (rotation) ImageManip, so a closed gate keeps the ImageManip and
 *    everything behind it idle,
 *  - *Raw gates sit in front of the XLinkOut of the raw image,
 *  - *Enc gates sit in front of the JPEG encoder.
 *
 * The targets are written like enum values (Control::RGB_RAW) and converted implicitly, so they can
 * be compared and used in a switch.
 *
 * This header does not depend on DepthAI, so that the logic using it can be unit tested.
 */
class Control
{
public:
  enum Value : uint8_t
  {
    RGB_RAW,
    RGB_ENC,
    RGB_RECT_RAW,
    RGB_RECT_ENC,
    DEPTH_IN,
    DEPTH_RAW,
    POINT_CLOUD,
    LEFT_IN,
    LEFT_RAW,
    LEFT_ENC,
    LEFT_RECT_IN,
    LEFT_RECT_RAW,
    LEFT_RECT_ENC,
    RIGHT_IN,
    RIGHT_RAW,
    RIGHT_ENC,
    RIGHT_RECT_IN,
    RIGHT_RECT_RAW,
    RIGHT_RECT_ENC,
    IMU,
    STILL,
    CAMERA_B,  // left stereo sensor (CameraControl: start/stop streaming)
    CAMERA_C,  // right stereo sensor (CameraControl: start/stop streaming)
    /// No target, where a stream does not have one of its gates
    NONE
  };

  /// Number of targets, NONE excluded
  static constexpr std::size_t kCount = NONE;

  // Implicit on purpose: Control::RGB_RAW is a Control
  constexpr Control(Value value)  // NOLINT(google-explicit-constructor)
  : value_(value)
  {
  }

  // Implicit on purpose: comparisons and switch statements work on the value
  constexpr operator Value() const { return value_; }  // NOLINT(google-explicit-constructor)

  /// All targets in order, NONE excluded
  static constexpr std::array<Control, kCount> all()
  {
    return all(std::make_index_sequence<kCount>{});
  }

  /// Index in arrays of size kCount and the id sent over the control channel
  constexpr std::size_t index() const { return static_cast<std::size_t>(value_); }

  constexpr bool is_none() const { return value_ == NONE; }

  /// Name used in logs
  constexpr const char * name() const
  {
    switch (value_) {
      case RGB_RAW:
        return "rgb";
      case RGB_ENC:
        return "rgb compressed";
      case RGB_RECT_RAW:
        return "rgb_rect";
      case RGB_RECT_ENC:
        return "rgb_rect compressed";
      case DEPTH_IN:
        return "depth input";
      case DEPTH_RAW:
        return "depth";
      case POINT_CLOUD:
        return "pointcloud";
      case LEFT_IN:
        return "left input";
      case LEFT_RAW:
        return "left";
      case LEFT_ENC:
        return "left compressed";
      case LEFT_RECT_IN:
        return "left_rect input";
      case LEFT_RECT_RAW:
        return "left_rect";
      case LEFT_RECT_ENC:
        return "left_rect compressed";
      case RIGHT_IN:
        return "right input";
      case RIGHT_RAW:
        return "right";
      case RIGHT_ENC:
        return "right compressed";
      case RIGHT_RECT_IN:
        return "right_rect input";
      case RIGHT_RECT_RAW:
        return "right_rect";
      case RIGHT_RECT_ENC:
        return "right_rect compressed";
      case IMU:
        return "imu";
      case STILL:
        return "still image";
      case CAMERA_B:
        return "camera B";
      case CAMERA_C:
        return "camera C";
      case NONE:
        break;
    }
    return "none";
  }

  /// The input gates of the image processing are opened and closed too often to log them as info
  constexpr bool is_internal() const
  {
    switch (value_) {
      case DEPTH_IN:
      case LEFT_IN:
      case LEFT_RECT_IN:
      case RIGHT_IN:
      case RIGHT_RECT_IN:
        return true;
      default:
        return false;
    }
  }

  /// A gate that follows the subscribers. The still image gate is opened by the service and the
  /// sensors are started and stopped separately
  constexpr bool follows_demand() const
  {
    return value_ != STILL && value_ != CAMERA_B && value_ != CAMERA_C && value_ != NONE;
  }

private:
  template <std::size_t... Indices>
  static constexpr std::array<Control, kCount> all(std::index_sequence<Indices...> /*indices*/)
  {
    return {Control(static_cast<Value>(Indices))...};
  }

  Value value_;
};

}  // namespace raph_oak
