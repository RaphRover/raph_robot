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
 *  - *Enc gates sit in front of the JPEG encoder (for the neural network, in front of the
 *    conversion to NV12 that the encoder needs).
 *
 * The targets are written like enum values (Control::RgbRaw) and converted implicitly, so they can
 * be compared and used in a switch.
 *
 * This header does not depend on DepthAI, so that the logic using it can be unit tested.
 */
class Control
{
public:
  enum Value : uint8_t
  {
    RgbRaw,
    RgbEnc,
    RgbRectRaw,
    RgbRectEnc,
    DepthIn,
    DepthRaw,
    PointCloud,
    LeftIn,
    LeftRaw,
    LeftEnc,
    LeftRectIn,
    LeftRectRaw,
    LeftRectEnc,
    RightIn,
    RightRaw,
    RightEnc,
    RightRectIn,
    RightRectRaw,
    RightRectEnc,
    Imu,
    Nn,
    NnEnc,
    Still,
    CameraB,  // left stereo sensor (CameraControl: start/stop streaming)
    CameraC,  // right stereo sensor (CameraControl: start/stop streaming)
    /// No target, where a stream does not have one of its gates
    None
  };

  /// Number of targets, None excluded
  static constexpr std::size_t Count = None;

  // Implicit on purpose: Control::RgbRaw is a Control
  constexpr Control(Value value)  // NOLINT(google-explicit-constructor)
  : value_(value)
  {
  }

  // Implicit on purpose: comparisons and switch statements work on the value
  constexpr operator Value() const { return value_; }  // NOLINT(google-explicit-constructor)

  /// All targets in order, None excluded
  static constexpr std::array<Control, Count> all()
  {
    return all(std::make_index_sequence<Count>{});
  }

  /// Index in arrays of size Count and the id sent over the control channel
  constexpr std::size_t index() const { return static_cast<std::size_t>(value_); }

  constexpr bool is_none() const { return value_ == None; }

  /// Name used in logs
  constexpr const char * name() const
  {
    switch (value_) {
      case RgbRaw:
        return "rgb";
      case RgbEnc:
        return "rgb compressed";
      case RgbRectRaw:
        return "rgb_rect";
      case RgbRectEnc:
        return "rgb_rect compressed";
      case DepthIn:
        return "depth input";
      case DepthRaw:
        return "depth";
      case PointCloud:
        return "pointcloud";
      case LeftIn:
        return "left input";
      case LeftRaw:
        return "left";
      case LeftEnc:
        return "left compressed";
      case LeftRectIn:
        return "left_rect input";
      case LeftRectRaw:
        return "left_rect";
      case LeftRectEnc:
        return "left_rect compressed";
      case RightIn:
        return "right input";
      case RightRaw:
        return "right";
      case RightEnc:
        return "right compressed";
      case RightRectIn:
        return "right_rect input";
      case RightRectRaw:
        return "right_rect";
      case RightRectEnc:
        return "right_rect compressed";
      case Imu:
        return "imu";
      case Nn:
        return "nn";
      case NnEnc:
        return "nn compressed";
      case Still:
        return "still image";
      case CameraB:
        return "camera B";
      case CameraC:
        return "camera C";
      case None:
        break;
    }
    return "none";
  }

  /// The input gates of the image processing are opened and closed too often to log them as info
  constexpr bool is_internal() const
  {
    switch (value_) {
      case DepthIn:
      case LeftIn:
      case LeftRectIn:
      case RightIn:
      case RightRectIn:
        return true;
      default:
        return false;
    }
  }

  /// A gate that follows the subscribers. The still image gate is opened by the service and the
  /// sensors are started and stopped separately
  constexpr bool follows_demand() const
  {
    return value_ != Still && value_ != CameraB && value_ != CameraC && value_ != None;
  }

private:
  template <std::size_t... Indices>
  static constexpr std::array<Control, Count> all(std::index_sequence<Indices...> /*indices*/)
  {
    return {Control(static_cast<Value>(Indices))...};
  }

  Value value_;
};

}  // namespace raph_oak
