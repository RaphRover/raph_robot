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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"

namespace raph_oak
{

/**
 * Targets of the single device control channel.
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
 */
enum class Control : uint8_t
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
  Still,
  CameraB,  // left stereo sensor (CameraControl: start/stop streaming)
  CameraC,  // right stereo sensor (CameraControl: start/stop streaming)
  Count
};

constexpr std::size_t ControlCount = static_cast<std::size_t>(Control::Count);

struct PipelineDetails
{
  std::shared_ptr<dai::Pipeline> pipeline;
  std::shared_ptr<dai::MessageQueue> rgb_queue;
  std::shared_ptr<dai::MessageQueue> rgb_compressed_queue;
  std::shared_ptr<dai::MessageQueue> rgb_rect_queue;
  std::shared_ptr<dai::MessageQueue> rgb_rect_compressed_queue;
  std::shared_ptr<dai::MessageQueue> depth_queue;
  std::shared_ptr<dai::MessageQueue> left_queue;
  std::shared_ptr<dai::MessageQueue> left_compressed_queue;
  std::shared_ptr<dai::MessageQueue> left_rect_queue;
  std::shared_ptr<dai::MessageQueue> left_rect_compressed_queue;
  std::shared_ptr<dai::MessageQueue> right_queue;
  std::shared_ptr<dai::MessageQueue> right_compressed_queue;
  std::shared_ptr<dai::MessageQueue> right_rect_queue;
  std::shared_ptr<dai::MessageQueue> right_rect_compressed_queue;
  std::shared_ptr<dai::MessageQueue> imu_queue;
  std::shared_ptr<dai::InputQueue> depth_config_queue;
  std::shared_ptr<dai::MessageQueue> pointcloud_queue;
  std::shared_ptr<dai::MessageQueue> still_image_queue;

  // Control channel (see Control)
  std::shared_ptr<dai::InputQueue> control_queue;
  std::shared_ptr<std::mutex> control_mutex;

  dai::StereoDepthConfig depth_config;
};

/// Send a dai::GateControl or dai::CameraControl message to the given control target.
void send_control(
  const PipelineDetails & details, Control target, const std::shared_ptr<dai::ADatatype> & message);

PipelineDetails create_dai_pipeline(std::shared_ptr<dai::Device> & device, const Params & params);

}  // namespace raph_oak
