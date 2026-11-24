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
#include <memory>
#include <mutex>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "raph_oak/control.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/streams.hpp"

namespace raph_oak
{

/// The queues of one image stream, see kImageStreamSpecs
struct ImageQueueSet
{
  std::shared_ptr<dai::MessageQueue> raw_queue;
  /// Empty if the stream has no compressed variant
  std::shared_ptr<dai::MessageQueue> compressed_queue;
};

struct PipelineDetails
{
  std::shared_ptr<dai::Pipeline> pipeline;
  std::array<ImageQueueSet, kImageStreamCount> image_queue_sets;
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
