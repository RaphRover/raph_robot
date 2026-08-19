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

#include "raph_oak/pipeline.hpp"

#include <memory>

// DepthAI
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"

// ROS
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"

namespace raph_oak
{
PipelineDetails create_dai_pipeline(std::shared_ptr<dai::Device> & device, const Params & params)
{
  PipelineDetails details;
  auto pipeline = std::make_shared<dai::Pipeline>(device);

  // Create nodes
  // RGB camera node
  auto rgb_node = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
  auto rgb_output = rgb_node->requestOutput(
    {params.rgb.width, params.rgb.height}, dai::ImgFrame::Type::NV12,
    dai::ImgResizeMode::CROP, params.rgb.fps);
  auto rgb_queue = rgb_output->createOutputQueue(1, false);
  rgb_queue->setName("rgb");

  // RGB compressed
  auto rgb_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  rgb_encoder_node->setDefaultProfilePreset(
    params.rgb.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  rgb_encoder_node->setQuality(params.rgb_compressed.jpeg_quality);
  rgb_output->link(rgb_encoder_node->input);
  auto rgb_encoder_queue = rgb_encoder_node->out.createOutputQueue(1, false);
  rgb_encoder_queue->setName("rgb_compressed");

  // Create pipeline details
  details.pipeline = pipeline;
  details.rgb_queue = rgb_queue;
  details.rgb_compressed_queue = rgb_encoder_queue;

  return details;
}
}  // namespace raph_oak
