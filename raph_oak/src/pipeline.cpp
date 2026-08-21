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
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/IMU.hpp"

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

  // Mono cameras are mounted upside down. Use CAM_C as logical left and CAM_B as logical right,
  // then rotate by 180 degrees with ImageManip.
  auto left_node = pipeline->create<dai::node::Camera>()->build(
    dai::CameraBoardSocket::CAM_C, {}, params.mono.fps);
  auto right_node = pipeline->create<dai::node::Camera>()->build(
    dai::CameraBoardSocket::CAM_B, {}, params.mono.fps);

  auto left_raw_output = left_node->requestOutput(
    {params.mono.width, params.mono.height}, dai::ImgFrame::Type::RAW8,
    dai::ImgResizeMode::CROP, params.mono.fps);
  auto right_raw_output = right_node->requestOutput(
    {params.mono.width, params.mono.height}, dai::ImgFrame::Type::RAW8,
    dai::ImgResizeMode::CROP, params.mono.fps);

  auto left_rotate = pipeline->create<dai::node::ImageManip>();
  left_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  left_rotate->initialConfig->addRotateDeg(180.0);
  left_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  left_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  left_raw_output->link(left_rotate->inputImage);

  auto right_rotate = pipeline->create<dai::node::ImageManip>();
  right_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  right_rotate->initialConfig->addRotateDeg(180.0);
  right_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  right_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  right_raw_output->link(right_rotate->inputImage);

  auto left_rect_rotate = pipeline->create<dai::node::ImageManip>();
  left_rect_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  left_rect_rotate->initialConfig->addRotateDeg(180.0);
  left_rect_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  left_rect_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  left_raw_output->link(left_rect_rotate->inputImage);

  auto right_rect_rotate = pipeline->create<dai::node::ImageManip>();
  right_rect_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  right_rect_rotate->initialConfig->addRotateDeg(180.0);
  right_rect_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  right_rect_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  right_raw_output->link(right_rect_rotate->inputImage);

  auto left_queue = left_rotate->out.createOutputQueue(1, false);
  left_queue->setName("left");
  auto left_rect_queue = left_rect_rotate->out.createOutputQueue(1, false);
  left_rect_queue->setName("left_rect");
  auto right_queue = right_rotate->out.createOutputQueue(1, false);
  right_queue->setName("right");
  auto right_rect_queue = right_rect_rotate->out.createOutputQueue(1, false);
  right_rect_queue->setName("right_rect");

  auto left_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  left_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  left_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  left_rotate->out.link(left_encoder_node->input);
  auto left_compressed_queue = left_encoder_node->out.createOutputQueue(1, false);
  left_compressed_queue->setName("left_compressed");

  auto left_rect_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  left_rect_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  left_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  left_rect_rotate->out.link(left_rect_encoder_node->input);
  auto left_rect_compressed_queue = left_rect_encoder_node->out.createOutputQueue(1, false);
  left_rect_compressed_queue->setName("left_rect_compressed");

  auto right_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  right_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  right_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  right_rotate->out.link(right_encoder_node->input);
  auto right_compressed_queue = right_encoder_node->out.createOutputQueue(1, false);
  right_compressed_queue->setName("right_compressed");

  auto right_rect_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  right_rect_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  right_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  right_rect_rotate->out.link(right_rect_encoder_node->input);
  auto right_rect_compressed_queue = right_rect_encoder_node->out.createOutputQueue(1, false);
  right_rect_compressed_queue->setName("right_rect_compressed");

  // Imu node
  auto imu_node = pipeline->create<dai::node::IMU>();
  imu_node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
  imu_node->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
  imu_node->setBatchReportThreshold(5);
  imu_node->setMaxBatchReports(20);
  auto imu_queue = imu_node->out.createOutputQueue(1, false);
  imu_queue->setName("imu");

  // Create pipeline details
  details.pipeline = pipeline;
  details.rgb_queue = rgb_queue;
  details.rgb_compressed_queue = rgb_encoder_queue;
  details.left_queue = left_queue;
  details.left_compressed_queue = left_compressed_queue;
  details.left_rect_queue = left_rect_queue;
  details.left_rect_compressed_queue = left_rect_compressed_queue;
  details.right_queue = right_queue;
  details.right_compressed_queue = right_compressed_queue;
  details.right_rect_queue = right_rect_queue;
  details.right_rect_compressed_queue = right_rect_compressed_queue;
  details.imu_queue = imu_queue;

  return details;
}
}  // namespace raph_oak
