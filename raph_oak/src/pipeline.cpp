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
#include <string>
#include <vector>

// DepthAI
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Gate.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"
#include "depthai/pipeline/node/Script.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/IMUProperties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"

// ROS
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"

namespace raph_oak
{

void PipelineDetails::send_control(
  Control target, const std::shared_ptr<dai::ADatatype> & message) const
{
  if (!control_queue || !control_mutex) {
    return;
  }

  // The control script reads the target id and then the message from the same stream, so the pair
  // must not be interleaved with another sender
  auto header = std::make_shared<dai::Buffer>();
  header->setData(std::vector<std::uint8_t>{static_cast<std::uint8_t>(target)});

  std::lock_guard<std::mutex> lock(*control_mutex);
  control_queue->send(header);
  control_queue->send(message);
}

PipelineDetails create_dai_pipeline(std::shared_ptr<dai::Device> & device, const Params & params)
{
  PipelineDetails details;
  details.control_mutex = std::make_shared<std::mutex>();

  auto pipeline = std::make_shared<dai::Pipeline>(device);
  pipeline->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);

  // Control channel: one XLink input stream for all gates and the stereo cameras. A Script node
  // fans the messages out to the targets, see Control. (Neither the sequence number of the message
  // nor a Script-side dai.GateControl are usable on RVC2, hence the separate header message.)
  auto control_script = pipeline->create<dai::node::Script>();
  control_script->setScript(
    R"(
      while True:
          header = node.inputs["control"].get()
          message = node.inputs["control"].get()
          node.io["c%d" % header.getData()[0]].send(message)
  )");
  details.control_queue = control_script->inputs["control"].createInputQueue();

  auto control_output = [&](Control target) -> dai::Node::Output & {
    return control_script->outputs["c" + std::to_string(static_cast<int>(target))];
  };

  // A gate that starts closed and is opened and closed through the control channel
  auto make_gate = [&](dai::Node::Output & source, Control target) {
    auto gate = pipeline->create<dai::node::Gate>();
    gate->initialConfig->open = false;
    gate->input.setBlocking(false);
    source.link(gate->input);
    control_output(target).link(gate->inputControl);
    return gate;
  };

  // Raw output: source -> gate -> host
  auto make_raw_queue = [&](dai::Node::Output & source, Control gate_target, const char * name) {
    auto gate = make_gate(source, gate_target);
    auto queue = gate->output.createOutputQueue(1, false);
    queue->setName(name);
    return queue;
  };

  // Compressed output: source -> gate -> MJPEG encoder -> host. The gate sits in front of the
  // encoder, so the encoder is idle and nothing is transferred while it is closed.
  auto make_compressed_queue = [&](
                                 dai::Node::Output & source, Control gate_target, float fps,
                                 int jpeg_quality, const char * name) {
    auto gate = make_gate(source, gate_target);
    auto encoder = pipeline->create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(fps, dai::VideoEncoderProperties::Profile::MJPEG);
    encoder->setQuality(jpeg_quality);
    encoder->input.setBlocking(false);
    gate->output.link(encoder->input);
    auto queue = encoder->out.createOutputQueue(1, false);
    queue->setName(name);
    return queue;
  };

  // A 180 degree rotated image coming from a stereo output. The "in" gate in front of the
  // ImageManip keeps the rotation (and the encoder behind it) idle while nobody wants the image.
  auto make_rotated_mono = [&](
                             dai::Node::Output & stereo_output, Control in_gate, Control raw_gate,
                             Control enc_gate, const char * name, const char * compressed_name,
                             std::shared_ptr<dai::MessageQueue> & raw_queue,
                             std::shared_ptr<dai::MessageQueue> & compressed_queue) {
    auto rotate = pipeline->create<dai::node::ImageManip>();
    rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
    rotate->initialConfig->addRotateDeg(180.0);
    rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
    rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
    rotate->inputImage.setBlocking(false);
    make_gate(stereo_output, in_gate)->output.link(rotate->inputImage);

    raw_queue = make_raw_queue(rotate->out, raw_gate, name);
    compressed_queue = make_compressed_queue(
      rotate->out, enc_gate, params.mono.fps, params.mono_compressed.jpeg_quality, compressed_name);
  };

  // RGB camera node
  auto rgb_node = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
  auto * rgb_output = rgb_node->requestOutput(
    {params.rgb.width, params.rgb.height}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP,
    params.rgb.fps);
  details.rgb_queue = make_raw_queue(*rgb_output, Control::RgbRaw, "rgb");
  details.rgb_compressed_queue = make_compressed_queue(
    *rgb_output, Control::RgbEnc, params.rgb.fps, params.rgb_compressed.jpeg_quality,
    "rgb_compressed");

  // Stereo depth
  auto stereo_depth_node = pipeline->create<dai::node::StereoDepth>()->build(
    true, dai::node::StereoDepth::PresetMode::ROBOTICS, {params.mono.width, params.mono.height},
    params.mono.fps);
  stereo_depth_node->setRectifyEdgeFillColor(0);
  stereo_depth_node->setExtendedDisparity(params.depth.extended_disparity_enabled);
  stereo_depth_node->setRuntimeModeSwitch(true);

  // Align to right (which becomes left after 180-degree rotation)
  stereo_depth_node->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT);
  update_depth_config_from_params(*stereo_depth_node->initialConfig, params);

  // The stereo cameras were created by the StereoDepth node, they can be paused while no stereo
  // output is needed
  for (const auto & node : pipeline->getAllNodes()) {
    auto camera = std::dynamic_pointer_cast<dai::node::Camera>(node);
    if (!camera) {
      continue;
    }
    if (camera->getBoardSocket() == dai::CameraBoardSocket::CAM_B) {
      control_output(Control::CameraB).link(camera->inputControl);
    } else if (camera->getBoardSocket() == dai::CameraBoardSocket::CAM_C) {
      control_output(Control::CameraC).link(camera->inputControl);
    }
  }

  // Depth: depth gate -> rotation -> raw gate -> host
  //                                \-> point cloud gate -> point cloud node
  auto depth_rotate = pipeline->create<dai::node::ImageManip>();
  // Robotics preset outputs depth at half the input resolution
  depth_rotate->initialConfig->setOutputSize(params.mono.width / 2, params.mono.height / 2);
  depth_rotate->initialConfig->addRotateDeg(180.0);
  depth_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW16);
  depth_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  depth_rotate->inputImage.setBlocking(false);
  make_gate(stereo_depth_node->depth, Control::DepthIn)->output.link(depth_rotate->inputImage);

  details.depth_queue = make_raw_queue(depth_rotate->out, Control::DepthRaw, "depth");

  auto depth_config_queue = stereo_depth_node->inputConfig.createInputQueue(1, false);

  // Left camera (physically the right one, rotated by 180 degrees)
  make_rotated_mono(
    stereo_depth_node->syncedRight, Control::LeftIn, Control::LeftRaw, Control::LeftEnc, "left",
    "left_compressed", details.left_queue, details.left_compressed_queue);

  // Left rectified
  make_rotated_mono(
    stereo_depth_node->rectifiedRight, Control::LeftRectIn, Control::LeftRectRaw,
    Control::LeftRectEnc, "left_rect", "left_rect_compressed", details.left_rect_queue,
    details.left_rect_compressed_queue);

  // Right camera (physically the left one, rotated by 180 degrees)
  make_rotated_mono(
    stereo_depth_node->syncedLeft, Control::RightIn, Control::RightRaw, Control::RightEnc, "right",
    "right_compressed", details.right_queue, details.right_compressed_queue);

  // Right rectified
  make_rotated_mono(
    stereo_depth_node->rectifiedLeft, Control::RightRectIn, Control::RightRectRaw,
    Control::RightRectEnc, "right_rect", "right_rect_compressed", details.right_rect_queue,
    details.right_rect_compressed_queue);

  // Imu node
  auto imu_node = pipeline->create<dai::node::IMU>();
  imu_node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
  imu_node->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
  imu_node->setBatchReportThreshold(5);
  imu_node->setMaxBatchReports(20);
  details.imu_queue = make_raw_queue(imu_node->out, Control::Imu, "imu");

  // Pointcloud node (gating inputDepth so pointcloud calculations only run when subscribed)
  auto pointcloud_node = pipeline->create<dai::node::PointCloud>();
  pointcloud_node->initialConfig->setOrganized(true);
  pointcloud_node->initialConfig->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_C);
  pointcloud_node->initialConfig->setLengthUnit(dai::LengthUnit::METER);
  make_gate(depth_rotate->out, Control::PointCloud)->output.link(pointcloud_node->inputDepth);
  details.pointcloud_queue = pointcloud_node->outputPointCloud.createOutputQueue(1, false);
  details.pointcloud_queue->setName("pointcloud");

  // Still image: the full resolution stream is only forwarded to the host when the gate is told
  // to let exactly one frame through (GateControl::openGate(1)). A Script that consumes every
  // frame and forwards one on a trigger costs about 10 percentage points more of the device's
  // media subsystem.
  auto * still_output = rgb_node->requestFullResolutionOutput(
    dai::ImgFrame::Type::NV12, static_cast<float>(params.rgb.fps));
  details.still_image_queue = make_raw_queue(*still_output, Control::Still, "still_image");

  // Create pipeline details
  details.pipeline = pipeline;
  details.depth_config_queue = depth_config_queue;
  details.depth_config = *stereo_depth_node->initialConfig;

  return details;
}
}  // namespace raph_oak
