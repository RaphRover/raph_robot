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

#include <algorithm>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

// DepthAI
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Config.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
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
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace raph_oak
{
namespace
{
// Loads the model from a local NNArchive file. If no such file exists, the parameter is treated
// as a Model Zoo slug, which is downloaded unless it is already present in the zoo cache
// (see the DEPTHAI_ZOO_CACHE_PATH environment variable).
dai::NNArchive load_nn_archive(const std::string & model, const std::string & platform)
{
  const std::filesystem::path model_path(model);
  if (std::filesystem::exists(model_path)) {
    return dai::NNArchive(model_path);
  }

  dai::NNModelDescription description;
  description.model = model;
  description.platform = platform;
  return dai::NNArchive(dai::getModelFromZoo(description, true));
}

// The frame type the network expects. Few archives declare it outright, so fall back to the
// input layout (most do declare that) and finally to what the platform's inference backend uses.
dai::ImgFrame::Type get_nn_frame_type(const dai::NNArchive & archive, dai::Platform platform)
{
  static const std::map<std::string, dai::ImgFrame::Type> frame_types{
    {"BGR888i", dai::ImgFrame::Type::BGR888i}, {"BGR888p", dai::ImgFrame::Type::BGR888p},
    {"RGB888i", dai::ImgFrame::Type::RGB888i}, {"RGB888p", dai::ImgFrame::Type::RGB888p},
    {"GRAY8", dai::ImgFrame::Type::GRAY8},     {"NV12", dai::ImgFrame::Type::NV12},
  };

  const auto & inputs = archive.getConfig<dai::nn_archive::v1::Config>().model.inputs;
  if (!inputs.empty()) {
    const auto & input = inputs.front();

    if (input.preprocessing.daiType) {
      const auto frame_type = frame_types.find(*input.preprocessing.daiType);
      if (frame_type != frame_types.end()) {
        return frame_type->second;
      }
    }

    // NCHW keeps the channels in separate planes, NHWC interleaves them
    if (input.layout) {
      if (*input.layout == "NCHW") {
        return dai::ImgFrame::Type::BGR888p;
      }
      if (*input.layout == "NHWC") {
        return dai::ImgFrame::Type::BGR888i;
      }
    }
  }

  // RVC2 runs planar inputs, RVC4 interleaved ones
  return platform == dai::Platform::RVC2 ? dai::ImgFrame::Type::BGR888p
                                         : dai::ImgFrame::Type::BGR888i;
}
}  // namespace

// Recurses at most once, to rebuild the pipeline with the neural network disabled
// NOLINTNEXTLINE(misc-no-recursion)
PipelineDetails create_dai_pipeline(
  std::shared_ptr<dai::Device> & device, const Params & params, const rclcpp::Logger & logger)
{
  PipelineDetails details;
  auto pipeline = std::make_shared<dai::Pipeline>(device);
  pipeline->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);

  auto make_gated_output = [&](dai::Node::Output & output, const std::string & name) {
    auto gate = pipeline->create<dai::node::Gate>();
    gate->initialConfig->open = false;
    gate->input.setBlocking(false);
    output.link(gate->input);
    auto queue = gate->output.createOutputQueue(1, false);
    queue->setName(name);
    auto ctrl_queue = gate->inputControl.createInputQueue(4, false);
    return std::make_pair(queue, ctrl_queue);
  };

  // Create nodes
  // RGB camera node
  auto rgb_node = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
  auto * rgb_output = rgb_node->requestOutput(
    {params.rgb.width, params.rgb.height}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP,
    params.rgb.fps);
  auto [rgb_queue, rgb_gate_queue] = make_gated_output(*rgb_output, "rgb");

  // RGB compressed
  auto rgb_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  rgb_encoder_node->setDefaultProfilePreset(
    params.rgb.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  rgb_encoder_node->setQuality(params.rgb_compressed.jpeg_quality);
  rgb_encoder_node->input.setBlocking(false);
  rgb_output->link(rgb_encoder_node->input);
  auto rgb_encoder_queue = rgb_encoder_node->out.createOutputQueue(1, false);
  rgb_encoder_queue->setName("rgb_compressed");

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

  auto depth_rotate = pipeline->create<dai::node::ImageManip>();
  // Robotics preset outputs depth at half the input resolution
  depth_rotate->initialConfig->setOutputSize(params.mono.width / 2, params.mono.height / 2);
  depth_rotate->initialConfig->addRotateDeg(180.0);
  depth_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW16);
  depth_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  depth_rotate->inputImage.setBlocking(false);
  stereo_depth_node->depth.link(depth_rotate->inputImage);

  auto [depth_queue, depth_gate_queue] = make_gated_output(depth_rotate->out, "depth");

  auto depth_config_queue = stereo_depth_node->inputConfig.createInputQueue(1, false);

  // Left camera
  auto left_rotate = pipeline->create<dai::node::ImageManip>();
  left_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  left_rotate->initialConfig->addRotateDeg(180.0);
  left_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  left_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  left_rotate->inputImage.setBlocking(false);
  // Link the right camera output to the left_rotate input (after 180-degree rotation)
  stereo_depth_node->syncedRight.link(left_rotate->inputImage);
  auto [left_queue, left_gate_queue] = make_gated_output(left_rotate->out, "left");

  // Left compressed
  auto left_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  left_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  left_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  left_encoder_node->input.setBlocking(false);
  left_rotate->out.link(left_encoder_node->input);
  auto left_compressed_queue = left_encoder_node->out.createOutputQueue(1, false);
  left_compressed_queue->setName("left_compressed");

  // Left rectified
  auto left_rect_rotate = pipeline->create<dai::node::ImageManip>();
  left_rect_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  left_rect_rotate->initialConfig->addRotateDeg(180.0);
  left_rect_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  left_rect_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  left_rect_rotate->inputImage.setBlocking(false);
  // Link the right camera output to the left_rect_rotate input (after 180-degree rotation)
  stereo_depth_node->rectifiedRight.link(left_rect_rotate->inputImage);
  auto [left_rect_queue, left_rect_gate_queue] =
    make_gated_output(left_rect_rotate->out, "left_rect");

  // Left rectified compressed
  auto left_rect_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  left_rect_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  left_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  left_rect_encoder_node->input.setBlocking(false);
  left_rect_rotate->out.link(left_rect_encoder_node->input);
  auto left_rect_compressed_queue = left_rect_encoder_node->out.createOutputQueue(1, false);
  left_rect_compressed_queue->setName("left_rect_compressed");

  // Right camera
  auto right_rotate = pipeline->create<dai::node::ImageManip>();
  right_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  right_rotate->initialConfig->addRotateDeg(180.0);
  right_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  right_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  right_rotate->inputImage.setBlocking(false);
  // Link the left camera output to the right_rotate input (after 180-degree rotation)
  stereo_depth_node->syncedLeft.link(right_rotate->inputImage);
  auto [right_queue, right_gate_queue] = make_gated_output(right_rotate->out, "right");

  // Right compressed
  auto right_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  right_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  right_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  right_encoder_node->input.setBlocking(false);
  right_rotate->out.link(right_encoder_node->input);
  auto right_compressed_queue = right_encoder_node->out.createOutputQueue(1, false);
  right_compressed_queue->setName("right_compressed");

  // Right rectified
  auto right_rect_rotate = pipeline->create<dai::node::ImageManip>();
  right_rect_rotate->initialConfig->setOutputSize(params.mono.width, params.mono.height);
  right_rect_rotate->initialConfig->addRotateDeg(180.0);
  right_rect_rotate->initialConfig->setFrameType(dai::ImgFrame::Type::RAW8);
  right_rect_rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
  right_rect_rotate->inputImage.setBlocking(false);
  // Link the left camera output to the right_rect_rotate input (after 180-degree rotation)
  stereo_depth_node->rectifiedLeft.link(right_rect_rotate->inputImage);
  auto [right_rect_queue, right_rect_gate_queue] =
    make_gated_output(right_rect_rotate->out, "right_rect");

  // Right rectified compressed
  auto right_rect_encoder_node = pipeline->create<dai::node::VideoEncoder>();
  right_rect_encoder_node->setDefaultProfilePreset(
    params.mono.fps, dai::VideoEncoderProperties::Profile::MJPEG);
  right_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);
  right_rect_encoder_node->input.setBlocking(false);
  right_rect_rotate->out.link(right_rect_encoder_node->input);
  auto right_rect_compressed_queue = right_rect_encoder_node->out.createOutputQueue(1, false);
  right_rect_compressed_queue->setName("right_rect_compressed");

  // Imu node
  auto imu_node = pipeline->create<dai::node::IMU>();
  imu_node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
  imu_node->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
  imu_node->setBatchReportThreshold(5);
  imu_node->setMaxBatchReports(20);
  auto [imu_queue, imu_gate_queue] = make_gated_output(imu_node->out, "imu");

  // Pointcloud node (gating inputDepth so pointcloud calculations only run when subscribed)
  auto pc_depth_gate = pipeline->create<dai::node::Gate>();
  pc_depth_gate->initialConfig->open = false;
  depth_rotate->out.link(pc_depth_gate->input);
  auto pointcloud_gate_queue = pc_depth_gate->inputControl.createInputQueue(4, false);

  auto pointcloud_node = pipeline->create<dai::node::PointCloud>();
  pointcloud_node->initialConfig->setOrganized(true);
  pointcloud_node->initialConfig->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_C);
  pointcloud_node->initialConfig->setLengthUnit(dai::LengthUnit::METER);
  pc_depth_gate->output.link(pointcloud_node->inputDepth);
  auto pointcloud_queue = pointcloud_node->outputPointCloud.createOutputQueue(1, false);
  pointcloud_queue->setName("pointcloud");

  // Still image
  auto * still_output = rgb_node->requestFullResolutionOutput(
    dai::ImgFrame::Type::NV12, static_cast<float>(params.rgb.fps));

  // Current workaround for OAK4 cameras, as Camera node doesn't yet support "still" frame capture:
  // a script node continuously consumes the full resolution stream and only forwards a frame to
  // the host once a trigger message arrives.
  auto still_script_node = pipeline->create<dai::node::Script>();
  still_script_node->inputs["in"].setBlocking(false);
  still_script_node->inputs["in"].setMaxSize(1);
  still_output->link(still_script_node->inputs["in"]);

  still_script_node->setScript(
    R"(
      while True:
          message = node.inputs["in"].get()
          if node.inputs["trigger"].tryGet() is not None:
              node.io["still"].send(message)
  )");

  auto still_image_queue = still_script_node->outputs["still"].createOutputQueue(1, false);
  still_image_queue->setName("still_image");
  auto still_trigger_queue = still_script_node->inputs["trigger"].createInputQueue();

  // Neural network. A failure here rebuilds the pipeline without it, so that a missing or broken
  // model never keeps the camera itself from coming up.
  if (params.nn.enabled) {
    try {
      if (params.nn.model.empty()) {
        throw std::runtime_error("The nn.model parameter is empty");
      }

      const auto archive = load_nn_archive(params.nn.model, device->getPlatformAsString());

      const auto platforms = archive.getSupportedPlatforms();
      if (std::find(platforms.begin(), platforms.end(), device->getPlatform()) == platforms.end()) {
        throw std::runtime_error(
          "The model does not support the " + device->getPlatformAsString() + " platform");
      }

      const auto input_size = archive.getInputSize();
      if (!input_size) {
        throw std::runtime_error("The model does not declare its input size");
      }

      // Gate the RGB stream before resizing, so that neither the resize nor the inference runs
      // while nobody is subscribed. The network is fed from the existing RGB output rather than
      // a separate camera output, which RVC2 devices do not have the resources for. The gate also
      // limits the inference rate, as it is opened with nn.fps (see OakWrapper::check_publishers).
      auto nn_gate = pipeline->create<dai::node::Gate>();
      nn_gate->initialConfig->open = false;
      nn_gate->input.setBlocking(false);
      rgb_output->link(nn_gate->input);
      details.nn_gate_queue = nn_gate->inputControl.createInputQueue(4, false);

      // Letterboxing keeps the full RGB field of view, at the cost of padding the frame
      auto nn_resize = pipeline->create<dai::node::ImageManip>();
      nn_resize->initialConfig->setOutputSize(
        input_size->first, input_size->second, dai::ImageManipConfig::ResizeMode::LETTERBOX);
      nn_resize->initialConfig->setFrameType(get_nn_frame_type(archive, device->getPlatform()));
      nn_resize->setMaxOutputFrameSize(
        static_cast<int>(input_size->first * input_size->second * 3));
      nn_resize->inputImage.setBlocking(false);
      nn_gate->output.link(nn_resize->inputImage);

      auto nn_node =
        pipeline->create<dai::node::DetectionNetwork>()->build(nn_resize->out, archive);

      // RVC2 models ship as a superblob that has to be compiled for a specific number of SHAVE
      // cores. The default does not fit next to the rest of the pipeline, so ask for the
      // configured count instead.
      if (archive.getModelType() == dai::model::ModelType::SUPERBLOB) {
        nn_node->setNNArchive(archive, params.nn.num_shaves);
      }

      nn_node->setConfidenceThreshold(static_cast<float>(params.nn.confidence_threshold));

      details.nn_queue = nn_node->out.createOutputQueue(1, false);
      details.nn_queue->setName("nn");
      details.nn_image_queue = nn_node->passthrough.createOutputQueue(1, false);
      details.nn_image_queue->setName("nn_image");

      RCLCPP_INFO_STREAM(
        logger, "Loaded neural network \"" << params.nn.model << "\" (" << input_size->first << "x"
                                           << input_size->second << " input)");
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger,
        "Failed to set up the neural network: " << e.what() << ". Continuing without detections.");

      // The nodes created before the failure are already linked into the pipeline (and a network
      // that got its model would still claim device resources on start), so start over instead
      // of trying to take them out.
      Params params_without_nn = params;
      params_without_nn.nn.enabled = false;
      return create_dai_pipeline(device, params_without_nn, logger);
    }
  }

  // Create pipeline details
  details.pipeline = pipeline;
  details.rgb_queue = rgb_queue;
  details.rgb_compressed_queue = rgb_encoder_queue;
  details.depth_queue = depth_queue;
  details.left_queue = left_queue;
  details.left_compressed_queue = left_compressed_queue;
  details.left_rect_queue = left_rect_queue;
  details.left_rect_compressed_queue = left_rect_compressed_queue;
  details.right_queue = right_queue;
  details.right_compressed_queue = right_compressed_queue;
  details.right_rect_queue = right_rect_queue;
  details.right_rect_compressed_queue = right_rect_compressed_queue;
  details.imu_queue = imu_queue;
  details.depth_config_queue = depth_config_queue;
  details.depth_config = *stereo_depth_node->initialConfig;
  details.pointcloud_queue = pointcloud_queue;
  details.still_image_queue = still_image_queue;
  details.still_trigger_queue = still_trigger_queue;

  details.rgb_gate_queue = rgb_gate_queue;
  details.depth_gate_queue = depth_gate_queue;
  details.left_gate_queue = left_gate_queue;
  details.left_rect_gate_queue = left_rect_gate_queue;
  details.right_gate_queue = right_gate_queue;
  details.right_rect_gate_queue = right_rect_gate_queue;
  details.imu_gate_queue = imu_gate_queue;
  details.pointcloud_gate_queue = pointcloud_gate_queue;

  return details;
}
}  // namespace raph_oak
