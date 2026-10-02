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
#include <cstdint>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
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
#include "raph_oak/control.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"
#include "raph_oak/streams.hpp"
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

void send_control(
  const PipelineDetails & details, Control target, const std::shared_ptr<dai::ADatatype> & message)
{
  if (!details.control_queue || !details.control_mutex) {
    return;
  }

  // The control script reads the target id and then the message from the same stream, so the pair
  // must not be interleaved with another sender
  auto header = std::make_shared<dai::Buffer>();
  header->setData(std::vector<std::uint8_t>{static_cast<std::uint8_t>(target.index())});

  const std::lock_guard<std::mutex> lock(*details.control_mutex);
  details.control_queue->send(header);
  details.control_queue->send(message);
}

// Recurses at most once, to rebuild the pipeline with the neural network disabled
// NOLINTNEXTLINE(misc-no-recursion)
PipelineDetails create_dai_pipeline(
  std::shared_ptr<dai::Device> & device, const Params & params, const rclcpp::Logger & logger)
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
  // Blocking, because a dropped message would misalign the header/message pairs. The default
  // (16 messages, dropping) is overrun by a burst of control commands, e.g. when several topics
  // get subscribed at once.
  details.control_queue = control_script->inputs["control"].createInputQueue(256, true);

  auto control_output = [&](Control target) -> dai::Node::Output & {
    return control_script->outputs["c" + std::to_string(target.index())];
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
  auto make_raw_queue =
    [&](dai::Node::Output & source, Control gate_target, const std::string & name) {
      auto gate = make_gate(source, gate_target);
      auto queue = gate->output.createOutputQueue(1, false);
      queue->setName(name);
      return queue;
    };

  // Compressed output: source -> gate -> MJPEG encoder -> host. The gate sits in front of the
  // encoder, so the encoder is idle and nothing is transferred while it is closed.
  auto make_compressed_queue = [&](
                                 dai::Node::Output & source, Control gate_target, float fps,
                                 int jpeg_quality, const std::string & name) {
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

  // The queues of an image stream: the raw image and, if the stream has one, the JPEG compressed
  // one, both behind the gates of the stream
  auto make_image_queue_set =
    [&](dai::Node::Output & source, ImageStreamId stream_id, float fps, int jpeg_quality) {
      const auto & spec = get_image_stream_spec(stream_id);
      auto & queue_set = details.image_queue_sets[get_image_index(stream_id)];
      queue_set.raw_queue = make_raw_queue(source, spec.raw_gate, spec.name);
      if (spec.has_compressed()) {
        queue_set.compressed_queue = make_compressed_queue(
          source, spec.enc_gate, fps, jpeg_quality, std::string(spec.name) + "_compressed");
      }
    };

  // Rotation of a stereo output by 180 degrees (the camera is mounted upside down). The "in" gate
  // in front of the ImageManip keeps the rotation, and everything behind it, idle while nobody
  // wants the image.
  auto make_rotation = [&](
                         dai::Node::Output & stereo_output, ImageStreamId stream_id, uint32_t width,
                         uint32_t height, dai::ImgFrame::Type type) -> dai::Node::Output & {
    auto rotate = pipeline->create<dai::node::ImageManip>();
    rotate->initialConfig->setOutputSize(width, height);
    rotate->initialConfig->addRotateDeg(180.0);
    rotate->initialConfig->setFrameType(type);
    rotate->setMaxOutputFrameSize(params.mono.width * params.mono.height);
    rotate->inputImage.setBlocking(false);
    make_gate(stereo_output, get_image_stream_spec(stream_id).in_gate)
      ->output.link(rotate->inputImage);
    return rotate->out;
  };

  // A 180 degree rotated mono image coming from a stereo output
  auto make_rotated_mono = [&](dai::Node::Output & stereo_output, ImageStreamId stream_id) {
    auto & rotated = make_rotation(
      stereo_output, stream_id, params.mono.width, params.mono.height, dai::ImgFrame::Type::RAW8);
    make_image_queue_set(rotated, stream_id, params.mono.fps, params.mono_compressed.jpeg_quality);
  };

  // RGB camera node
  auto rgb_node = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
  auto * rgb_output = rgb_node->requestOutput(
    {params.rgb.width, params.rgb.height}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP,
    params.rgb.fps);
  make_image_queue_set(
    *rgb_output, ImageStreamId::Rgb, params.rgb.fps, params.rgb_compressed.jpeg_quality);

  // Undistorted RGB image, computed on the camera from its calibration. It is a second output of
  // the same sensor, so it shares CAM_A's rotation, and has its own raw and encoder gates.
  std::optional<float> rect_alpha_scaling;
  if (params.rgb.rect_alpha_scaling >= 0.0) {
    rect_alpha_scaling = static_cast<float>(params.rgb.rect_alpha_scaling);
  }
  auto * rgb_rect_output = rgb_node->requestOutput(
    {params.rgb.width, params.rgb.height}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP,
    params.rgb.fps, true, rect_alpha_scaling);
  make_image_queue_set(
    *rgb_rect_output, ImageStreamId::RgbRect, params.rgb.fps, params.rgb_compressed.jpeg_quality);

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
  // Robotics preset outputs depth at half the input resolution
  auto & depth_rotated = make_rotation(
    stereo_depth_node->depth, ImageStreamId::Depth, params.mono.width / 2, params.mono.height / 2,
    dai::ImgFrame::Type::RAW16);
  make_image_queue_set(depth_rotated, ImageStreamId::Depth, params.mono.fps, 0);

  auto depth_config_queue = stereo_depth_node->inputConfig.createInputQueue(1, false);

  // Left is the physically right camera and the other way round
  make_rotated_mono(stereo_depth_node->syncedRight, ImageStreamId::Left);
  make_rotated_mono(stereo_depth_node->rectifiedRight, ImageStreamId::LeftRect);
  make_rotated_mono(stereo_depth_node->syncedLeft, ImageStreamId::Right);
  make_rotated_mono(stereo_depth_node->rectifiedLeft, ImageStreamId::RightRect);

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
  make_gate(depth_rotated, Control::PointCloud)->output.link(pointcloud_node->inputDepth);
  details.pointcloud_queue = pointcloud_node->outputPointCloud.createOutputQueue(1, false);
  details.pointcloud_queue->setName("pointcloud");

  // Still image: the full resolution stream is only forwarded to the host when the gate is told
  // to let exactly one frame through (GateControl::openGate(1)). A Script that consumes every
  // frame and forwards one on a trigger costs about 10 percentage points more of the device's
  // media subsystem.
  auto * still_output = rgb_node->requestFullResolutionOutput(
    dai::ImgFrame::Type::NV12, static_cast<float>(params.rgb.fps));
  details.still_image_queue = make_raw_queue(*still_output, Control::Still, "still_image");

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
      auto nn_gate = make_gate(*rgb_output, Control::Nn);

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
  details.depth_config_queue = depth_config_queue;
  details.depth_config = *stereo_depth_node->initialConfig;

  return details;
}
}  // namespace raph_oak
