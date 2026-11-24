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

#include "raph_oak/oak_wrapper.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// DepthAI
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// XLink
#include "XLink/XLinkPublicDefines.h"

// ROS
#include "raph_oak/camera_info.hpp"
#include "raph_oak/control.hpp"
#include "raph_oak/conversions.hpp"
#include "raph_oak/demand.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"
#include "raph_oak/pipeline.hpp"
#include "raph_oak/streams.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std::chrono_literals;

namespace raph_oak
{

static const std::vector<std::string> kUsbStrings = {"UNKNOWN", "LOW",   "FULL",
                                                     "HIGH",    "SUPER", "SUPER_PLUS"};

// How long the ~/capture_still service waits for the triggered frame to arrive from the device
static constexpr std::chrono::milliseconds kStillCaptureTimeout{2000};

// Minimum time between stopping the stereo sensors and starting them again. A start sent shortly
// after a stop is ignored by the camera and the stereo streams stay silent.
static constexpr std::chrono::milliseconds kStereoRestartDelay{3000};

OakWrapper::OakWrapper(rclcpp::NodeOptions options)
: Node("oak_wrapper", options), param_listener_(get_node_parameters_interface())
{
  update_parameters();
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
    std::bind(&OakWrapper::post_set_parameters_callback, this, std::placeholders::_1));

  this->create_ros_publishers();
  this->create_ros_services();

  imu_converter_.emplace(tf_frame("oak_imu_frame"), 0.001, 0.00001);
  pointcloud_converter_.emplace(tf_frame("oak_stereo_camera_optical_frame"));

  check_timer_ = create_wall_timer(100ms, std::bind(&OakWrapper::check_timer_callback, this));
}

std::string OakWrapper::tf_frame(const std::string & frame) const
{
  return params_.tf_frame_prefix + frame;
}

void OakWrapper::create_ros_publishers()
{
  for (const auto & spec : kImageStreamSpecs) {
    if (spec.needs_nn && !params_.nn.enabled) {
      continue;
    }
    auto & publisher_set = image_publisher_sets_[get_image_index(spec.id)];
    const std::string topic = std::string("~/") + spec.topic;
    publisher_set.image_pub = create_publisher<sensor_msgs::msg::Image>(topic, 1);
    if (spec.has_compressed()) {
      publisher_set.compressed_pub =
        create_publisher<sensor_msgs::msg::CompressedImage>(topic + "/compressed", 1);
    }
    publisher_set.camera_info_pub =
      create_publisher<sensor_msgs::msg::CameraInfo>(std::string("~/") + spec.info_topic, 1);
  }

  // IMU
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", 10);

  // Pointcloud
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 1);

  // Still Image
  const auto still_qos = rclcpp::QoS(1).transient_local();
  still_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/rgb_still/image_still", still_qos);
  still_cam_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb_still/camera_info", still_qos);

  // Neural network
  if (params_.nn.enabled) {
    nn_detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("~/nn/detections", 1);
    nn_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/nn/overlay", 1);
  }
}

void OakWrapper::create_ros_services()
{
  capture_still_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/capture_still",
    std::bind(&OakWrapper::capture_still, this, std::placeholders::_1, std::placeholders::_2));
}

void OakWrapper::fill_camera_info(const dai::CalibrationHandler & calibration_handler)
{
  calibration_handler_ = calibration_handler;

  const auto camera_info = [&](dai::CameraBoardSocket socket, uint32_t width, uint32_t height) {
    return calibration_to_camera_info(calibration_handler, socket, width, height);
  };
  const auto publisher_calibration_info =
    [this](ImageStreamId stream_id) -> sensor_msgs::msg::CameraInfo & {
    return image_publisher_sets_[get_image_index(stream_id)].calibration_info;
  };

  // The camera is mounted upside down, so the images are rotated by 180 degrees and the physically
  // right camera is the left one and the other way round
  const auto left_socket = calibration_handler.getStereoRightCameraId();
  const auto right_socket = calibration_handler.getStereoLeftCameraId();
  const auto mono_width = static_cast<uint32_t>(params_.mono.width);
  const auto mono_height = static_cast<uint32_t>(params_.mono.height);

  publisher_calibration_info(ImageStreamId::RGB) = get_rotated_camera_info(camera_info(
    dai::CameraBoardSocket::CAM_A, static_cast<uint32_t>(params_.rgb.width),
    static_cast<uint32_t>(params_.rgb.height)));
  publisher_calibration_info(ImageStreamId::LEFT) =
    get_rotated_camera_info(camera_info(left_socket, mono_width, mono_height));
  publisher_calibration_info(ImageStreamId::LEFT_RECT) =
    get_rotated_camera_info(camera_info(left_socket, mono_width, mono_height), true);
  publisher_calibration_info(ImageStreamId::RIGHT) =
    get_rotated_camera_info(camera_info(right_socket, mono_width, mono_height));
  publisher_calibration_info(ImageStreamId::RIGHT_RECT) =
    get_rotated_camera_info(camera_info(right_socket, mono_width, mono_height), true);

  // The undistorted RGB image has the pose of the RGB camera
  publisher_calibration_info(ImageStreamId::RGB_RECT) =
    publisher_calibration_info(ImageStreamId::RGB);

  // The network input comes from the RGB camera. The frame is resized and letterboxed but not
  // undistorted, so the distortion of the calibration still applies
  publisher_calibration_info(ImageStreamId::NN_PASSTHROUGH) =
    publisher_calibration_info(ImageStreamId::RGB);

  // Depth: aligned to the rectified right sensor, which is the left image after the rotation, at
  // half the mono resolution. Only a fallback: the camera info of every depth frame is built from
  // the intrinsics DepthAI attaches to it.
  auto & depth_info = publisher_calibration_info(ImageStreamId::DEPTH);
  depth_info = get_scaled_camera_info(
    publisher_calibration_info(ImageStreamId::LEFT_RECT), params_.mono.width / 2,
    params_.mono.height / 2);
  depth_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  depth_info.p[3] = 0.0;

  for (const auto & spec : kImageStreamSpecs) {
    publisher_calibration_info(spec.id).header.frame_id = tf_frame(spec.frame_id);
  }

  // Invalidate the cached still image camera info, it is rebuilt on the next capture
  still_camera_info_ = sensor_msgs::msg::CameraInfo();
}

const sensor_msgs::msg::CameraInfo & OakWrapper::get_still_camera_info(
  uint32_t width, uint32_t height)
{
  if (still_camera_info_.width == width && still_camera_info_.height == height) {
    return still_camera_info_;
  }

  // Same sensor as the RGB stream, but at the full resolution the still frame came in at
  still_camera_info_ = get_rotated_camera_info(calibration_to_camera_info(
    *calibration_handler_, dai::CameraBoardSocket::CAM_A, width, height));
  still_camera_info_.header.frame_id = tf_frame("oak_rgb_camera_optical_frame");

  return still_camera_info_;
}

void OakWrapper::run_pipeline()
{
  Params pipeline_params = params_;
  if (nn_failed_starts_ >= kMaxNnStartAttempts) {
    pipeline_params.nn.enabled = false;
  }

  pipeline_details_ = create_dai_pipeline(device_, pipeline_params, get_logger());
  depth_config_ = pipeline_details_.depth_config;

  // Register permanent callbacks on queues
  for (const auto & spec : kImageStreamSpecs) {
    const auto stream_id = spec.id;
    const auto & queue_set = pipeline_details_.image_queue_sets[get_image_index(stream_id)];
    // The streams of the network only exist when it made it into the pipeline
    if (!queue_set.raw_queue) {
      continue;
    }
    queue_set.raw_queue->addCallback(
      [this, stream_id](const std::shared_ptr<dai::ADatatype> & message) {
        publish_image(stream_id, message);
      });
    if (queue_set.compressed_queue) {
      queue_set.compressed_queue->addCallback(
        [this, stream_id](const std::shared_ptr<dai::ADatatype> & message) {
          publish_compressed_image(stream_id, message);
        });
    }
  }

  pipeline_details_.imu_queue->addCallback(
    [this](const std::shared_ptr<dai::ADatatype> & message) { publish_imu(message); });

  pipeline_details_.pointcloud_queue->addCallback(
    [this](const std::shared_ptr<dai::ADatatype> & message) { publish_pointcloud(message); });

  if (pipeline_details_.nn_queue) {
    pipeline_details_.nn_queue->addCallback(
      [this](const std::shared_ptr<dai::ADatatype> & message) { publish_detections(message); });
  }

  // All gates start closed and the sensors streaming
  reset_control_state();

  // The network queues are only set when the network made it into the pipeline
  const bool with_nn = pipeline_details_.nn_queue != nullptr;

  try {
    pipeline_details_.pipeline->start();
  } catch (const std::exception & e) {
    // Some model problems only surface once the device starts the pipeline. After a few attempts
    // drop the network, so that the camera itself still comes up.
    if (with_nn) {
      ++nn_failed_starts_;
      if (nn_failed_starts_ >= kMaxNnStartAttempts) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Pipeline failed to start with the neural network enabled "
                          << kMaxNnStartAttempts << " times in a row. Retrying without it.");
      } else {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Pipeline failed to start with the neural network enabled (attempt "
                          << nn_failed_starts_ << " of " << kMaxNnStartAttempts << ").");
      }
    }
    throw;
  }

  if (with_nn) {
    nn_failed_starts_ = 0;
  }
}

void OakWrapper::check_timer_callback()
{
  if (!device_) {
    // Connect to device and start the pipeline. A failure at any step leaves a half initialized
    // device and pipeline behind, so everything is released to let the next call start over.
    try {
      device_ = this->connect_to_device();
      this->run_pipeline();

      auto calibration_handler = device_->readCalibration();
      this->fill_camera_info(calibration_handler);

      device_->setIrFloodLightIntensity(params_.device.ir_flood_light_intensity);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to device: %s", e.what());

      // The device may be connected while the pipeline failed to start. Release both, so that the
      // next attempt starts from scratch instead of carrying on with a pipeline that never ran.
      release_device();

      rclcpp::sleep_for(5s);
      return;
    }
  }

  if (device_->isClosed()) {
    RCLCPP_ERROR(get_logger(), "Device disconnected. Freeing resources...");
    release_device();
    return;
  }

  this->check_publishers();
}

void OakWrapper::release_device()
{
  if (pipeline_details_.pipeline) {
    try {
      pipeline_details_.pipeline->stop();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to stop the pipeline: %s", e.what());
    }
  }
  pipeline_details_ = PipelineDetails{};

  device_.reset();

  // A new device starts with the laser off and every gate closed
  laser_intensity_.reset();
  reset_control_state();
}

std::shared_ptr<dai::Device> OakWrapper::connect_to_device()
{
  std::vector<dai::DeviceInfo> available_devices = dai::Device::getAllAvailableDevices();
  if (available_devices.empty()) {
    throw std::runtime_error("No devices detected!");
  }

  std::shared_ptr<dai::Device> device;

  if (params_.device.id.empty() && params_.device.usb_port_id.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "No device.id or device.usb_port_id specified, connecting to the next available "
      "device.");
    device = std::make_shared<dai::Device>(available_devices[0], dai::UsbSpeed::SUPER);
  } else {
    for (const auto & info : available_devices) {
      if (!params_.device.id.empty() && info.getDeviceId() == params_.device.id) {
        RCLCPP_INFO(
          get_logger(), "Connecting to the camera using ID: %s", params_.device.id.c_str());
        if (info.state != X_LINK_BOOTED) {
          device = std::make_shared<dai::Device>(info, dai::UsbSpeed::SUPER);
          break;
        }
        throw std::runtime_error("Device is already booted in different process.");
      }
      if (!params_.device.usb_port_id.empty() && info.name == params_.device.usb_port_id) {
        RCLCPP_INFO(
          get_logger(), "Connecting to the camera using USB ID: %s",
          params_.device.usb_port_id.c_str());
        if (info.state != X_LINK_BOOTED) {
          device = std::make_shared<dai::Device>(info, dai::UsbSpeed::SUPER);
          break;
        }
        throw std::runtime_error("Device is already booted in different process.");
      }
      RCLCPP_INFO(
        get_logger(), "Ignoring device info: ID: %s, USB port id: %s", info.getDeviceId().c_str(),
        info.name.c_str());
    }
  }

  if (!device) {
    throw std::runtime_error("Could not connect to any device.");
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "Connected to device with ID: " << device->getDeviceId() << ", USB port id: "
                                                  << device->getDeviceInfo().name);
  RCLCPP_INFO_STREAM(
    get_logger(), "USB Speed: " << kUsbStrings[static_cast<int32_t>(device->getUsbSpeed())]);

  auto calibration_handler = device->readCalibration();
  auto eeprom = calibration_handler.getEepromData();

  RCLCPP_INFO_STREAM(get_logger(), "Product name: " << eeprom.productName);
  RCLCPP_INFO_STREAM(get_logger(), "Board custom: " << eeprom.boardCustom);
  RCLCPP_INFO_STREAM(get_logger(), "Board name: " << eeprom.boardName);
  RCLCPP_INFO_STREAM(get_logger(), "Board Rev: " << eeprom.boardRev);
  RCLCPP_INFO_STREAM(get_logger(), "Board Conf: " << eeprom.boardConf);
  RCLCPP_INFO_STREAM(get_logger(), "Hardware Conf: " << eeprom.hardwareConf);

  auto sensor_name = device->getCameraSensorNames()[dai::CameraBoardSocket::CAM_A];

  RCLCPP_INFO_STREAM(get_logger(), "Camera sensor name: " << sensor_name);

  return device;
}

void OakWrapper::check_publishers() { apply_demand(Demand(count_subscribers())); }

SubscriberCounts OakWrapper::count_subscribers() const
{
  SubscriberCounts counts;
  for (const auto & spec : kImageStreamSpecs) {
    const auto & publisher_set = image_publisher_sets_[get_image_index(spec.id)];
    // The publishers of the streams of the network only exist when nn.enabled is set
    if (!publisher_set.image_pub) {
      continue;
    }
    counts.images[get_image_index(spec.id)] = {
      publisher_set.image_pub->get_subscription_count(),
      publisher_set.compressed_pub ? publisher_set.compressed_pub->get_subscription_count() : 0,
      publisher_set.camera_info_pub->get_subscription_count()};
  }
  counts.pointcloud = pointcloud_pub_->get_subscription_count();
  counts.imu = imu_pub_->get_subscription_count();
  // The publishers only exist when nn.enabled is set
  if (nn_detections_pub_) {
    counts.nn =
      nn_detections_pub_->get_subscription_count() + nn_overlay_pub_->get_subscription_count();
  }
  return counts;
}

void OakWrapper::apply_demand(const Demand & demand)
{
  const auto wanted_gates = demand.gate_states();
  const bool depth_is_needed = demand.depth_needed();

  // Wake the sensors up before opening anything that depends on them and put them to sleep after
  // everything depending on them was closed
  const auto sensors = stereo_sensor_action(
    stereo_streaming_, demand.stereo_needed(), std::chrono::steady_clock::now(), stereo_stopped_at_,
    kStereoRestartDelay);
  if (sensors == StereoSensorAction::START) {
    set_stereo_streaming(true);
  }

  // The point cloud gate limits the rate of the depth frames fed to the point cloud node. The
  // neural network gates only exist when the network made it into the pipeline (nn.enabled can be
  // true while the model failed to load, see run_pipeline()).
  const int pointcloud_fps = static_cast<int>(params_.pointcloud.max_rate);
  const bool nn_in_pipeline = pipeline_details_.nn_queue != nullptr;
  for (const auto gate : Control::all()) {
    if ((gate == Control::NN || gate == Control::NN_ENC) && !nn_in_pipeline) {
      continue;
    }
    if (gate.follows_demand()) {
      set_gate(
        gate, wanted_gates.is_open(gate), gate == Control::POINT_CLOUD ? pointcloud_fps : -1);
    }
  }

  if (sensors == StereoSensorAction::STOP) {
    set_stereo_streaming(false);
  }

  // The IR laser dot projector follows the parameters at runtime: it is always on, or with the lazy
  // parameter only while depth is needed. The intensity is applied whenever it differs from what
  // the device was last given, so a changed intensity takes effect while the projector is on.
  if (!device_->isClosed()) {
    const auto intensity = laser_intensity(
      params_.device.ir_laser_dot_projector_lazy, depth_is_needed,
      params_.device.ir_laser_dot_projector_intensity);
    if (laser_intensity_ != intensity) {
      device_->setIrLaserDotProjectorIntensity(intensity);
      laser_intensity_ = intensity;
    }
  }
}

void OakWrapper::reset_control_state()
{
  for (auto & state : control_state_) {
    state = false;
  }
  gate_fps_.fill(-1);
  // The stereo sensors stream after the pipeline was started
  stereo_streaming_ = true;
}

void OakWrapper::set_gate(Control gate, bool should_be_open, int fps)
{
  auto & is_open = control_state_[gate.index()];
  auto & sent_fps = gate_fps_[gate.index()];

  // An open gate is opened again to apply a changed rate
  const bool open_changed = should_be_open != is_open;
  const bool fps_changed = should_be_open && fps != sent_fps;
  if (!open_changed && !fps_changed) {
    return;
  }

  std::ostringstream message;
  if (open_changed) {
    message << (should_be_open ? "Opening" : "Closing") << " gate for \"" << gate.name()
            << "\" stream";
  } else {
    message << "Changing the rate of the gate for \"" << gate.name() << "\" stream";
  }
  if (should_be_open && fps > 0) {
    message << " (" << fps << " fps)";
  }
  if (gate.is_internal()) {
    RCLCPP_DEBUG_STREAM(get_logger(), message.str());
  } else {
    RCLCPP_INFO_STREAM(get_logger(), message.str());
  }

  auto gate_control = std::make_shared<dai::GateControl>(should_be_open, -1, fps);
  send_control(pipeline_details_, gate, gate_control);

  is_open = should_be_open;
  sent_fps = should_be_open ? fps : -1;
}

void OakWrapper::set_stereo_streaming(bool streaming)
{
  if (streaming == stereo_streaming_) {
    return;
  }

  RCLCPP_INFO_STREAM(
    get_logger(), (streaming ? "Starting" : "Stopping") << " the stereo camera sensors");

  for (const Control sensor : {Control::CAMERA_B, Control::CAMERA_C}) {
    auto control = std::make_shared<dai::CameraControl>();
    if (streaming) {
      control->setStartStreaming();
    } else {
      control->setStopStreaming();
    }
    send_control(pipeline_details_, sensor, control);
  }
  stereo_streaming_ = streaming;
  if (!streaming) {
    stereo_stopped_at_ = std::chrono::steady_clock::now();
  }
}

void OakWrapper::post_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Parameter " << param.get_name() << " changed to: " << param.value_to_string());
  }

  update_parameters();
  if (device_) {
    send_parameters();
  }
}

void OakWrapper::update_parameters()
{
  param_listener_.refresh_dynamic_parameters();
  params_ = param_listener_.get_params();

  update_depth_config_from_params(depth_config_, params_);
}

void OakWrapper::send_parameters() const
{
  if (pipeline_details_.depth_config_queue) {
    pipeline_details_.depth_config_queue->send(
      std::make_shared<dai::StereoDepthConfig>(depth_config_));
  }

  // The IR laser dot projector is updated by the periodic check, see check_publishers()
  device_->setIrFloodLightIntensity(params_.device.ir_flood_light_intensity);
}

void OakWrapper::publish_image(
  ImageStreamId stream_id, const std::shared_ptr<dai::ADatatype> & message)
{
  const auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not an image frame");
    return;
  }

  const auto & spec = get_image_stream_spec(stream_id);
  const auto & publisher_set = image_publisher_sets_[get_image_index(stream_id)];

  // The raw gate of a stream with a compressed variant is only open for the raw image, except for
  // the passthrough, which comes out of the network that also runs for the detections and the
  // overlay. The overlay is drawn on the same image
  const bool overlay_wanted =
    stream_id == ImageStreamId::NN_PASSTHROUGH && nn_overlay_pub_->get_subscription_count() > 0;
  if (
    spec.has_compressed() && publisher_set.image_pub->get_subscription_count() == 0 &&
    !overlay_wanted) {
    return;
  }

  // The intrinsics of a rectified or undistorted image are only known to DepthAI, so the camera
  // info is built from the frame
  auto cam_info = spec.rectified
                    ? get_rectified_camera_info(
                        in_data->getTransformation(), in_data->getWidth(), in_data->getHeight(),
                        publisher_set.calibration_info, spec.rotated_at_sensor, spec.undistorted)
                    : publisher_set.calibration_info;
  cam_info.header.stamp = frame_clock_.to_ros_time(in_data->getTimestamp());

  auto image = to_ros_image(*in_data, cam_info.header, get_logger());
  if (!image) {
    return;
  }

  if (overlay_wanted) {
    set_nn_overlay_image(*image, in_data->getSequenceNum());
  }

  publisher_set.camera_info_pub->publish(cam_info);
  publisher_set.image_pub->publish(std::move(image));
}

void OakWrapper::publish_compressed_image(
  ImageStreamId stream_id, const std::shared_ptr<dai::ADatatype> & message)
{
  const auto in_data = std::dynamic_pointer_cast<dai::EncodedFrame>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not an encoded frame");
    return;
  }

  const auto & spec = get_image_stream_spec(stream_id);
  const auto & publisher_set = image_publisher_sets_[get_image_index(stream_id)];
  const auto stamp = frame_clock_.to_ros_time(in_data->getTimestamp());

  // The raw image publishes the camera info while it is subscribed. Otherwise the compressed
  // stream (which is also opened for camera info subscribers) takes over.
  if (
    publisher_set.image_pub->get_subscription_count() == 0 &&
    publisher_set.camera_info_pub->get_subscription_count() > 0) {
    auto cam_info = spec.rectified
                      ? get_rectified_camera_info(
                          in_data->transformation, in_data->getWidth(), in_data->getHeight(),
                          publisher_set.calibration_info, spec.rotated_at_sensor, spec.undistorted)
                      : publisher_set.calibration_info;
    cam_info.header.stamp = stamp;
    publisher_set.camera_info_pub->publish(cam_info);
  }

  if (publisher_set.compressed_pub->get_subscription_count() == 0) {
    return;
  }

  auto header = publisher_set.calibration_info.header;
  header.stamp = stamp;
  publisher_set.compressed_pub->publish(to_ros_compressed_image(*in_data, header));
}

void OakWrapper::publish_imu(const std::shared_ptr<dai::ADatatype> & message)
{
  auto in_data = std::dynamic_pointer_cast<dai::IMUData>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not IMU data");
    return;
  }

  for (const auto & imu : imu_converter_->convert(*in_data, frame_clock_)) {
    imu_pub_->publish(imu);
  }
}

void OakWrapper::publish_pointcloud(const std::shared_ptr<dai::ADatatype> & message)
{
  auto in_data = std::dynamic_pointer_cast<dai::PointCloudData>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not point cloud data");
    return;
  }

  pointcloud_pub_->publish(pointcloud_converter_->convert(*in_data, frame_clock_));
}

void OakWrapper::publish_detections(const std::shared_ptr<dai::ADatatype> & message)
{
  const auto in_data = std::dynamic_pointer_cast<dai::ImgDetections>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not a set of detections");
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = tf_frame("oak_rgb_camera_optical_frame");
  header.stamp = frame_clock_.to_ros_time(in_data->getTimestamp());
  auto detections = to_ros_detections(*in_data, header);

  if (nn_detections_pub_->get_subscription_count() > 0) {
    nn_detections_pub_->publish(detections);
  }

  if (nn_overlay_pub_->get_subscription_count() > 0) {
    const std::lock_guard<std::mutex> lock(nn_overlay_mutex_);
    nn_overlay_detections_ = std::move(detections);
    nn_overlay_detections_sequence_ = in_data->getSequenceNum();
    publish_nn_overlay_if_ready();
  }
}

void OakWrapper::set_nn_overlay_image(const sensor_msgs::msg::Image & image, std::int64_t sequence)
{
  const std::lock_guard<std::mutex> lock(nn_overlay_mutex_);
  nn_overlay_image_ = image;
  nn_overlay_image_sequence_ = sequence;
  publish_nn_overlay_if_ready();
}

void OakWrapper::publish_nn_overlay_if_ready()
{
  if (
    !nn_overlay_image_ || !nn_overlay_detections_ ||
    nn_overlay_image_sequence_ != nn_overlay_detections_sequence_) {
    return;
  }

  auto overlay = std::make_unique<sensor_msgs::msg::Image>(std::move(*nn_overlay_image_));
  draw_detections(*overlay, *nn_overlay_detections_);
  nn_overlay_image_.reset();
  nn_overlay_detections_.reset();
  nn_overlay_pub_->publish(std::move(overlay));
}

void OakWrapper::capture_still(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (
    !device_ || device_->isClosed() || !pipeline_details_.control_queue ||
    !pipeline_details_.still_image_queue || !calibration_handler_) {
    response->success = false;
    response->message = "Device is not connected";
    RCLCPP_WARN_STREAM(get_logger(), "Still image capture failed: " << response->message);
    return;
  }

  // Drop anything left over from a previous, timed out capture
  pipeline_details_.still_image_queue->tryGetAll();
  send_control(pipeline_details_, Control::STILL, dai::GateControl::openGate(1));

  bool timed_out = false;
  auto in_data =
    pipeline_details_.still_image_queue->get<dai::ImgFrame>(kStillCaptureTimeout, timed_out);
  if (timed_out || !in_data) {
    response->success = false;
    response->message = "Timed out waiting for the still image frame";
    RCLCPP_WARN_STREAM(get_logger(), "Still image capture failed: " << response->message);
    return;
  }

  sensor_msgs::msg::CameraInfo cam_info =
    get_still_camera_info(in_data->getWidth(), in_data->getHeight());
  cam_info.header.stamp = frame_clock_.to_ros_time(in_data->getTimestamp());

  auto image = to_ros_image(*in_data, cam_info.header, get_logger());
  if (!image) {
    response->success = false;
    response->message = "Failed to convert the still image frame";
    RCLCPP_WARN_STREAM(get_logger(), "Still image capture failed: " << response->message);
    return;
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "Captured still image (" << image->width << "x" << image->height << ")");

  still_cam_info_pub_->publish(cam_info);
  still_image_pub_->publish(std::move(image));

  response->success = true;
  response->message = "Captured still image";
}

}  // namespace raph_oak

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(raph_oak::OakWrapper)
