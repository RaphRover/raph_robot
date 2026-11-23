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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// OpenCV
#include <opencv2/core/hal/interface.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

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
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// XLink
#include "XLink/XLinkPublicDefines.h"

// ROS
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "raph_oak/camera_info.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"
#include "raph_oak/pipeline.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace raph_oak
{

static const std::vector<std::string> UsbStrings = {"UNKNOWN", "LOW",   "FULL",
                                                    "HIGH",    "SUPER", "SUPER_PLUS"};

// How long the ~/capture_still service waits for the triggered frame to arrive from the device
static constexpr std::chrono::milliseconds StillCaptureTimeout{2000};

// Minimum time between stopping the stereo sensors and starting them again. A start sent shortly
// after a stop is ignored by the camera and the stereo streams stay silent.
static constexpr std::chrono::milliseconds StereoRestartDelay{3000};

OakWrapper::OakWrapper(rclcpp::NodeOptions options)
: Node("oak_wrapper", options),
  steady_base_time_(std::chrono::steady_clock::now()),
  param_listener_(get_node_parameters_interface())
{
  ros_base_time_ = rclcpp::Clock().now();
  update_parameters();
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
    std::bind(&OakWrapper::post_set_parameters_callback, this, std::placeholders::_1));

  this->create_ros_publishers();
  this->create_ros_services();

  // TODO(fszkudlarek): make sure this works under namespace
  imu_converter_ = std::make_shared<depthai_bridge::ImuConverter>(
    "oak_imu_frame", depthai_bridge::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO, 0.001, 0.00001);

  pointcloud_converter_ =
    std::make_shared<depthai_bridge::PointCloudConverter>("oak_stereo_camera_optical_frame");

  check_timer_ = create_wall_timer(100ms, std::bind(&OakWrapper::check_timer_callback, this));
}

void OakWrapper::create_ros_publishers()
{
  for (const auto & spec : ImageStreamSpecs) {
    auto & stream = image_publisher_sets_[image_index(spec.id)];
    const std::string topic = std::string("~/") + spec.topic;
    stream.image_pub = create_publisher<sensor_msgs::msg::Image>(topic, 1);
    if (spec.has_compressed()) {
      stream.compressed_pub =
        create_publisher<sensor_msgs::msg::CompressedImage>(topic + "/compressed", 1);
    }
    stream.camera_info_pub =
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

  // Only used to get camera info matrices
  auto img_converter = depthai_bridge::ImageConverter("oak_rgb_camera_optical_frame", false);

  const auto camera_info = [&](dai::CameraBoardSocket socket, int width, int height) {
    return img_converter.calibrationToCameraInfo(calibration_handler, socket, width, height);
  };
  const auto stream_info = [this](ImageStreamId id) -> sensor_msgs::msg::CameraInfo & {
    return image_publisher_sets_[image_index(id)].calibration_info;
  };

  // The camera is mounted upside down, so the images are rotated by 180 degrees and the physically
  // right camera is the left one and the other way round
  const auto left_socket = calibration_handler.getStereoRightCameraId();
  const auto right_socket = calibration_handler.getStereoLeftCameraId();
  const int mono_width = static_cast<int>(params_.mono.width);
  const int mono_height = static_cast<int>(params_.mono.height);

  stream_info(ImageStreamId::Rgb) = get_rotated_camera_info(camera_info(
    dai::CameraBoardSocket::CAM_A, static_cast<int>(params_.rgb.width),
    static_cast<int>(params_.rgb.height)));
  stream_info(ImageStreamId::Left) =
    get_rotated_camera_info(camera_info(left_socket, mono_width, mono_height));
  stream_info(ImageStreamId::LeftRect) =
    get_rotated_camera_info(camera_info(left_socket, mono_width, mono_height), true);
  stream_info(ImageStreamId::Right) =
    get_rotated_camera_info(camera_info(right_socket, mono_width, mono_height));
  stream_info(ImageStreamId::RightRect) =
    get_rotated_camera_info(camera_info(right_socket, mono_width, mono_height), true);

  // The undistorted RGB image has the pose of the RGB camera
  stream_info(ImageStreamId::RgbRect) = stream_info(ImageStreamId::Rgb);

  // Depth: aligned to the rectified right sensor, which is the left image after the rotation, at
  // half the mono resolution. Only a fallback: the camera info of every depth frame is built from
  // the intrinsics DepthAI attaches to it.
  auto & depth_info = stream_info(ImageStreamId::Depth);
  depth_info = get_scaled_camera_info(
    stream_info(ImageStreamId::LeftRect), params_.mono.width / 2, params_.mono.height / 2);
  depth_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  depth_info.p[3] = 0.0;

  for (const auto & spec : ImageStreamSpecs) {
    stream_info(spec.id).header.frame_id = spec.frame_id;
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

  // Only used to get camera info matrices
  auto img_converter = depthai_bridge::ImageConverter("oak_rgb_camera_optical_frame", false);

  // Same sensor as the RGB stream, but at the full resolution the still frame came in at
  still_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
    *calibration_handler_, dai::CameraBoardSocket::CAM_A, static_cast<int>(width),
    static_cast<int>(height)));
  still_camera_info_.header.frame_id = "oak_rgb_camera_optical_frame";

  return still_camera_info_;
}

void OakWrapper::run_pipeline()
{
  pipeline_details_ = create_dai_pipeline(device_, params_);
  depth_config_ = pipeline_details_.depth_config;

  // Register permanent callbacks on queues
  for (const auto & spec : ImageStreamSpecs) {
    const auto id = spec.id;
    const auto & queues = pipeline_details_.images[image_index(id)];
    queues.raw->addCallback(
      [this, id](const std::shared_ptr<dai::ADatatype> & message) { publish_image(id, message); });
    if (queues.compressed) {
      queues.compressed->addCallback([this, id](const std::shared_ptr<dai::ADatatype> & message) {
        publish_compressed_image(id, message);
      });
    }
  }

  pipeline_details_.imu_queue->addCallback(
    [this](const std::shared_ptr<dai::ADatatype> & message) { publish_imu(message); });

  pipeline_details_.pointcloud_queue->addCallback(
    [this](const std::shared_ptr<dai::ADatatype> & message) { publish_pointcloud(message); });

  // All gates start closed and the sensors streaming
  reset_control_state();

  pipeline_details_.pipeline->start();
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
    get_logger(), "USB Speed: " << UsbStrings[static_cast<int32_t>(device->getUsbSpeed())]);

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
  for (const auto & spec : ImageStreamSpecs) {
    const auto & stream = image_publisher_sets_[image_index(spec.id)];
    counts.images[image_index(spec.id)] = {
      stream.image_pub->get_subscription_count(),
      stream.compressed_pub ? stream.compressed_pub->get_subscription_count() : 0,
      stream.camera_info_pub->get_subscription_count()};
  }
  counts.pointcloud = pointcloud_pub_->get_subscription_count();
  counts.imu = imu_pub_->get_subscription_count();
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
    StereoRestartDelay);
  if (sensors == StereoSensorAction::Start) {
    set_stereo_streaming(true);
  }

  // The point cloud gate limits the rate of the depth frames fed to the point cloud node
  const int pointcloud_fps = pointcloud_rate(params_.pointcloud.fps);
  for (const auto gate : Control::all()) {
    if (gate.follows_demand()) {
      set_gate(gate, wanted_gates.is_open(gate), gate == Control::PointCloud ? pointcloud_fps : -1);
    }
  }

  if (sensors == StereoSensorAction::Stop) {
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

bool OakWrapper::is_gate_open(Control gate) const { return control_state_[gate.index()]; }

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

  for (const Control sensor : {Control::CameraB, Control::CameraC}) {
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

void OakWrapper::publish_image(ImageStreamId id, const std::shared_ptr<dai::ADatatype> & message)
{
  const auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not an image frame");
    return;
  }

  const auto & spec = get_image_stream_spec(id);
  const auto & stream = image_publisher_sets_[image_index(id)];

  // The intrinsics of a rectified or undistorted image are only known to DepthAI, so the camera
  // info is built from the frame
  auto cam_info = spec.rectified
                    ? get_rectified_camera_info(
                        in_data->getTransformation(), in_data->getWidth(), in_data->getHeight(),
                        stream.calibration_info, spec.rotated_at_sensor)
                    : stream.calibration_info;
  cam_info.header.stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

  auto image = to_ros_image(in_data, cam_info.header);
  if (!image) {
    return;
  }

  stream.camera_info_pub->publish(cam_info);
  stream.image_pub->publish(std::move(image));
}

std::unique_ptr<sensor_msgs::msg::Image> OakWrapper::to_ros_image(
  const std::shared_ptr<dai::ImgFrame> & in_data, const std_msgs::msg::Header & header) const
{
  auto image = std::make_unique<sensor_msgs::msg::Image>();

  image->header = header;
  image->width = in_data->getWidth();
  image->height = in_data->getHeight();
  image->is_bigendian = 1U;

  if (in_data->getType() == dai::ImgFrame::Type::NV12) {
    image->encoding = "bgr8";
    image->step = image->width * 3;
    image->data.resize(image->width * image->height * 3);

    cv::Mat const in_mat(
      cv::Size(in_data->getWidth(), in_data->getHeight() * 3 / 2), CV_8UC1,
      in_data->getData().data());
    cv::Mat out_mat(
      cv::Size(in_data->getWidth(), in_data->getHeight()), CV_8UC3, image->data.data());
    cv::cvtColor(in_mat, out_mat, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
  } else if (in_data->getType() == dai::ImgFrame::Type::RAW8) {
    image->encoding = "mono8";
    image->step = image->width;
    image->data.assign(in_data->getData().begin(), in_data->getData().end());
  } else if (in_data->getType() == dai::ImgFrame::Type::RAW16) {
    image->encoding = "16UC1";
    image->is_bigendian = 0U;
    image->step = image->width * 2;
    image->data.assign(in_data->getData().begin(), in_data->getData().end());
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "Unsupported image frame type: " << static_cast<int>(in_data->getType()));
    return nullptr;
  }

  return image;
}

void OakWrapper::publish_compressed_image(
  ImageStreamId id, const std::shared_ptr<dai::ADatatype> & message)
{
  const auto in_data = std::dynamic_pointer_cast<dai::EncodedFrame>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not an encoded frame");
    return;
  }

  const auto & spec = get_image_stream_spec(id);
  const auto & stream = image_publisher_sets_[image_index(id)];
  const auto stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

  // The raw image publishes the camera info while the raw gate is open. Otherwise the compressed
  // stream (which is also opened for camera info subscribers) takes over.
  if (!is_gate_open(spec.raw_gate) && stream.camera_info_pub->get_subscription_count() > 0) {
    auto cam_info = spec.rectified
                      ? get_rectified_camera_info(
                          in_data->transformation, in_data->getWidth(), in_data->getHeight(),
                          stream.calibration_info, spec.rotated_at_sensor)
                      : stream.calibration_info;
    cam_info.header.stamp = stamp;
    stream.camera_info_pub->publish(cam_info);
  }

  if (stream.compressed_pub->get_subscription_count() == 0) {
    return;
  }

  auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
  image->header.stamp = stamp;
  image->header.frame_id = stream.calibration_info.header.frame_id;
  image->format = "jpeg";
  image->data.assign(in_data->getData().begin(), in_data->getData().end());

  stream.compressed_pub->publish(std::move(image));
}

void OakWrapper::publish_imu(const std::shared_ptr<dai::ADatatype> & message)
{
  auto in_data = std::dynamic_pointer_cast<dai::IMUData>(message);
  if (!in_data) {
    RCLCPP_WARN(get_logger(), "Received a message that is not IMU data");
    return;
  }

  std::deque<sensor_msgs::msg::Imu> op_msgs;
  imu_converter_->toRosMsg(in_data, op_msgs);

  while (!op_msgs.empty()) {
    sensor_msgs::msg::Imu imu = op_msgs.front();
    op_msgs.pop_front();

    // Mark the orientation as unknown
    imu.orientation_covariance[0] = -1.0;

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

  std::deque<sensor_msgs::msg::PointCloud2> op_msgs;
  pointcloud_converter_->toRosMsg(in_data, op_msgs);

  while (!op_msgs.empty()) {
    const sensor_msgs::msg::PointCloud2 pointcloud = op_msgs.front();
    op_msgs.pop_front();

    pointcloud_pub_->publish(pointcloud);
  }
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
  send_control(pipeline_details_, Control::Still, dai::GateControl::openGate(1));

  bool timed_out = false;
  auto in_data =
    pipeline_details_.still_image_queue->get<dai::ImgFrame>(StillCaptureTimeout, timed_out);
  if (timed_out || !in_data) {
    response->success = false;
    response->message = "Timed out waiting for the still image frame";
    RCLCPP_WARN_STREAM(get_logger(), "Still image capture failed: " << response->message);
    return;
  }

  sensor_msgs::msg::CameraInfo cam_info =
    get_still_camera_info(in_data->getWidth(), in_data->getHeight());
  cam_info.header.stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

  auto image = to_ros_image(in_data, cam_info.header);
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
