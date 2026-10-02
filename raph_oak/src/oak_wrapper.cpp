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

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// OpenCV
#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// DepthAI
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
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
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

using namespace std::chrono_literals;

namespace raph_oak
{

static const std::vector<std::string> UsbStrings = {"UNKNOWN", "LOW",   "FULL",
                                                    "HIGH",    "SUPER", "SUPER_PLUS"};

// How long the ~/capture_still service waits for the triggered frame to arrive from the device
static constexpr std::chrono::milliseconds StillCaptureTimeout{2000};

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
  // RGB
  rgb_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/rgb/image_raw", 1);
  rgb_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb/camera_info", 1);

  // RGB Compressed
  rgb_compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("~/rgb/image_raw/compressed", 1);

  // Left
  left_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/left/image_raw", 1);
  left_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/left/camera_info", 1);

  // Left Compressed
  left_compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("~/left/image_raw/compressed", 1);

  // Left Rect
  left_rect_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/left_rect/image_rect", 1);
  left_rect_cam_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>("~/left_rect/camera_info", 1);

  // Left Rect Compressed
  left_rect_compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("~/left_rect/image_rect/compressed", 1);

  // Right
  right_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/right/image_raw", 1);
  right_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/right/camera_info", 1);

  // Right Compressed
  right_compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("~/right/image_raw/compressed", 1);

  // Right Rect
  right_rect_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/right_rect/image_rect", 1);
  right_rect_cam_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>("~/right_rect/camera_info", 1);

  // Right Rect Compressed
  right_rect_compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("~/right_rect/image_rect/compressed", 1);

  // Depth
  stereo_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/stereo/image_raw", 1);
  stereo_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/stereo/camera_info", 1);

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
    nn_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/nn/image_raw", 1);
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

  // Only used to get camera info matrices
  auto img_converter = depthai_bridge::ImageConverter("oak_rgb_camera_optical_frame", false);

  // RGB
  rgb_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
    calibration_handler, dai::CameraBoardSocket::CAM_A, params_.rgb.width, params_.rgb.height));
  rgb_camera_info_.header.frame_id = "oak_rgb_camera_optical_frame";

  // Invalidate the cached still image camera info, it is rebuilt on the next capture
  still_camera_info_ = sensor_msgs::msg::CameraInfo();

  // Left (physically right camera, but becomes left after 180 degree rotation)
  left_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
    calibration_handler, calibration_handler.getStereoRightCameraId(), params_.mono.width,
    params_.mono.height));
  left_camera_info_.header.frame_id = "oak_left_camera_optical_frame";

  // Left Rect
  left_rect_camera_info_ = get_rotated_camera_info(
    img_converter.calibrationToCameraInfo(
      calibration_handler, calibration_handler.getStereoRightCameraId(), params_.mono.width,
      params_.mono.height),
    true);
  left_rect_camera_info_.header.frame_id = "oak_left_camera_optical_frame";

  // Right (physically left camera, but becomes right after 180 degree rotation)
  right_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
    calibration_handler, calibration_handler.getStereoLeftCameraId(), params_.mono.width,
    params_.mono.height));
  right_camera_info_.header.frame_id = "oak_right_camera_optical_frame";

  // Right Rect
  right_rect_camera_info_ = get_rotated_camera_info(
    img_converter.calibrationToCameraInfo(
      calibration_handler, calibration_handler.getStereoLeftCameraId(), params_.mono.width,
      params_.mono.height),
    true);
  right_rect_camera_info_.header.frame_id = "oak_right_camera_optical_frame";

  // Depth
  stereo_camera_info_ = img_converter.calibrationToCameraInfo(
    calibration_handler, calibration_handler.getStereoRightCameraId(), params_.mono.width,
    params_.mono.height);
  stereo_camera_info_.header.frame_id = "oak_stereo_camera_optical_frame";
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
  Params pipeline_params = params_;
  if (nn_failed_starts_ >= kMaxNnStartAttempts) {
    pipeline_params.nn.enabled = false;
  }

  pipeline_details_ = create_dai_pipeline(device_, pipeline_params, get_logger());

  // Register permanent callbacks on queues
  pipeline_details_.rgb_queue->addCallback([this]() {
    publish_image(rgb_img_pub_, rgb_cam_info_pub_, rgb_camera_info_, pipeline_details_.rgb_queue);
  });

  pipeline_details_.rgb_compressed_queue->addCallback([this]() {
    publish_compressed_image(
      rgb_compressed_pub_, "oak_rgb_camera_optical_frame", pipeline_details_.rgb_compressed_queue);
  });

  pipeline_details_.depth_queue->addCallback([this]() {
    publish_image(
      stereo_depth_pub_, stereo_cam_info_pub_, stereo_camera_info_, pipeline_details_.depth_queue);
  });

  pipeline_details_.left_queue->addCallback([this]() {
    publish_image(
      left_img_pub_, left_cam_info_pub_, left_camera_info_, pipeline_details_.left_queue);
  });

  pipeline_details_.left_compressed_queue->addCallback([this]() {
    publish_compressed_image(
      left_compressed_pub_, "oak_left_camera_optical_frame",
      pipeline_details_.left_compressed_queue);
  });

  pipeline_details_.left_rect_queue->addCallback([this]() {
    publish_image(
      left_rect_img_pub_, left_rect_cam_info_pub_, left_rect_camera_info_,
      pipeline_details_.left_rect_queue);
  });

  pipeline_details_.left_rect_compressed_queue->addCallback([this]() {
    publish_compressed_image(
      left_rect_compressed_pub_, "oak_left_camera_optical_frame",
      pipeline_details_.left_rect_compressed_queue);
  });

  pipeline_details_.right_queue->addCallback([this]() {
    publish_image(
      right_img_pub_, right_cam_info_pub_, right_camera_info_, pipeline_details_.right_queue);
  });

  pipeline_details_.right_compressed_queue->addCallback([this]() {
    publish_compressed_image(
      right_compressed_pub_, "oak_right_camera_optical_frame",
      pipeline_details_.right_compressed_queue);
  });

  pipeline_details_.right_rect_queue->addCallback([this]() {
    publish_image(
      right_rect_img_pub_, right_rect_cam_info_pub_, right_rect_camera_info_,
      pipeline_details_.right_rect_queue);
  });

  pipeline_details_.right_rect_compressed_queue->addCallback([this]() {
    publish_compressed_image(
      right_rect_compressed_pub_, "oak_right_camera_optical_frame",
      pipeline_details_.right_rect_compressed_queue);
  });

  pipeline_details_.imu_queue->addCallback([this]() { publish_imu(); });

  pipeline_details_.pointcloud_queue->addCallback([this]() { publish_pointcloud(); });

  if (pipeline_details_.nn_queue) {
    pipeline_details_.nn_queue->addCallback([this]() { publish_detections(); });
  }

  if (pipeline_details_.nn_image_queue) {
    pipeline_details_.nn_image_queue->addCallback([this]() {
      auto in_data = pipeline_details_.nn_image_queue->tryGet<dai::ImgFrame>();
      if (!in_data) {
        return;
      }

      std_msgs::msg::Header header;
      header.frame_id = "oak_rgb_camera_optical_frame";
      header.stamp =
        depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

      auto image = to_ros_image(in_data, header);
      if (image) {
        nn_img_pub_->publish(std::move(image));
      }
    });
  }

  // Initialize gate state flags to closed
  rgb_gate_open_ = false;
  left_gate_open_ = false;
  left_rect_gate_open_ = false;
  right_gate_open_ = false;
  right_rect_gate_open_ = false;
  depth_gate_open_ = false;
  imu_gate_open_ = false;
  pointcloud_gate_open_ = false;
  nn_gate_open_ = false;

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
    // Connect to device
    try {
      device_ = this->connect_to_device();
      this->run_pipeline();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to device: %s", e.what());

      // The device may be connected while the pipeline failed to start. Release both, so that the
      // next attempt starts from scratch instead of carrying on with a pipeline that never ran.
      pipeline_details_ = PipelineDetails{};
      if (device_) {
        device_->close();
        device_.reset();
      }

      rclcpp::sleep_for(5s);
      return;
    }

    auto calibration_handler = device_->readCalibration();
    this->fill_camera_info(calibration_handler);

    if (!params_.device.ir_laser_dot_projector_lazy) {
      device_->setIrLaserDotProjectorIntensity(params_.device.ir_laser_dot_projector_intensity);
    }
    device_->setIrFloodLightIntensity(params_.device.ir_flood_light_intensity);
  }

  if (device_->isClosed()) {
    RCLCPP_ERROR(get_logger(), "Device disconnected. Freeing resources...");

    if (pipeline_details_.pipeline) {
      pipeline_details_.pipeline->stop();
    }
    pipeline_details_ = PipelineDetails{};

    device_.reset();

    // Reset gate states
    rgb_gate_open_ = false;
    left_gate_open_ = false;
    left_rect_gate_open_ = false;
    right_gate_open_ = false;
    right_rect_gate_open_ = false;
    depth_gate_open_ = false;
    imu_gate_open_ = false;
    pointcloud_gate_open_ = false;
    nn_gate_open_ = false;

    return;
  }

  this->check_publishers();
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

void OakWrapper::check_publishers()
{
  manage_gate(
    rgb_img_pub_->get_subscription_count() + rgb_cam_info_pub_->get_subscription_count(), "rgb",
    pipeline_details_.rgb_gate_queue, rgb_gate_open_);

  manage_gate(
    stereo_depth_pub_->get_subscription_count() + stereo_cam_info_pub_->get_subscription_count(),
    "depth", pipeline_details_.depth_gate_queue, depth_gate_open_);

  manage_gate(
    left_img_pub_->get_subscription_count() + left_cam_info_pub_->get_subscription_count(), "left",
    pipeline_details_.left_gate_queue, left_gate_open_);

  manage_gate(
    left_rect_img_pub_->get_subscription_count() +
      left_rect_cam_info_pub_->get_subscription_count(),
    "left_rect", pipeline_details_.left_rect_gate_queue, left_rect_gate_open_);

  manage_gate(
    right_img_pub_->get_subscription_count() + right_cam_info_pub_->get_subscription_count(),
    "right", pipeline_details_.right_gate_queue, right_gate_open_);

  manage_gate(
    right_rect_img_pub_->get_subscription_count() +
      right_rect_cam_info_pub_->get_subscription_count(),
    "right_rect", pipeline_details_.right_rect_gate_queue, right_rect_gate_open_);

  manage_gate(
    imu_pub_->get_subscription_count(), "imu", pipeline_details_.imu_gate_queue, imu_gate_open_);

  manage_gate(
    pointcloud_pub_->get_subscription_count(), "pointcloud",
    pipeline_details_.pointcloud_gate_queue, pointcloud_gate_open_);

  // The publishers exist whenever nn.enabled is set, while the gate only exists when the network
  // made it into the pipeline
  if (nn_detections_pub_ && nn_img_pub_ && pipeline_details_.nn_gate_queue) {
    manage_gate(
      nn_detections_pub_->get_subscription_count() + nn_img_pub_->get_subscription_count(), "nn",
      pipeline_details_.nn_gate_queue, nn_gate_open_, static_cast<int>(params_.nn.fps));
  }

  if (params_.device.ir_laser_dot_projector_lazy && !device_->isClosed()) {
    const bool should_be_active =
      stereo_depth_pub_->get_subscription_count() + stereo_cam_info_pub_->get_subscription_count() >
      0;
    if (should_be_active && !laser_dot_projector_active_) {
      device_->setIrLaserDotProjectorIntensity(params_.device.ir_laser_dot_projector_intensity);
      laser_dot_projector_active_ = true;
    } else if (!should_be_active && laser_dot_projector_active_) {
      device_->setIrLaserDotProjectorIntensity(0.0);
      laser_dot_projector_active_ = false;
    }
  }
}

void OakWrapper::manage_gate(
  int subscription_count, const std::string & stream_name,
  std::shared_ptr<dai::InputQueue> gate_queue, bool & is_open, int fps)
{
  const bool should_be_open = subscription_count > 0;

  if (should_be_open && !is_open) {
    RCLCPP_INFO_STREAM(get_logger(), "Opening gate for \"" << stream_name << "\" stream");
    if (gate_queue) {
      gate_queue->send(dai::GateControl::openGate(-1, fps));
    }
    is_open = true;
  } else if (!should_be_open && is_open) {
    RCLCPP_INFO_STREAM(get_logger(), "Closing gate for \"" << stream_name << "\" stream");
    if (gate_queue) {
      gate_queue->send(dai::GateControl::closeGate());
    }
    is_open = false;
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

  if (!params_.device.ir_laser_dot_projector_lazy) {
    device_->setIrLaserDotProjectorIntensity(params_.device.ir_laser_dot_projector_intensity);
  }
  device_->setIrFloodLightIntensity(params_.device.ir_flood_light_intensity);
}

void OakWrapper::publish_image(
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> img_pub,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> cam_info_pub,
  sensor_msgs::msg::CameraInfo cam_info, std::shared_ptr<dai::MessageQueue> queue)
{
  auto in_data = queue->tryGet<dai::ImgFrame>();
  if (!in_data) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Failed to get data from \"" << queue->getName() << "\" queue");
    return;
  }

  cam_info.header.stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

  auto image = to_ros_image(in_data, cam_info.header);
  if (!image) {
    return;
  }

  cam_info_pub->publish(cam_info);
  img_pub->publish(std::move(image));
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
  } else if (
    in_data->getType() == dai::ImgFrame::Type::BGR888i ||
    in_data->getType() == dai::ImgFrame::Type::RGB888i) {
    image->encoding = in_data->getType() == dai::ImgFrame::Type::BGR888i ? "bgr8" : "rgb8";
    image->step = image->width * 3;
    image->data.assign(in_data->getData().begin(), in_data->getData().end());
  } else if (
    in_data->getType() == dai::ImgFrame::Type::BGR888p ||
    in_data->getType() == dai::ImgFrame::Type::RGB888p) {
    // Planar frames (what RVC2 networks take) need the channels interleaved for ROS
    image->encoding = in_data->getType() == dai::ImgFrame::Type::BGR888p ? "bgr8" : "rgb8";
    image->step = image->width * 3;
    image->data.resize(static_cast<size_t>(image->width) * image->height * 3);

    const cv::Size plane_size(static_cast<int>(image->width), static_cast<int>(image->height));
    const auto plane_bytes = static_cast<size_t>(image->width) * image->height;
    auto * in_planes = in_data->getData().data();

    const std::array<cv::Mat, 3> planes{
      cv::Mat(plane_size, CV_8UC1, in_planes),
      cv::Mat(plane_size, CV_8UC1, in_planes + plane_bytes),
      cv::Mat(plane_size, CV_8UC1, in_planes + (2 * plane_bytes))};
    cv::Mat out_mat(plane_size, CV_8UC3, image->data.data());
    cv::merge(planes.data(), planes.size(), out_mat);
  } else if (
    in_data->getType() == dai::ImgFrame::Type::RAW8 ||
    in_data->getType() == dai::ImgFrame::Type::GRAY8) {
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
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> img_pub,
  const std::string & frame_id, std::shared_ptr<dai::MessageQueue> queue)
{
  auto in_data = queue->tryGet<dai::EncodedFrame>();
  if (!in_data) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Failed to get data from \"" << queue->getName() << "\" queue");
    return;
  }

  auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
  image->header.stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());
  image->header.frame_id = frame_id;
  image->format = "jpeg";
  image->data.assign(in_data->getData().begin(), in_data->getData().end());

  img_pub->publish(std::move(image));
}

void OakWrapper::publish_imu()
{
  auto in_data = pipeline_details_.imu_queue->tryGet<dai::IMUData>();
  if (!in_data) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to get data from \"" << pipeline_details_.imu_queue->getName() << "\" queue");
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

void OakWrapper::publish_pointcloud()
{
  auto in_data = pipeline_details_.pointcloud_queue->tryGet<dai::PointCloudData>();

  if (!in_data) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to get data from \"" << pipeline_details_.pointcloud_queue->getName() << "\" queue");
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

void OakWrapper::publish_detections()
{
  auto in_data = pipeline_details_.nn_queue->tryGet<dai::ImgDetections>();

  if (!in_data) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to get data from \"" << pipeline_details_.nn_queue->getName() << "\" queue");
    return;
  }

  vision_msgs::msg::Detection2DArray msg;
  msg.header.frame_id = "oak_rgb_camera_optical_frame";
  msg.header.stamp =
    depthai_bridge::getFrameTime(ros_base_time_, steady_base_time_, in_data->getTimestamp());

  // Detections are normalized to the network input frame, which is what ~/nn/image_raw
  // publishes, so scale them into that frame's pixel coordinates. Note that this frame is a
  // letterboxed version of the RGB stream, so bounding boxes do not line up with ~/rgb/image_raw
  // without remapping.
  double width = 1.0;
  double height = 1.0;
  if (in_data->transformation) {
    const auto size = in_data->transformation->getSize();
    width = static_cast<double>(size.first);
    height = static_cast<double>(size.second);
  }

  msg.detections.reserve(in_data->detections.size());
  for (const auto & detection : in_data->detections) {
    vision_msgs::msg::Detection2D ros_detection;
    ros_detection.header = msg.header;
    ros_detection.id =
      detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = ros_detection.id;
    hypothesis.hypothesis.score = detection.confidence;
    ros_detection.results.push_back(hypothesis);

    // [xmin, ymin, xmax, ymax]
    const auto bbox = detection.getOuterBoundingBox();
    ros_detection.bbox.size_x = (bbox[2] - bbox[0]) * width;
    ros_detection.bbox.size_y = (bbox[3] - bbox[1]) * height;
    ros_detection.bbox.center.position.x = (bbox[0] * width) + (ros_detection.bbox.size_x / 2.0);
    ros_detection.bbox.center.position.y = (bbox[1] * height) + (ros_detection.bbox.size_y / 2.0);

    msg.detections.push_back(ros_detection);
  }

  nn_detections_pub_->publish(msg);
}

void OakWrapper::capture_still(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (
    !device_ || device_->isClosed() || !pipeline_details_.still_trigger_queue ||
    !pipeline_details_.still_image_queue || !calibration_handler_) {
    response->success = false;
    response->message = "Device is not connected";
    RCLCPP_WARN_STREAM(get_logger(), "Still image capture failed: " << response->message);
    return;
  }

  // Drop anything left over from a previous, timed out capture
  pipeline_details_.still_image_queue->tryGetAll();
  pipeline_details_.still_trigger_queue->send(std::make_shared<dai::Buffer>());

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
