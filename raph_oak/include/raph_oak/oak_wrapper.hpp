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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

// DepthAI
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"

// ROS
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace raph_oak
{

class OakWrapper : public rclcpp::Node
{
public:
  explicit OakWrapper(rclcpp::NodeOptions options);

private:
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::Pipeline> pipeline_;

  // DepthAI data queues
  std::shared_ptr<dai::MessageQueue> rgb_queue_;
  std::shared_ptr<dai::MessageQueue> rgb_compressed_queue_;
  std::shared_ptr<dai::MessageQueue> left_queue_;
  std::shared_ptr<dai::MessageQueue> left_compressed_queue_;
  std::shared_ptr<dai::MessageQueue> left_rect_queue_;
  std::shared_ptr<dai::MessageQueue> left_rect_compressed_queue_;
  std::shared_ptr<dai::MessageQueue> right_queue_;
  std::shared_ptr<dai::MessageQueue> right_compressed_queue_;
  std::shared_ptr<dai::MessageQueue> right_rect_queue_;
  std::shared_ptr<dai::MessageQueue> right_rect_compressed_queue_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;
  std::shared_ptr<dai::MessageQueue> imu_queue_;
  std::shared_ptr<dai::InputQueue> depth_config_queue_;
  std::shared_ptr<dai::MessageQueue> pointcloud_queue_;
  std::shared_ptr<dai::MessageQueue> still_image_queue_;
  std::shared_ptr<dai::InputQueue> still_trigger_queue_;

  // ROS Publishers
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rgb_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> rgb_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> rgb_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> left_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> left_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> left_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> left_rect_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> left_rect_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> left_rect_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> right_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> right_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> right_rect_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> right_rect_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_rect_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> stereo_depth_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> stereo_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> still_image_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> still_cam_info_pub_;

  // ROS Services
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> capture_still_srv_;

  std::shared_ptr<depthai_bridge::ImuConverter> imu_converter_;
  std::shared_ptr<depthai_bridge::PointCloudConverter> pointcloud_converter_;

  // Callback IDs for dynamic callback management
  int rgb_callback_id_{-1};
  int rgb_compressed_callback_id_{-1};
  int left_callback_id_{-1};
  int left_compressed_callback_id_{-1};
  int left_rect_callback_id_{-1};
  int left_rect_compressed_callback_id_{-1};
  int right_callback_id_{-1};
  int right_compressed_callback_id_{-1};
  int right_rect_callback_id_{-1};
  int right_rect_compressed_callback_id_{-1};
  int depth_callback_id_{-1};
  int imu_callback_id_{-1};
  int pointcloud_callback_id_{-1};

  // Camera info for callbacks
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo left_camera_info_;
  sensor_msgs::msg::CameraInfo left_rect_camera_info_;
  sensor_msgs::msg::CameraInfo right_camera_info_;
  sensor_msgs::msg::CameraInfo right_rect_camera_info_;
  sensor_msgs::msg::CameraInfo stereo_camera_info_;

  // The still image resolution is only known once the first frame arrives, so its camera info is
  // built on demand and cached here, together with the calibration it was derived from.
  std::optional<dai::CalibrationHandler> calibration_handler_;
  sensor_msgs::msg::CameraInfo still_camera_info_;

  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  rclcpp::Time ros_base_time_;
  std::shared_ptr<rclcpp::TimerBase> check_timer_;

  ParamListener param_listener_;
  Params params_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  dai::StereoDepthConfig depth_config_;

  bool laser_dot_projector_active_ = false;

  void run_pipeline();
  void create_ros_publishers();
  void create_ros_services();
  void fill_camera_info(const dai::CalibrationHandler & calibration_handler);
  const sensor_msgs::msg::CameraInfo & get_still_camera_info(uint32_t width, uint32_t height);
  void check_timer_callback();
  std::shared_ptr<dai::Device> connect_to_device();
  void check_publishers();
  void manage_callback(
    int subscription_count, std::shared_ptr<dai::MessageQueue> queue, int & callback_id,
    std::function<void()> callback);
  void post_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  void update_parameters();
  void send_parameters() const;
  void publish_image(
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> img_pub,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> cam_info_pub,
    sensor_msgs::msg::CameraInfo cam_info, std::shared_ptr<dai::MessageQueue> queue);
  std::unique_ptr<sensor_msgs::msg::Image> to_ros_image(
    const std::shared_ptr<dai::ImgFrame> & in_data, const std_msgs::msg::Header & header) const;
  void publish_compressed_image(
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> img_pub,
    const std::string & frame_id, std::shared_ptr<dai::MessageQueue> queue);
  void publish_imu();
  void publish_pointcloud();
  void capture_still(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
};

}  // namespace raph_oak
