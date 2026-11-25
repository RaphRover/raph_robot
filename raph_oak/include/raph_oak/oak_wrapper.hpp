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
#include <atomic>
#include <chrono>
#include <cstddef>
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

// ROS
#include "raph_oak/control.hpp"
#include "raph_oak/conversions.hpp"
#include "raph_oak/demand.hpp"
#include "raph_oak/frame_clock.hpp"
#include "raph_oak/nn_model.hpp"
#include "raph_oak/nn_overlay.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/pipeline.hpp"
#include "raph_oak/streams.hpp"
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
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/label_info.hpp"

namespace raph_oak
{

class OakWrapper : public rclcpp::Node
{
public:
  explicit OakWrapper(rclcpp::NodeOptions options);

private:
  std::shared_ptr<dai::Device> device_;
  PipelineDetails pipeline_details_;

  // ROS Publishers of an image stream, see kImageStreamSpecs
  struct ImagePublisherSet
  {
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub;
    /// Empty if the stream has no compressed variant
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> compressed_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> camera_info_pub;
    /// Camera info from the factory calibration. It provides the distortion, the rectification
    /// rotation and the baseline, the intrinsics come from the frame, see
    /// get_camera_info_from_frame()
    sensor_msgs::msg::CameraInfo calibration_info;
  };
  std::array<ImagePublisherSet, kImageStreamCount> image_publisher_sets_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> still_image_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> still_cam_info_pub_;
  // The publishers of the results of the network are created once the pipeline is built, because
  // the model decides which one of them it needs, see create_nn_publishers()
  std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> nn_detections_pub_;
  std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Classification>> nn_classification_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> nn_segmentation_pub_;
  std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::LabelInfo>> nn_label_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> nn_overlay_pub_;
  NnOverlay nn_overlay_;

  // ROS Services
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> capture_still_srv_;

  // Converts the host timestamps of the DepthAI messages to ROS time
  FrameClock frame_clock_;
  // Created once the parameters, which hold the frame id prefix, are known
  std::optional<ImuConverter> imu_converter_;
  std::optional<PointCloudConverter> pointcloud_converter_;

  // Last state sent to each control target (true = gate open)
  std::array<std::atomic<bool>, Control::kCount> control_state_{};
  // Rate limit last sent to each open gate, -1 for none. Only used by the check timer
  std::array<int, Control::kCount> gate_fps_{};
  // The stereo sensors are paused while no stereo output is needed
  bool stereo_streaming_{true};
  // When the stereo sensors were last stopped, a start right after is lost by the camera
  std::chrono::steady_clock::time_point stereo_stopped_at_;

  // The still image resolution is only known once the first frame arrives, so its camera info is
  // built from the calibration and the frame on demand
  std::optional<dai::CalibrationHandler> calibration_handler_;

  std::shared_ptr<rclcpp::TimerBase> check_timer_;

  ParamListener param_listener_;
  Params params_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  dai::StereoDepthConfig depth_config_;

  // Last IR laser dot projector intensity sent to the device, empty if unknown
  std::optional<float> laser_intensity_;

  // Consecutive failed pipeline starts with the neural network in the pipeline. Some model problems
  // (an RVC2 shave count that does not fit, for one) only surface on start, so after
  // kMaxNnStartAttempts the network is dropped rather than keeping the camera down. A few
  // attempts are allowed so that a transient failure does not disable the network for good.
  static constexpr int kMaxNnStartAttempts = 3;
  int nn_failed_starts_ = 0;

  void run_pipeline();
  void create_ros_publishers();
  std::string tf_frame(const std::string & frame) const;
  void create_ros_services();
  void fill_camera_info(const dai::CalibrationHandler & calibration_handler);
  sensor_msgs::msg::CameraInfo get_still_camera_info(dai::ImgFrame & frame);
  void check_timer_callback();
  void release_device();
  std::shared_ptr<dai::Device> connect_to_device();
  void check_publishers();
  SubscriberCounts count_subscribers() const;
  void apply_demand(const Demand & demand);
  void reset_control_state();
  void set_gate(Control gate, bool should_be_open, int fps = -1);
  void set_stereo_streaming(bool streaming);
  void post_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  void update_parameters();
  void send_parameters() const;
  void publish_image(ImageStreamId stream_id, const std::shared_ptr<dai::ADatatype> & message);
  void publish_compressed_image(
    ImageStreamId stream_id, const std::shared_ptr<dai::ADatatype> & message);
  void publish_imu(const std::shared_ptr<dai::ADatatype> & message);
  void publish_pointcloud(const std::shared_ptr<dai::ADatatype> & message);
  void create_nn_publishers(const PipelineDetails & details);
  void publish_nn_result(NnKind kind, const std::shared_ptr<dai::ADatatype> & message);
  void publish_detections(const std::shared_ptr<dai::ADatatype> & message);
  void publish_classification(const std::shared_ptr<dai::ADatatype> & message);
  void publish_segmentation(const std::shared_ptr<dai::ADatatype> & message);
  /// Hands the result to the overlay if someone subscribed to it
  void update_nn_overlay(NnResult result, std::int64_t sequence);
  void capture_still(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
};

}  // namespace raph_oak
