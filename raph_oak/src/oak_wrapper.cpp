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

#include "rclcpp/rclcpp.hpp"

#include "depthai/depthai.hpp"

#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "raph_oak/oak_wrapper_parameters.hpp"

using namespace std::chrono_literals;

namespace raph_oak
{

// Calculate the camera info for the images rotated by 180 degrees
static sensor_msgs::msg::CameraInfo get_rotated_camera_info(
  const sensor_msgs::msg::CameraInfo & original_info,
  bool is_rectified = false)
{
  sensor_msgs::msg::CameraInfo rotated_info = original_info;
  double w = static_cast<double>(original_info.width);
  double h = static_cast<double>(original_info.height);

  // 1. Shift Principal Point in K (Intrinsics)
  // K[2] is cx, K[5] is cy
  rotated_info.k[2] = w - 1.0 - original_info.k[2];
  rotated_info.k[5] = h - 1.0 - original_info.k[5];

  // 2. Shift Principal Point in P (Projection)
  // P[2] is cx, P[6] is cy
  rotated_info.p[2] = w - 1.0 - original_info.p[2];
  rotated_info.p[6] = h - 1.0 - original_info.p[6];

  // 3. Handle Distortion (D)
  if (!is_rectified) {
    // If NOT rectified, flip the tangential distortion coefficients (p1, p2) in D,
    // and keep the radial coefficients (k1, k2, k3, k4) the same.
    if (rotated_info.d.size() >= 4) {
      rotated_info.d[2] = -original_info.d[2]; // p1
      rotated_info.d[3] = -original_info.d[3]; // p2
    }
  } else {
    // If ALREADY rectified, D should be zeros. Ensure it stays that way.
    std::fill(rotated_info.d.begin(), rotated_info.d.end(), 0.0);
  }

  return rotated_info;
}

class OakWrapper : public rclcpp::Node
{
  std::unique_ptr<dai::Device> device_;

  // DepthAI data queues
  std::shared_ptr<dai::DataOutputQueue> rgb_queue_;
  std::shared_ptr<dai::DataOutputQueue> rgb_compressed_queue_;
  std::shared_ptr<dai::DataOutputQueue> left_queue_;
  std::shared_ptr<dai::DataOutputQueue> left_compressed_queue_;
  std::shared_ptr<dai::DataOutputQueue> right_queue_;
  std::shared_ptr<dai::DataOutputQueue> right_compressed_queue_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
  std::shared_ptr<dai::DataOutputQueue> imu_queue_;
  std::shared_ptr<dai::DataInputQueue> depth_config_queue_;

  // ROS Publishers
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rgb_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> rgb_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> rgb_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> left_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> left_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> left_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> right_img_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> right_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_compressed_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> stereo_depth_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> stereo_cam_info_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;

  std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;

  // Callback IDs for dynamic callback management
  int rgb_callback_id_{-1};
  int rgb_compressed_callback_id_{-1};
  int left_callback_id_{-1};
  int left_compressed_callback_id_{-1};
  int right_callback_id_{-1};
  int right_compressed_callback_id_{-1};
  int depth_callback_id_{-1};
  int imu_callback_id_{-1};

  // Camera info for callbacks
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo left_camera_info_;
  sensor_msgs::msg::CameraInfo right_camera_info_;
  sensor_msgs::msg::CameraInfo stereo_camera_info_;

  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  rclcpp::Time ros_base_time_;
  std::shared_ptr<rclcpp::TimerBase> check_timer_;

  ParamListener param_listener_;
  Params params_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  dai::RawStereoDepthConfig depth_config_;

public:
  OakWrapper(rclcpp::NodeOptions options)
  : Node("oak_wrapper", options),
    steady_base_time_(std::chrono::steady_clock::now()),
    param_listener_(get_node_parameters_interface())
  {
    ros_base_time_ = rclcpp::Clock().now();
    update_parameters();
    parameter_callback_handle_ =
      this->add_post_set_parameters_callback(std::bind(&OakWrapper::post_set_parameters_callback,
        this, std::placeholders::_1));

    auto pipeline = create_dai_pipeline();
    device_ = std::make_unique<dai::Device>(pipeline, dai::UsbSpeed::FULL);

    auto calibration_handler = device_->readCalibration();
    auto eeprom = calibration_handler.getEepromData();

    RCLCPP_INFO_STREAM(get_logger(), "Product name: " << eeprom.productName);
    RCLCPP_INFO_STREAM(get_logger(), "Board custom: " << eeprom.boardCustom);
    RCLCPP_INFO_STREAM(get_logger(), "Board name: " << eeprom.boardName);
    RCLCPP_INFO_STREAM(get_logger(), "Board Rev: " << eeprom.boardRev);
    RCLCPP_INFO_STREAM(get_logger(), "Board Conf: " << eeprom.boardConf);
    RCLCPP_INFO_STREAM(get_logger(), "Hardware Conf: " << eeprom.hardwareConf);
    RCLCPP_INFO_STREAM(get_logger(), "Batch name: " << eeprom.batchName);

    // Set all output queues to non-blocking with size 1
    rgb_queue_ = device_->getOutputQueue("rgb", 1, true);
    rgb_compressed_queue_ = device_->getOutputQueue("rgb_compressed", 1, true);
    left_queue_ = device_->getOutputQueue("left", 1, true);
    left_compressed_queue_ = device_->getOutputQueue("left_compressed", 1, true);
    right_queue_ = device_->getOutputQueue("right", 1, true);
    right_compressed_queue_ = device_->getOutputQueue("right_compressed", 1, true);
    depth_queue_ = device_->getOutputQueue("depth", 1, true);
    imu_queue_ = device_->getOutputQueue("imu", 1, true);

    // Only used to get camera info matrices
    auto img_converter = dai::rosBridge::ImageConverter(false);

    // RGB
    rgb_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::CAM_A, 1344, 1008));
    rgb_camera_info_.header.frame_id = "oak_rgb_camera_optical_frame";
    rgb_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/rgb/image_raw", 10);
    rgb_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb/camera_info", 10);

    // RGB Compressed
    rgb_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/rgb/image_raw/compressed", 10);

    // Left (physically right camera, but becomes left after 180 degree rotation)
    left_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
      calibration_handler, calibration_handler.getStereoRightCameraId(), 1280, 800), true);
    left_camera_info_.header.frame_id = "oak_left_camera_optical_frame";
    left_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/left/image_rect", 10);
    left_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/left/camera_info", 10);

    // Left Compressed
    left_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/left/image_rect/compressed", 10);

    // Right (physically left camera, but becomes right after 180 degree rotation)
    right_camera_info_ = get_rotated_camera_info(img_converter.calibrationToCameraInfo(
      calibration_handler, calibration_handler.getStereoLeftCameraId(), 1280, 800), true);
    right_camera_info_.header.frame_id = "oak_right_camera_optical_frame";
    right_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/right/image_rect", 10);
    right_cam_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("~/right/camera_info", 10);

    // Right Compressed
    right_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/right/image_rect/compressed", 10);

    // Depth
    stereo_camera_info_ = img_converter.calibrationToCameraInfo(calibration_handler,
        calibration_handler.getStereoRightCameraId(), 1280, 800);
    stereo_camera_info_.header.frame_id = "oak_stereo_camera_optical_frame";
    stereo_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/stereo/image_raw", 10);
    stereo_cam_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("~/stereo/camera_info", 10);

    // Depth Config
    depth_config_queue_ = device_->getInputQueue("depth_config");

    // IMU
    imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>(
      "oak_imu_frame",
      dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO, 0.001, 0.00001);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);

    check_timer_ = create_wall_timer(100ms, std::bind(&OakWrapper::check_publishers, this));
  }

private:
  dai::Pipeline create_dai_pipeline()
  {
    dai::Pipeline pipeline;

    // Create nodes
    auto mono_left_node = pipeline.create<dai::node::MonoCamera>();
    auto mono_right_node = pipeline.create<dai::node::MonoCamera>();
    auto stereo_depth_node = pipeline.create<dai::node::StereoDepth>();
    auto left_encoder_node = pipeline.create<dai::node::VideoEncoder>();
    auto right_encoder_node = pipeline.create<dai::node::VideoEncoder>();
    auto xout_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_left_compressed = pipeline.create<dai::node::XLinkOut>();
    auto xout_right = pipeline.create<dai::node::XLinkOut>();
    auto xout_right_compressed = pipeline.create<dai::node::XLinkOut>();
    auto xout_depth = pipeline.create<dai::node::XLinkOut>();
    auto xin_depth_config = pipeline.create<dai::node::XLinkIn>();
    auto rgb_node = pipeline.create<dai::node::ColorCamera>();
    auto rgb_encoder_node = pipeline.create<dai::node::VideoEncoder>();
    auto xout_rgb = pipeline.create<dai::node::XLinkOut>();
    auto xout_rgb_compressed = pipeline.create<dai::node::XLinkOut>();
    auto imu_node = pipeline.create<dai::node::IMU>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();
    auto manip_left = pipeline.create<dai::node::ImageManip>();
    auto manip_right = pipeline.create<dai::node::ImageManip>();

    // Configure nodes
    mono_left_node->setCamera("left");
    mono_left_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    mono_left_node->setFps(10);

    mono_right_node->setCamera("right");
    mono_right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    mono_right_node->setFps(10);

    stereo_depth_node->setRectifyEdgeFillColor(0);
    stereo_depth_node->setExtendedDisparity(false);
    stereo_depth_node->setRuntimeModeSwitch(true);
    // Align to right (which becomes left after 180-degree rotation)
    stereo_depth_node->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT);
    stereo_depth_node->initialConfig.set(depth_config_);

    manip_left->initialConfig.setRotationDegrees(180);
    manip_right->initialConfig.setRotationDegrees(180);

    left_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    left_encoder_node->setQuality(80);

    right_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    right_encoder_node->setQuality(80);

    xout_left->setStreamName("left");
    xout_left->input.setQueueSize(1);
    xout_left->input.setBlocking(false);

    xout_left_compressed->setStreamName("left_compressed");
    xout_left_compressed->input.setQueueSize(1);
    xout_left_compressed->input.setBlocking(false);

    xout_right->setStreamName("right");
    xout_right->input.setQueueSize(1);
    xout_right->input.setBlocking(false);

    xout_right_compressed->setStreamName("right_compressed");
    xout_right_compressed->input.setQueueSize(1);
    xout_right_compressed->input.setBlocking(false);

    xout_depth->setStreamName("depth");
    xout_depth->input.setQueueSize(1);
    xout_depth->input.setBlocking(false);

    xin_depth_config->setStreamName("depth_config");

    rgb_node->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    rgb_node->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
    rgb_node->setIspScale(1, 3);
    rgb_node->setVideoSize(1344, 1008);
    rgb_node->setFps(15.0);
    rgb_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

    rgb_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    rgb_encoder_node->setQuality(80);

    xout_rgb->setStreamName("rgb");
    xout_rgb->input.setQueueSize(1);
    xout_rgb->input.setBlocking(false);

    xout_rgb_compressed->setStreamName("rgb_compressed");
    xout_rgb_compressed->input.setQueueSize(1);
    xout_rgb_compressed->input.setBlocking(false);

    imu_node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu_node->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu_node->setBatchReportThreshold(5);
    imu_node->setMaxBatchReports(20);

    xout_imu->setStreamName("imu");

    // Link nodes
    mono_left_node->out.link(stereo_depth_node->left);
    mono_right_node->out.link(stereo_depth_node->right);

    stereo_depth_node->rectifiedLeft.link(manip_right->inputImage);
    stereo_depth_node->rectifiedRight.link(manip_left->inputImage);

    manip_left->out.link(xout_left->input);
    manip_left->out.link(left_encoder_node->input);
    manip_right->out.link(xout_right->input);
    manip_right->out.link(right_encoder_node->input);

    stereo_depth_node->depth.link(xout_depth->input);

    xin_depth_config->out.link(stereo_depth_node->inputConfig);

    left_encoder_node->bitstream.link(xout_left_compressed->input);
    right_encoder_node->bitstream.link(xout_right_compressed->input);

    rgb_node->video.link(xout_rgb->input);
    rgb_node->video.link(rgb_encoder_node->input);
    rgb_encoder_node->bitstream.link(xout_rgb_compressed->input);
    imu_node->out.link(xout_imu->input);

    return pipeline;
  }

  void manage_callback(
    int subscription_count,
    std::shared_ptr<dai::DataOutputQueue> queue,
    int & callback_id,
    std::function<void()> callback)
  {
    bool should_be_active = subscription_count > 0;
    bool is_active = callback_id >= 0;

    if (should_be_active && !is_active) {
      RCLCPP_INFO_STREAM(get_logger(),
          "Activating callback for \"" << queue->getName() << "\" queue");
      callback_id = queue->addCallback(callback);
      queue->tryGetAll(); // Clear any existing data in the queue
    } else if (!should_be_active && is_active) {
      RCLCPP_INFO_STREAM(get_logger(),
          "Deactivating callback for \"" << queue->getName() << "\" queue");
      queue->removeCallback(callback_id);
      callback_id = -1;
    }
  }

  void check_publishers()
  {
    manage_callback(
      rgb_img_pub_->get_subscription_count() + rgb_cam_info_pub_->get_subscription_count(),
      rgb_queue_, rgb_callback_id_,
      std::bind(&OakWrapper::publish_image, this, rgb_img_pub_, rgb_cam_info_pub_,
        rgb_camera_info_, rgb_queue_));

    manage_callback(
      rgb_compressed_pub_->get_subscription_count(),
      rgb_compressed_queue_, rgb_compressed_callback_id_,
      std::bind(&OakWrapper::publish_compressed_image, this, rgb_compressed_pub_,
        "oak_rgb_camera_optical_frame", rgb_compressed_queue_));

    manage_callback(
      left_img_pub_->get_subscription_count() + left_cam_info_pub_->get_subscription_count(),
      left_queue_, left_callback_id_,
      std::bind(&OakWrapper::publish_image, this, left_img_pub_, left_cam_info_pub_,
        left_camera_info_, left_queue_));

    manage_callback(
      left_compressed_pub_->get_subscription_count(),
      left_compressed_queue_, left_compressed_callback_id_,
      std::bind(&OakWrapper::publish_compressed_image, this, left_compressed_pub_,
        "oak_left_camera_optical_frame", left_compressed_queue_));

    manage_callback(
      right_img_pub_->get_subscription_count() + right_cam_info_pub_->get_subscription_count(),
      right_queue_, right_callback_id_,
      std::bind(&OakWrapper::publish_image, this, right_img_pub_, right_cam_info_pub_,
        right_camera_info_, right_queue_));

    manage_callback(
      right_compressed_pub_->get_subscription_count(),
      right_compressed_queue_, right_compressed_callback_id_,
      std::bind(&OakWrapper::publish_compressed_image, this, right_compressed_pub_,
        "oak_right_camera_optical_frame", right_compressed_queue_));

    manage_callback(
      stereo_depth_pub_->get_subscription_count() + stereo_cam_info_pub_->get_subscription_count(),
      depth_queue_, depth_callback_id_,
      std::bind(&OakWrapper::publish_image, this, stereo_depth_pub_, stereo_cam_info_pub_,
        stereo_camera_info_, depth_queue_));

    manage_callback(
      imu_pub_->get_subscription_count(),
      imu_queue_, imu_callback_id_, std::bind(&OakWrapper::publish_imu, this));
  }

  void post_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (auto & param: parameters) {
      RCLCPP_INFO_STREAM(this->get_logger(),
          "Parameter " << param.get_name() << " changed to: " << param.value_to_string());
    }

    update_parameters();
    send_parameters();
  }

  void update_parameters()
  {
    param_listener_.refresh_dynamic_parameters();
    params_ = param_listener_.get_params();

    depth_config_.costMatching.confidenceThreshold = params_.depth.confidence_threshold;
    depth_config_.algorithmControl.enableLeftRightCheck = params_.depth.lr_check_enabled;
    depth_config_.algorithmControl.leftRightCheckThreshold = params_.depth.lr_check_threshold;
    depth_config_.algorithmControl.enableSubpixel = params_.depth.subpixel_enabled;
    depth_config_.algorithmControl.subpixelFractionalBits = params_.depth.subpixel_fractional_bits;
    depth_config_.postProcessing.thresholdFilter.minRange =
      static_cast<int>(params_.depth.min_distance * 1000.0);
    depth_config_.postProcessing.thresholdFilter.maxRange =
      static_cast<int>(params_.depth.max_distance * 1000.0);
  }

  void send_parameters()
  {
    dai::StereoDepthConfig config;
    config.set(depth_config_);
    depth_config_queue_->send(config);
  }

  void publish_image(
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> img_pub,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> cam_info_pub,
    sensor_msgs::msg::CameraInfo cam_info,
    std::shared_ptr<dai::DataOutputQueue> queue)
  {
    auto in_data = queue->tryGet<dai::ImgFrame>();
    if (!in_data) {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to get data from \"" << queue->getName() <<
        "\" queue");
      return;
    }

    cam_info.header.stamp = dai::ros::getFrameTime(
      ros_base_time_, steady_base_time_,
      in_data->getTimestamp());
    cam_info_pub->publish(cam_info);

    auto image = std::make_unique<sensor_msgs::msg::Image>();

    image->header = cam_info.header;
    image->width = in_data->getWidth();
    image->height = in_data->getHeight();
    image->is_bigendian = true;

    if (in_data->getType() == dai::RawImgFrame::Type::NV12) {
      image->encoding = "bgr8";
      image->step = image->width * 3;
      image->data.resize(image->width * image->height * 3);

      cv::Mat in_mat(cv::Size(in_data->getWidth(), in_data->getHeight() * 3 / 2), CV_8UC1,
        in_data->getData().data());
      cv::Mat out_mat(cv::Size(in_data->getWidth(), in_data->getHeight()), CV_8UC3,
        image->data.data());
      cv::cvtColor(in_mat, out_mat, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
    } else if (in_data->getType() == dai::RawImgFrame::Type::RAW8) {
      image->encoding = "mono8";
      image->step = image->width;
      image->data = std::move(in_data->getData());
    } else if (in_data->getType() == dai::RawImgFrame::Type::RAW16) {
      image->encoding = "16UC1";
      image->is_bigendian = false;
      image->step = image->width * 2;
      image->data = std::move(in_data->getData());
    }

    img_pub->publish(std::move(image));
  }

  void publish_compressed_image(
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> img_pub,
    std::string frame_id,
    std::shared_ptr<dai::DataOutputQueue> queue)
  {
    auto in_data = queue->tryGet<dai::ImgFrame>();
    if (!in_data) {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to get data from \"" << queue->getName() <<
        "\" queue");
      return;
    }

    auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
    image->header.stamp = dai::ros::getFrameTime(
      ros_base_time_, steady_base_time_,
      in_data->getTimestamp());
    image->header.frame_id = frame_id;
    image->format = "jpeg";
    image->data = std::move(in_data->getData());

    img_pub->publish(std::move(image));
  }

  void publish_imu()
  {
    auto in_data = imu_queue_->tryGet<dai::IMUData>();
    if (!in_data) {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to get data from \"" << imu_queue_->getName() <<
        "\" queue");
      return;
    }

    std::deque<sensor_msgs::msg::Imu> op_msgs;
    imu_converter_->toRosMsg(in_data, op_msgs);

    while (op_msgs.size()) {
      sensor_msgs::msg::Imu imu = op_msgs.front();
      op_msgs.pop_front();

      imu_pub_->publish(imu);
    }
  }
};


}  // namespace raph_oak

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(raph_oak::OakWrapper)
