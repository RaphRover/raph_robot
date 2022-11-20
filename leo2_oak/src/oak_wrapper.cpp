// Copyright 2022 Kell Ideas sp. z o.o.
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

// #include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include <depthai_bridge/ImuConverter.hpp>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"

namespace leo2_oak
{

class OakWrapper : public rclcpp::Node
{
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::DataOutputQueue> rgb_queue_;
  std::shared_ptr<dai::DataOutputQueue> left_queue_;
  std::shared_ptr<dai::DataOutputQueue> right_queue_;

  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
  // image_transport::CameraPublisher depth_pub_;

  std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;
  std::shared_ptr<dai::DataOutputQueue> imu_queue_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_compressed_pub_;
  std::shared_ptr<dai::DataOutputQueue> rgb_compressed_queue_;

  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  rclcpp::Time ros_base_time_;

public:
  OakWrapper(rclcpp::NodeOptions options)
  : Node("oak_wrapper"),
    steady_base_time_(std::chrono::steady_clock::now())
  {
    ros_base_time_ = rclcpp::Clock().now();

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

    // Only used to get camera info matrices
    auto img_converter = dai::rosBridge::ImageConverter(false);

    rgb_queue_ = device_->getOutputQueue("rgb", 30, false);
    auto rgb_camera_info = img_converter.calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::RGB, 1280, 720);
    rgb_camera_info.header.frame_id = "oak_rgb_camera_optical_frame";

    auto rgb_img_pub = create_publisher<sensor_msgs::msg::Image>("rgb/image_color", 10);
    auto rgb_cam_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
    rgb_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_image, this, rgb_img_pub, rgb_cam_info_pub, rgb_camera_info,
        std::placeholders::_1));

    left_queue_ = device_->getOutputQueue("left", 30, false);
    auto left_camera_info = img_converter.calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::LEFT, 1280, 720);
    left_camera_info.header.frame_id = "oak_left_camera_optical_frame";

    auto left_img_pub = create_publisher<sensor_msgs::msg::Image>("left/image_mono", 10);
    auto left_cam_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
    left_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_image, this, left_img_pub, left_cam_info_pub, left_camera_info,
        std::placeholders::_1));

    right_queue_ = device_->getOutputQueue("right", 30, false);
    auto right_camera_info = img_converter.calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::RIGHT, 1280, 720);
    right_camera_info.header.frame_id = "oak_right_camera_optical_frame";

    auto right_img_pub = create_publisher<sensor_msgs::msg::Image>("right/image_mono", 10);
    auto right_cam_info_pub =
      create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
    right_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_image, this, right_img_pub, right_cam_info_pub, right_camera_info,
        std::placeholders::_1));

    depth_queue_ = device_->getOutputQueue("depth", 30, false);

    auto depth_img_pub = create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    auto stereo_cam_info_pub =
      create_publisher<sensor_msgs::msg::CameraInfo>("stereo/camera_info", 10);
    depth_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_image, this, depth_img_pub, stereo_cam_info_pub, right_camera_info,
        std::placeholders::_1
      )
    );

    imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>(
      "oak_imu_frame",
      dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO);
    imu_queue_ = device_->getOutputQueue("imu", 30, false);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_queue_->addCallback(std::bind(&OakWrapper::publish_imu, this, std::placeholders::_1));

    auto rgb_compressed_pub = create_publisher<sensor_msgs::msg::CompressedImage>(
      "rgb/image_color/compressed", 10);
    rgb_compressed_queue_ = device_->getOutputQueue("rgb_compressed", 30, false);
    rgb_compressed_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_compressed_image, this, rgb_compressed_pub,
        "oak_rgb_camera_optical_frame",
        std::placeholders::_1));
  }

private:
  dai::Pipeline create_dai_pipeline()
  {
    dai::Pipeline pipeline;

    // Create nodes
    auto mono_left_node = pipeline.create<dai::node::MonoCamera>();
    auto mono_right_node = pipeline.create<dai::node::MonoCamera>();
    auto stereo_depth_node = pipeline.create<dai::node::StereoDepth>();
    auto xout_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_right = pipeline.create<dai::node::XLinkOut>();
    auto xout_depth = pipeline.create<dai::node::XLinkOut>();
    auto rgb_node = pipeline.create<dai::node::ColorCamera>();
    auto rgb_encoder_node = pipeline.create<dai::node::VideoEncoder>();
    auto xout_rgb = pipeline.create<dai::node::XLinkOut>();
    auto xout_rgb_compressed = pipeline.create<dai::node::XLinkOut>();
    auto imu_node = pipeline.create<dai::node::IMU>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();

    // Configure nodes
    mono_left_node->setBoardSocket(dai::CameraBoardSocket::LEFT);
    mono_left_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_left_node->setFps(30);
    mono_right_node->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    mono_right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_right_node->setFps(30);

    stereo_depth_node->setRectifyEdgeFillColor(0);
    stereo_depth_node->initialConfig.setConfidenceThreshold(100);
    stereo_depth_node->setLeftRightCheck(true);
    stereo_depth_node->initialConfig.setLeftRightCheckThreshold(5);
    stereo_depth_node->setExtendedDisparity(false);
    stereo_depth_node->setSubpixel(false);

    xout_left->setStreamName("left");
    xout_left->input.setQueueSize(1);
    xout_left->input.setBlocking(false);

    xout_right->setStreamName("right");
    xout_right->input.setQueueSize(1);
    xout_right->input.setBlocking(false);

    xout_depth->setStreamName("depth");
    xout_depth->input.setQueueSize(1);
    xout_depth->input.setBlocking(false);

    rgb_node->setBoardSocket(dai::CameraBoardSocket::RGB);
    rgb_node->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    rgb_node->setIspScale(1, 3);
    rgb_node->setVideoSize(1280, 720);
    rgb_node->setFps(30.0);
    rgb_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

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

    stereo_depth_node->rectifiedLeft.link(xout_left->input);
    stereo_depth_node->rectifiedRight.link(xout_right->input);
    stereo_depth_node->depth.link(xout_depth->input);

    rgb_node->video.link(xout_rgb->input);
    rgb_node->video.link(rgb_encoder_node->input);
    rgb_encoder_node->bitstream.link(xout_rgb_compressed->input);
    imu_node->out.link(xout_imu->input);

    return pipeline;
  }

  void publish_image(
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> img_pub,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> cam_info_pub,
    sensor_msgs::msg::CameraInfo cam_info,
    std::shared_ptr<dai::ADatatype> data)
  {
    auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(data);

    cam_info.header.stamp = dai::ros::getFrameTime(
      ros_base_time_, steady_base_time_,
      in_data->getTimestamp());
    cam_info_pub->publish(cam_info);

    if (img_pub->get_subscription_count() == 0) {return;}

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
    std::shared_ptr<dai::ADatatype> data)
  {
    if (img_pub->get_subscription_count() == 0) {return;}

    auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(data);

    auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
    image->header.stamp = dai::ros::getFrameTime(
      ros_base_time_, steady_base_time_,
      in_data->getTimestamp());
    image->header.frame_id = frame_id;
    image->format = "jpeg";
    image->data = std::move(in_data->getData());

    img_pub->publish(std::move(image));
  }

  void publish_imu(std::shared_ptr<dai::ADatatype> data)
  {
    if (imu_pub_->get_subscription_count() == 0) {return;}

    auto in_data = std::dynamic_pointer_cast<dai::IMUData>(data);

    std::deque<sensor_msgs::msg::Imu> op_msgs;
    imu_converter_->toRosMsg(in_data, op_msgs);

    while (op_msgs.size()) {
      sensor_msgs::msg::Imu imu = op_msgs.front();
      op_msgs.pop_front();

      imu_pub_->publish(imu);
    }
  }
};


}  // namespace leo2_oak

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo2_oak::OakWrapper)
