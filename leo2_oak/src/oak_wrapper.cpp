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
#include "image_transport/image_transport.hpp"

namespace leo2_oak
{

void publish_image(
  image_transport::CameraPublisher & cam_pub,
  sensor_msgs::msg::CameraInfo cam_info,
  std::shared_ptr<dai::rosBridge::ImageConverter> & cam_converter,
  std::shared_ptr<dai::ADatatype> data)
{
  if (cam_pub.getNumSubscribers() == 0) {return;}

  auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(data);

  std::deque<sensor_msgs::msg::Image> op_msgs;
  cam_converter->toRosMsg(in_data, op_msgs);

  while (op_msgs.size()) {
    sensor_msgs::msg::Image image = op_msgs.front();
    op_msgs.pop_front();

    cam_info.header.stamp = image.header.stamp;

    cam_pub.publish(image, cam_info);
  }
}

class OakWrapper : public rclcpp::Node
{
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::rosBridge::ImageConverter> rgb_converter_;
  std::shared_ptr<dai::DataOutputQueue> rgb_queue_;
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  image_transport::CameraPublisher rgb_pub_;

  std::shared_ptr<dai::rosBridge::ImageConverter> left_converter_;
  std::shared_ptr<dai::DataOutputQueue> left_queue_;
  sensor_msgs::msg::CameraInfo left_camera_info_;
  image_transport::CameraPublisher left_pub_;

  std::shared_ptr<dai::rosBridge::ImageConverter> right_converter_;
  std::shared_ptr<dai::DataOutputQueue> right_queue_;
  sensor_msgs::msg::CameraInfo right_camera_info_;
  image_transport::CameraPublisher right_pub_;

  std::shared_ptr<dai::rosBridge::ImageConverter> depth_converter_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
  image_transport::CameraPublisher depth_pub_;

  std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;
  std::shared_ptr<dai::DataOutputQueue> imu_queue_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::shared_ptr<dai::DataOutputQueue> rgb_compressed_queue_;

public:
  OakWrapper(rclcpp::NodeOptions options)
  : Node("oak_wrapper")
  {
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

    left_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(
      "oak_left_camera_optical_frame", true);
    left_queue_ = device_->getOutputQueue("left", 30, false);
    left_camera_info_ = left_converter_->calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::LEFT, 1280, 720);
    left_camera_info_.header.frame_id = "oak_left_camera_optical_frame";

    left_pub_ = image_transport::create_camera_publisher(this, "left/image_rect");
    left_queue_->addCallback(
      std::bind(
        publish_image, left_pub_, left_camera_info_, left_converter_, std::placeholders::_1));

    right_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(
      "oak_right_camera_optical_frame", true);
    right_queue_ = device_->getOutputQueue("right", 30, false);
    right_camera_info_ = right_converter_->calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::RIGHT, 1280, 720);
    right_camera_info_.header.frame_id = "oak_right_camera_optical_frame";

    right_pub_ = image_transport::create_camera_publisher(this, "right/image_rect");
    right_queue_->addCallback(
      std::bind(
        publish_image, right_pub_, right_camera_info_, right_converter_, std::placeholders::_1));

    depth_queue_ = device_->getOutputQueue("depth", 30, false);
    depth_pub_ = image_transport::create_camera_publisher(this, "stereo/depth");
    depth_queue_->addCallback(
      std::bind(
        publish_image, depth_pub_, right_camera_info_,
        right_converter_, std::placeholders::_1));

    rgb_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(
      "oak_rgb_camera_optical_frame", false);
    rgb_queue_ = device_->getOutputQueue("rgb", 30, false);
    rgb_camera_info_ = rgb_converter_->calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::RGB, 1280, 720);
    rgb_camera_info_.header.frame_id = "oak_rgb_camera_optical_frame";

    rgb_pub_ = image_transport::create_camera_publisher(this, "rgb/image_color");
    rgb_queue_->addCallback(
      std::bind(
        publish_image, rgb_pub_, rgb_camera_info_, rgb_converter_, std::placeholders::_1));

    imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>(
      "oak_imu_frame",
      dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO);
    imu_queue_ = device_->getOutputQueue("imu", 30, false);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_queue_->addCallback(std::bind(&OakWrapper::publish_imu, this, std::placeholders::_1));

    rgb_compressed_queue_ = device_->getOutputQueue("rgb_compressed", 30, false);
    rgb_compressed_queue_->addCallback(
      std::bind(
        &OakWrapper::publish_rgb_compressed, this,
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
    rgb_node->setInterleaved(false);
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

  void publish_rgb_compressed(std::shared_ptr<dai::ADatatype> data)
  {
    static int cnt = 0;
    RCLCPP_INFO_STREAM(get_logger(), "RGB-COMPRESSED callback " << cnt++);

    auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(data);

    RCLCPP_INFO_STREAM(get_logger(), (int)in_data->getType());
  }

  void publish_imu(std::shared_ptr<dai::ADatatype> data)
  {
    static int cnt = 0;
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "IMU callback " << cnt++ << ". Subscribers: " << rgb_pub_.getNumSubscribers());

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
