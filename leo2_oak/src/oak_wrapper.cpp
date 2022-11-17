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

class OakWrapper : public rclcpp::Node
{
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::rosBridge::ImageConverter> rgb_converter_;
  std::shared_ptr<dai::DataOutputQueue> rgb_queue_;
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  image_transport::CameraPublisher rgb_pub_; 

  std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;
  std::shared_ptr<dai::DataOutputQueue> imu_queue_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

public:
  OakWrapper(rclcpp::NodeOptions options)
  : Node("oak_wrapper")
  {
    auto pipeline = create_dai_pipeline();
    device_ = std::make_unique<dai::Device>(pipeline, dai::UsbSpeed::SUPER_PLUS);

    auto calibration_handler = device_->readCalibration();
    auto eeprom = calibration_handler.getEepromData();

    RCLCPP_INFO_STREAM(get_logger(), "Product name: " << eeprom.productName);
    RCLCPP_INFO_STREAM(get_logger(), "Board custom: " << eeprom.boardCustom);
    RCLCPP_INFO_STREAM(get_logger(), "Board name: " << eeprom.boardName);
    RCLCPP_INFO_STREAM(get_logger(), "Board Rev: " << eeprom.boardRev);
    RCLCPP_INFO_STREAM(get_logger(), "Board Conf: " << eeprom.boardConf);
    RCLCPP_INFO_STREAM(get_logger(), "Hardware Conf: " << eeprom.hardwareConf);
    RCLCPP_INFO_STREAM(get_logger(), "Batch name: " << eeprom.batchName);

    rgb_converter_ = std::make_shared<dai::rosBridge::ImageConverter>("oak_rgb_camera_optical_frame", true);
    rgb_queue_ = device_->getOutputQueue("rgb", 30, false);
    rgb_camera_info_ = rgb_converter_->calibrationToCameraInfo(
      calibration_handler, dai::CameraBoardSocket::RGB, 1920, 1080);
    
    rgb_pub_ = image_transport::create_camera_publisher(this, "rgb/image_color");
    rgb_queue_->addCallback(std::bind(&OakWrapper::publish_rgb, this, std::placeholders::_1));

    imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>("oak_imu_frame", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO);
    imu_queue_ = device_->getOutputQueue("imu", 30, false);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_queue_->addCallback(std::bind(&OakWrapper::publish_imu, this, std::placeholders::_1));
  }

private:
  dai::Pipeline create_dai_pipeline()
  {
    dai::Pipeline pipeline;

    // Create nodes
    // auto mono_left_node = pipeline.create<dai::node::MonoCamera>();
    // auto mono_right_node = pipeline.create<dai::node::MonoCamera>();
    // auto stereo_depth_node = pipeline.create<dai::node::StereoDepth>();
    // auto xout_left = pipeline.create<dai::node::XLinkOut>();
    // auto xout_right = pipeline.create<dai::node::XLinkOut>();
    // auto xout_depth = pipeline.create<dai::node::XLinkOut>();
    auto rgb_node = pipeline.create<dai::node::ColorCamera>();
    auto xout_rgb = pipeline.create<dai::node::XLinkOut>();
    auto imu_node = pipeline.create<dai::node::IMU>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();

    // Configure nodes
    rgb_node->setBoardSocket(dai::CameraBoardSocket::RGB);
    rgb_node->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    rgb_node->setVideoSize(1920, 1080);
    rgb_node->setInterleaved(false);
    rgb_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    xout_rgb->setStreamName("rgb");
    xout_rgb->input.setQueueSize(1);
    xout_rgb->input.setBlocking(false);

    imu_node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu_node->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu_node->setBatchReportThreshold(5);
    imu_node->setMaxBatchReports(20);

    xout_imu->setStreamName("imu");

    // Link nodes
    rgb_node->video.link(xout_rgb->input);
    imu_node->out.link(xout_imu->input);

    return pipeline;
  }

  void publish_rgb(std::shared_ptr<dai::ADatatype> data) {
    static int cnt = 0;
    RCLCPP_DEBUG_STREAM(get_logger(), "RGB callback " << cnt++ << ". Subscribers: " << rgb_pub_.getNumSubscribers());

    if (rgb_pub_.getNumSubscribers() == 0) return;

    auto in_data = std::dynamic_pointer_cast<dai::ImgFrame>(data);

    std::deque<sensor_msgs::msg::Image> op_msgs;
    rgb_converter_->toRosMsg(in_data, op_msgs);

    while(op_msgs.size()) {
      sensor_msgs::msg::Image image = op_msgs.front();
      op_msgs.pop_front();

      sensor_msgs::msg::CameraInfo cam_info = rgb_camera_info_;
      cam_info.header.stamp = image.header.stamp;

      rgb_pub_.publish(image, rgb_camera_info_);
    }
  }

  void publish_imu(std::shared_ptr<dai::ADatatype> data) {
    static int cnt = 0;
    RCLCPP_DEBUG_STREAM(get_logger(), "IMU callback " << cnt++ << ". Subscribers: " << rgb_pub_.getNumSubscribers());

    if (imu_pub_->get_subscription_count() == 0) return;

    auto in_data = std::dynamic_pointer_cast<dai::IMUData>(data);

    std::deque<sensor_msgs::msg::Imu> op_msgs;
    imu_converter_->toRosMsg(in_data, op_msgs);

    while(op_msgs.size()) {
      sensor_msgs::msg::Imu imu = op_msgs.front();
      op_msgs.pop_front();

      imu_pub_->publish(imu);
    }
  }
};


}  // namespace leo2_oak

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo2_oak::OakWrapper)
