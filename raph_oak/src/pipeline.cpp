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

// DepthAI
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraImageOrientation.hpp"
#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/IMUProperties.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai-shared/properties/StereoDepthProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

// ROS
#include "raph_oak/oak_wrapper_parameters.hpp"
#include "raph_oak/parameters.hpp"

namespace raph_oak
{
dai::Pipeline create_dai_pipeline(const Params & params)
{
  dai::Pipeline pipeline;

  // Create nodes
  auto mono_left_node = pipeline.create<dai::node::MonoCamera>();
  auto mono_right_node = pipeline.create<dai::node::MonoCamera>();
  auto stereo_depth_node = pipeline.create<dai::node::StereoDepth>();
  auto left_encoder_node = pipeline.create<dai::node::VideoEncoder>();
  auto left_rect_encoder_node = pipeline.create<dai::node::VideoEncoder>();
  auto right_encoder_node = pipeline.create<dai::node::VideoEncoder>();
  auto right_rect_encoder_node = pipeline.create<dai::node::VideoEncoder>();
  auto xout_left = pipeline.create<dai::node::XLinkOut>();
  auto xout_left_compressed = pipeline.create<dai::node::XLinkOut>();
  auto xout_left_rect = pipeline.create<dai::node::XLinkOut>();
  auto xout_left_rect_compressed = pipeline.create<dai::node::XLinkOut>();
  auto xout_right = pipeline.create<dai::node::XLinkOut>();
  auto xout_right_compressed = pipeline.create<dai::node::XLinkOut>();
  auto xout_right_rect = pipeline.create<dai::node::XLinkOut>();
  auto xout_right_rect_compressed = pipeline.create<dai::node::XLinkOut>();
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
  auto manip_left_rect = pipeline.create<dai::node::ImageManip>();
  auto manip_right_rect = pipeline.create<dai::node::ImageManip>();

  // Configure nodes
  mono_left_node->setCamera("left");
  mono_left_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  mono_left_node->setFps(params.mono.fps);

  mono_right_node->setCamera("right");
  mono_right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  mono_right_node->setFps(params.mono.fps);

  stereo_depth_node->setRectifyEdgeFillColor(0);
  stereo_depth_node->setExtendedDisparity(false);
  stereo_depth_node->setRuntimeModeSwitch(true);

  // Align to right (which becomes left after 180-degree rotation)
  stereo_depth_node->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT);

  dai::RawStereoDepthConfig depth_initial_config;
  update_depth_config_from_params(depth_initial_config, params);
  stereo_depth_node->initialConfig.set(depth_initial_config);

  manip_left->initialConfig.setRotationDegrees(180);
  manip_right->initialConfig.setRotationDegrees(180);

  manip_left_rect->initialConfig.setRotationDegrees(180);
  manip_right_rect->initialConfig.setRotationDegrees(180);

  left_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  left_encoder_node->setQuality(params.mono_compressed.jpeg_quality);

  right_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  right_encoder_node->setQuality(params.mono_compressed.jpeg_quality);

  left_rect_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  left_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);

  right_rect_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  right_rect_encoder_node->setQuality(params.mono_compressed.jpeg_quality);

  xout_left->setStreamName("left");
  xout_left->input.setQueueSize(1);
  xout_left->input.setBlocking(false);

  xout_left_compressed->setStreamName("left_compressed");
  xout_left_compressed->input.setQueueSize(1);
  xout_left_compressed->input.setBlocking(false);

  xout_left_rect->setStreamName("left_rect");
  xout_left_rect->input.setQueueSize(1);
  xout_left_rect->input.setBlocking(false);

  xout_left_rect_compressed->setStreamName("left_rect_compressed");
  xout_left_rect_compressed->input.setQueueSize(1);
  xout_left_rect_compressed->input.setBlocking(false);

  xout_right->setStreamName("right");
  xout_right->input.setQueueSize(1);
  xout_right->input.setBlocking(false);

  xout_right_compressed->setStreamName("right_compressed");
  xout_right_compressed->input.setQueueSize(1);
  xout_right_compressed->input.setBlocking(false);

  xout_right_rect->setStreamName("right_rect");
  xout_right_rect->input.setQueueSize(1);
  xout_right_rect->input.setBlocking(false);

  xout_right_rect_compressed->setStreamName("right_rect_compressed");
  xout_right_rect_compressed->input.setQueueSize(1);
  xout_right_rect_compressed->input.setBlocking(false);

  xout_depth->setStreamName("depth");
  xout_depth->input.setQueueSize(1);
  xout_depth->input.setBlocking(false);

  xin_depth_config->setStreamName("depth_config");

  rgb_node->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  rgb_node->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
  rgb_node->setIspScale(params.rgb.isp_scale_num, params.rgb.isp_scale_den);
  rgb_node->setVideoSize(params.rgb.width, params.rgb.height);
  rgb_node->setFps(params.rgb.fps);
  rgb_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

  rgb_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  rgb_encoder_node->setQuality(params.rgb_compressed.jpeg_quality);

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
  mono_left_node->out.link(manip_right->inputImage);
  mono_right_node->out.link(stereo_depth_node->right);
  mono_right_node->out.link(manip_left->inputImage);

  stereo_depth_node->rectifiedLeft.link(manip_right_rect->inputImage);
  stereo_depth_node->rectifiedRight.link(manip_left_rect->inputImage);

  manip_left->out.link(xout_left->input);
  manip_left->out.link(left_encoder_node->input);
  manip_right->out.link(xout_right->input);
  manip_right->out.link(right_encoder_node->input);

  manip_left_rect->out.link(xout_left_rect->input);
  manip_left_rect->out.link(left_rect_encoder_node->input);
  manip_right_rect->out.link(xout_right_rect->input);
  manip_right_rect->out.link(right_rect_encoder_node->input);

  stereo_depth_node->depth.link(xout_depth->input);

  xin_depth_config->out.link(stereo_depth_node->inputConfig);

  left_encoder_node->bitstream.link(xout_left_compressed->input);
  right_encoder_node->bitstream.link(xout_right_compressed->input);
  left_rect_encoder_node->bitstream.link(xout_left_rect_compressed->input);
  right_rect_encoder_node->bitstream.link(xout_right_rect_compressed->input);

  rgb_node->video.link(xout_rgb->input);
  rgb_node->video.link(rgb_encoder_node->input);
  rgb_encoder_node->bitstream.link(xout_rgb_compressed->input);
  imu_node->out.link(xout_imu->input);

  return pipeline;
}
}  // namespace raph_oak
