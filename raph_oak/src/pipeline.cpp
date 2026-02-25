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

#include "depthai/depthai.hpp"

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
  mono_left_node->setFps(kMonoFps);

  mono_right_node->setCamera("right");
  mono_right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  mono_right_node->setFps(kMonoFps);

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

  left_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  left_encoder_node->setQuality(kMonoCompressedQuality);

  right_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  right_encoder_node->setQuality(kMonoCompressedQuality);

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
  rgb_node->setIspScale(kRgbIspScaleNumerator, kRgbIspScaleDenominator);
  rgb_node->setVideoSize(kRgbWidth, kRgbHeight);
  rgb_node->setFps(kRgbFps);
  rgb_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  rgb_node->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

  rgb_encoder_node->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
  rgb_encoder_node->setQuality(kRgbCompressedQuality);

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
}   // namespace raph_oak
