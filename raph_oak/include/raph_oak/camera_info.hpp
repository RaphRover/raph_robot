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

#include <cstdint>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace raph_oak
{

// Camera info of a camera from its factory calibration, at the given resolution: the intrinsics
// and the first 8 distortion coefficients (rational polynomial model). For the cameras of the
// stereo pair also the rectification rotation, and for the left one of the calibration (the
// first camera of the pair) the baseline in P, as in depthai_bridge.
sensor_msgs::msg::CameraInfo calibration_to_camera_info(
  const dai::CalibrationHandler & calibration, dai::CameraBoardSocket socket, uint32_t width,
  uint32_t height);

// Calculate the camera info for the images rotated by 180 degrees
sensor_msgs::msg::CameraInfo get_rotated_camera_info(
  const sensor_msgs::msg::CameraInfo & original_info, bool is_rectified = false);

// Camera info of a rectified (or undistorted) image from the intrinsic matrix DepthAI attached to
// the frame, which includes the crop, resize and any rotation done by an ImageManip. DepthAI
// describes a 180 degree rotation by negating the focal lengths and measures the flipped principal
// point from w instead of w - 1; both are converted to the convention of the other camera infos
// (positive focal lengths, pixel centres at integer coordinates). A rotation done by the sensor
// itself is not part of the transformation, rotated_at_sensor flips the principal point for it.
// The rectification rotation and the baseline are taken from calibration_info (a camera info that
// was already rotated with get_rotated_camera_info); the baseline is kept in metres, so the Tx term
// of P follows the new focal length. If the frame carries no valid transformation, calibration_info
// is used as is, with its distortion removed. An image that was resized but not undistorted
// (undistorted = false) keeps the distortion of calibration_info.
sensor_msgs::msg::CameraInfo get_rectified_camera_info(
  const dai::ImgTransformation & transformation, uint32_t width, uint32_t height,
  const sensor_msgs::msg::CameraInfo & calibration_info, bool rotated_at_sensor = false,
  bool undistorted = true);

// Camera info of the same camera at a different resolution
sensor_msgs::msg::CameraInfo get_scaled_camera_info(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height);

}  // namespace raph_oak
