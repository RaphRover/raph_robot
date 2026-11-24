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

#include "raph_oak/camera_info.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace raph_oak
{

sensor_msgs::msg::CameraInfo calibration_to_camera_info(
  const dai::CalibrationHandler & calibration, dai::CameraBoardSocket socket, uint32_t width,
  uint32_t height)
{
  constexpr std::size_t kDistortionCoefficientCount = 8;
  // DepthAI stores translations in centimetres
  constexpr double kMetresPerCentimetre = 0.01;

  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "rational_polynomial";

  const auto intrinsics =
    calibration.getCameraIntrinsics(socket, static_cast<int>(width), static_cast<int>(height));
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      info.k[row * 3 + col] = intrinsics[row][col];
      info.p[row * 4 + col] = intrinsics[row][col];
    }
  }
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  const auto distortion = calibration.getDistortionCoefficients(socket);
  info.d.assign(kDistortionCoefficientCount, 0.0);
  for (std::size_t i = 0; i < kDistortionCoefficientCount && i < distortion.size(); ++i) {
    info.d[i] = distortion[i];
  }

  const auto left_socket = calibration.getStereoLeftCameraId();
  const auto right_socket = calibration.getStereoRightCameraId();
  if (
    left_socket == dai::CameraBoardSocket::AUTO || right_socket == dai::CameraBoardSocket::AUTO ||
    (socket != left_socket && socket != right_socket)) {
    return info;
  }

  std::vector<std::vector<float>> rectification;
  if (socket == left_socket) {
    // Projection into the first camera of the pair, seen from the second one
    info.p[3] = info.k[0] * calibration.getCameraExtrinsics(left_socket, right_socket)[0][3] *
                kMetresPerCentimetre;
    rectification = calibration.getStereoLeftRectificationRotation();
  } else {
    rectification = calibration.getStereoRightRectificationRotation();
  }
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      info.r[row * 3 + col] = rectification[row][col];
    }
  }
  return info;
}

sensor_msgs::msg::CameraInfo get_rotated_camera_info(
  const sensor_msgs::msg::CameraInfo & original_info, bool is_rectified)
{
  sensor_msgs::msg::CameraInfo rotated_info = original_info;
  const double width = static_cast<double>(original_info.width);
  const double height = static_cast<double>(original_info.height);

  // 1. Shift Principal Point in K (Intrinsics)
  // K[2] is cx, K[5] is cy
  rotated_info.k[2] = width - 1.0 - original_info.k[2];
  rotated_info.k[5] = height - 1.0 - original_info.k[5];

  // 2. Shift Principal Point in P (Projection)
  // P[2] is cx, P[6] is cy
  rotated_info.p[2] = width - 1.0 - original_info.p[2];
  rotated_info.p[6] = height - 1.0 - original_info.p[6];

  // 3. Conjugate the rectification rotation R with the 180 degree rotation about the optical axis,
  // R' = D R D with D = diag(-1, -1, 1): the terms coupling x or y with z change sign
  rotated_info.r[2] = -original_info.r[2];
  rotated_info.r[5] = -original_info.r[5];
  rotated_info.r[6] = -original_info.r[6];
  rotated_info.r[7] = -original_info.r[7];

  // 4. Handle Distortion (D)
  if (!is_rectified) {
    // If NOT rectified, flip the tangential distortion coefficients (p1, p2) in D,
    // and keep the radial coefficients (k1, k2, k3, k4) the same.
    if (rotated_info.d.size() >= 4) {
      rotated_info.d[2] = -original_info.d[2];  // p1
      rotated_info.d[3] = -original_info.d[3];  // p2
    }
  } else {
    // If ALREADY rectified, D should be zeros. Ensure it stays that way.
    std::fill(rotated_info.d.begin(), rotated_info.d.end(), 0.0);
  }

  return rotated_info;
}

sensor_msgs::msg::CameraInfo get_rectified_camera_info(
  const dai::ImgTransformation & transformation, uint32_t width, uint32_t height,
  const sensor_msgs::msg::CameraInfo & calibration_info, bool rotated_at_sensor)
{
  sensor_msgs::msg::CameraInfo info = calibration_info;
  info.width = width;
  info.height = height;
  std::fill(info.d.begin(), info.d.end(), 0.0);

  // Baseline in metres: P[3] = fx * Tx
  const double translation_x =
    calibration_info.p[0] != 0.0 ? calibration_info.p[3] / calibration_info.p[0] : 0.0;

  if (transformation.isValid()) {
    const auto matrix = transformation.getIntrinsicMatrix();
    const double focal_x = std::abs(static_cast<double>(matrix[0][0]));
    const double focal_y = std::abs(static_cast<double>(matrix[1][1]));
    double principal_x = matrix[0][2];
    double principal_y = matrix[1][2];

    if (matrix[0][0] < 0.0) {
      // Rotated by 180 degrees on the camera: DepthAI negates the focal lengths and flips the
      // principal point as w - cx, the pixel centre convention needs w - 1 - cx
      principal_x -= 1.0;
      principal_y -= 1.0;
    } else if (rotated_at_sensor) {
      principal_x = static_cast<double>(width) - 1.0 - principal_x;
      principal_y = static_cast<double>(height) - 1.0 - principal_y;
    }

    // The skew of a 180 degree rotation is only float rounding noise
    info.k = {focal_x, 0.0, principal_x, 0.0, focal_y, principal_y, 0.0, 0.0, 1.0};
  }

  // Rectified image: the projection matrix is the intrinsic matrix, with the baseline for the
  // second camera of a stereo pair
  info.p = {info.k[0], info.k[1], info.k[2], info.k[0] * translation_x,
            info.k[3], info.k[4], info.k[5], 0.0,
            info.k[6], info.k[7], info.k[8], 0.0};
  return info;
}

sensor_msgs::msg::CameraInfo get_scaled_camera_info(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height)
{
  sensor_msgs::msg::CameraInfo scaled = info;
  const double scale_x = static_cast<double>(width) / static_cast<double>(info.width);
  const double scale_y = static_cast<double>(height) / static_cast<double>(info.height);
  scaled.width = width;
  scaled.height = height;

  // Pixel centres are at integer coordinates, so the principal point is scaled around the corner
  scaled.k[0] = info.k[0] * scale_x;
  scaled.k[2] = (info.k[2] + 0.5) * scale_x - 0.5;
  scaled.k[4] = info.k[4] * scale_y;
  scaled.k[5] = (info.k[5] + 0.5) * scale_y - 0.5;

  scaled.p[0] = info.p[0] * scale_x;
  scaled.p[2] = (info.p[2] + 0.5) * scale_x - 0.5;
  scaled.p[3] = info.p[3] * scale_x;
  scaled.p[5] = info.p[5] * scale_y;
  scaled.p[6] = (info.p[6] + 0.5) * scale_y - 0.5;
  return scaled;
}

}  // namespace raph_oak
