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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "raph_oak/camera_info.hpp"
#include "raph_oak/conversions.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace raph_oak
{
namespace
{

constexpr uint32_t Width = 1280;
constexpr uint32_t Height = 800;
constexpr double Tolerance = 1e-3;

// Numbers of a real OAK-D Pro W: the right stereo sensor, which is the left image after the
// 180 degree rotation, and the left one, which is the right image
constexpr double LeftFx = 572.5621;
constexpr double LeftFy = 572.4834;
constexpr double LeftCx = 636.1994;  // after the rotation
constexpr double LeftCy = 384.0273;
constexpr double RightFx = 573.7007;
constexpr double Baseline = 0.0747430;  // metres

sensor_msgs::msg::CameraInfo make_info(double fx, double fy, double cx, double cy, double tx = 0.0)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = Width;
  info.height = Height;
  info.distortion_model = "rational_polynomial";
  info.d = {1.0, 2.0, 0.3, 0.4, 5.0, 6.0, 7.0, 8.0};
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {0.99996, 0.0005, 0.0092, -0.0005, 0.99999, -0.0051, -0.0092, 0.0051, 0.99994};
  info.p = {fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

std::array<std::array<float, 3>, 3> matrix(double fx, double fy, double cx, double cy)
{
  return {
    {{static_cast<float>(fx), 0.0F, static_cast<float>(cx)},
     {0.0F, static_cast<float>(fy), static_cast<float>(cy)},
     {0.0F, 0.0F, 1.0F}}};
}

TEST(RotatedCameraInfo, FlipsThePrincipalPoint)
{
  const auto rotated = get_rotated_camera_info(make_info(LeftFx, LeftFy, 600.0, 400.0));
  EXPECT_DOUBLE_EQ(rotated.k[2], Width - 1.0 - 600.0);
  EXPECT_DOUBLE_EQ(rotated.k[5], Height - 1.0 - 400.0);
  EXPECT_DOUBLE_EQ(rotated.p[2], Width - 1.0 - 600.0);
  EXPECT_DOUBLE_EQ(rotated.p[6], Height - 1.0 - 400.0);
  EXPECT_DOUBLE_EQ(rotated.k[0], LeftFx);
  EXPECT_DOUBLE_EQ(rotated.k[4], LeftFy);
}

TEST(RotatedCameraInfo, FlipsTheTangentialDistortion)
{
  const auto original = make_info(LeftFx, LeftFy, 600.0, 400.0);
  const auto rotated = get_rotated_camera_info(original);
  EXPECT_DOUBLE_EQ(rotated.d[0], original.d[0]);
  EXPECT_DOUBLE_EQ(rotated.d[1], original.d[1]);
  EXPECT_DOUBLE_EQ(rotated.d[2], -original.d[2]);
  EXPECT_DOUBLE_EQ(rotated.d[3], -original.d[3]);
  EXPECT_DOUBLE_EQ(rotated.d[4], original.d[4]);
}

TEST(RotatedCameraInfo, RectifiedImageHasNoDistortion)
{
  const auto rotated = get_rotated_camera_info(make_info(LeftFx, LeftFy, 600.0, 400.0), true);
  for (const auto coefficient : rotated.d) {
    EXPECT_DOUBLE_EQ(coefficient, 0.0);
  }
}

TEST(RotatedCameraInfo, ConjugatesTheRectificationRotation)
{
  const auto original = make_info(LeftFx, LeftFy, 600.0, 400.0);
  const auto rotated = get_rotated_camera_info(original);
  // R' = D R D with D = diag(-1, -1, 1)
  const std::array<double, 9> sign{1, 1, -1, 1, 1, -1, -1, -1, 1};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(rotated.r[i], sign[i] * original.r[i]) << i;
  }
}

TEST(RotatedCameraInfo, RotatingTwiceGivesTheOriginal)
{
  const auto original = make_info(LeftFx, LeftFy, 600.0, 400.0, -42.88);
  const auto twice = get_rotated_camera_info(get_rotated_camera_info(original));
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_NEAR(twice.k[i], original.k[i], 1e-9) << i;
    EXPECT_NEAR(twice.r[i], original.r[i], 1e-9) << i;
  }
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_NEAR(twice.p[i], original.p[i], 1e-9) << i;
  }
  for (std::size_t i = 0; i < original.d.size(); ++i) {
    EXPECT_NEAR(twice.d[i], original.d[i], 1e-9) << i;
  }
}

TEST(ScaledCameraInfo, HalvesTheIntrinsicsAroundTheCorner)
{
  const auto scaled =
    get_scaled_camera_info(make_info(LeftFx, LeftFy, LeftCx, LeftCy), Width / 2, Height / 2);
  EXPECT_EQ(scaled.width, Width / 2);
  EXPECT_EQ(scaled.height, Height / 2);
  EXPECT_NEAR(scaled.k[0], LeftFx / 2.0, 1e-9);
  EXPECT_NEAR(scaled.k[4], LeftFy / 2.0, 1e-9);
  // Pixel centres are at integer coordinates
  EXPECT_NEAR(scaled.k[2], (LeftCx + 0.5) / 2.0 - 0.5, 1e-9);
  EXPECT_NEAR(scaled.k[5], (LeftCy + 0.5) / 2.0 - 0.5, 1e-9);
  EXPECT_NEAR(scaled.p[2], scaled.k[2], 1e-9);
}

TEST(RectifiedCameraInfo, WithoutTransformationKeepsTheCalibration)
{
  const auto calibration = make_info(RightFx, LeftFy, 625.37, 380.44, -RightFx * Baseline);
  const auto info = get_rectified_camera_info(dai::ImgTransformation(), Width, Height, calibration);
  EXPECT_DOUBLE_EQ(info.k[0], RightFx);
  EXPECT_DOUBLE_EQ(info.k[2], 625.37);
  EXPECT_NEAR(info.p[3], -RightFx * Baseline, 1e-9);
  for (const auto coefficient : info.d) {
    EXPECT_DOUBLE_EQ(coefficient, 0.0);
  }
}

TEST(RectifiedCameraInfo, ImageManipRotationGivesTheRotatedIntrinsics)
{
  // Intrinsics of the unrotated sensor, rotated by 180 degrees like the ImageManip on the camera
  const double sensor_cx = Width - 1.0 - LeftCx;
  const double sensor_cy = Height - 1.0 - LeftCy;
  dai::ImgTransformation transformation(
    Width, Height, matrix(LeftFx, LeftFy, sensor_cx, sensor_cy));
  transformation.addRotation(180.0F, dai::Point2f(Width / 2.0F, Height / 2.0F));

  const auto info =
    get_rectified_camera_info(transformation, Width, Height, make_info(LeftFx, LeftFy, 0.0, 0.0));
  EXPECT_NEAR(info.k[0], LeftFx, Tolerance);
  EXPECT_NEAR(info.k[4], LeftFy, Tolerance);
  EXPECT_NEAR(info.k[2], LeftCx, Tolerance);
  EXPECT_NEAR(info.k[5], LeftCy, Tolerance);
  EXPECT_DOUBLE_EQ(info.k[1], 0.0);
  EXPECT_DOUBLE_EQ(info.k[3], 0.0);
}

TEST(RectifiedCameraInfo, SensorRotationFlipsThePrincipalPoint)
{
  const dai::ImgTransformation transformation(Width, Height, matrix(LeftFx, LeftFy, 600.0, 400.0));
  const auto info = get_rectified_camera_info(
    transformation, Width, Height, make_info(LeftFx, LeftFy, 0.0, 0.0), true);
  EXPECT_NEAR(info.k[2], Width - 1.0 - 600.0, Tolerance);
  EXPECT_NEAR(info.k[5], Height - 1.0 - 400.0, Tolerance);
}

TEST(RectifiedCameraInfo, KeepsTheBaselineInMetres)
{
  // Both rectified images get the intrinsics of one sensor, the baseline follows its focal length
  const auto calibration = make_info(RightFx, 573.72, 625.37, 380.44, -RightFx * Baseline);
  const dai::ImgTransformation transformation(Width, Height, matrix(LeftFx, LeftFy, 600.0, 400.0));
  const auto info = get_rectified_camera_info(transformation, Width, Height, calibration);
  EXPECT_NEAR(info.p[0], LeftFx, Tolerance);
  EXPECT_NEAR(info.p[3] / info.p[0], -Baseline, 1e-6);
  EXPECT_NEAR(info.p[3], -LeftFx * Baseline, Tolerance);
}

TEST(RectifiedCameraInfo, KeepsTheRectificationRotation)
{
  const auto calibration = make_info(LeftFx, LeftFy, 600.0, 400.0);
  const dai::ImgTransformation transformation(Width, Height, matrix(LeftFx, LeftFy, 600.0, 400.0));
  const auto info = get_rectified_camera_info(transformation, Width, Height, calibration);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(info.r[i], calibration.r[i]) << i;
  }
}

class CalibrationToCameraInfo : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::vector<std::vector<float>> identity{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
      calibration_.setCameraIntrinsics(
        socket, {{570.0F, 0.0F, 640.0F}, {0.0F, 571.0F, 400.0F}, {0.0F, 0.0F, 1.0F}}, 1280, 800);
      calibration_.setDistortionCoefficients(
        socket, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14});
    }
    // Translations in centimetres
    calibration_.setCameraExtrinsics(
      dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, identity, {-7.5F, 0.0F, 0.0F});
    calibration_.setStereoLeft(dai::CameraBoardSocket::CAM_B, left_rectification_);
    calibration_.setStereoRight(dai::CameraBoardSocket::CAM_C, right_rectification_);
  }

  dai::CalibrationHandler calibration_;
  std::vector<std::vector<float>> left_rectification_{
    {1.0F, 0.01F, 0.0F}, {-0.01F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}};
  std::vector<std::vector<float>> right_rectification_{
    {1.0F, 0.0F, 0.02F}, {0.0F, 1.0F, 0.0F}, {-0.02F, 0.0F, 1.0F}};
};

TEST_F(CalibrationToCameraInfo, IntrinsicsAndDistortion)
{
  const auto info =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_C, 1280, 800);
  EXPECT_EQ(info.width, 1280U);
  EXPECT_EQ(info.height, 800U);
  EXPECT_EQ(info.distortion_model, "rational_polynomial");
  EXPECT_NEAR(info.k[0], 570.0, Tolerance);
  EXPECT_NEAR(info.k[2], 640.0, Tolerance);
  EXPECT_NEAR(info.k[4], 571.0, Tolerance);
  EXPECT_NEAR(info.k[5], 400.0, Tolerance);
  ASSERT_EQ(info.d.size(), 8U);
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_DOUBLE_EQ(info.d[i], static_cast<double>(i + 1)) << i;
  }
}

TEST_F(CalibrationToCameraInfo, FirstCameraOfThePairHasTheBaseline)
{
  const auto left =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_B, 1280, 800);
  const auto right =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_C, 1280, 800);
  EXPECT_NEAR(left.p[3], 570.0 * -0.075, Tolerance);
  EXPECT_DOUBLE_EQ(right.p[3], 0.0);
}

TEST_F(CalibrationToCameraInfo, StereoCamerasHaveTheirRectificationRotation)
{
  const auto left =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_B, 1280, 800);
  const auto right =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_C, 1280, 800);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_NEAR(left.r[i], left_rectification_[i / 3][i % 3], 1e-6) << i;
    EXPECT_NEAR(right.r[i], right_rectification_[i / 3][i % 3], 1e-6) << i;
  }
}

TEST_F(CalibrationToCameraInfo, ScalesToTheRequestedResolution)
{
  const auto info =
    calibration_to_camera_info(calibration_, dai::CameraBoardSocket::CAM_C, 640, 400);
  EXPECT_NEAR(info.k[0], 285.0, Tolerance);
  EXPECT_NEAR(info.k[4], 285.5, Tolerance);
}

}  // namespace
}  // namespace raph_oak
