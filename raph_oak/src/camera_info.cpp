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

#include "sensor_msgs/msg/camera_info.hpp"

namespace raph_oak
{

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

  // 3. Handle Distortion (D)
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

}  // namespace raph_oak
