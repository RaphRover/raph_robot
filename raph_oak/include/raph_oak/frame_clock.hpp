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

#include <chrono>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace raph_oak
{

/**
 * Converts the host side timestamps DepthAI attaches to its messages (std::chrono::steady_clock)
 * to ROS time, by pairing a ROS time with a steady clock time taken at the same moment.
 */
class FrameClock
{
public:
  FrameClock() : FrameClock(rclcpp::Clock().now(), std::chrono::steady_clock::now()) {}

  FrameClock(
    const rclcpp::Time & ros_base_time, std::chrono::steady_clock::time_point steady_base_time)
  : ros_base_time_(ros_base_time), steady_base_time_(steady_base_time)
  {
  }

  rclcpp::Time to_ros_time(std::chrono::steady_clock::time_point time) const
  {
    return ros_base_time_ + rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              time - steady_base_time_));
  }

private:
  rclcpp::Time ros_base_time_;
  std::chrono::steady_clock::time_point steady_base_time_;
};

}  // namespace raph_oak
