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

#include <memory>
#include <utility>

#include "raph_oak/nn_overlay.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace raph_oak
{
namespace
{
sensor_msgs::msg::Image make_image()
{
  sensor_msgs::msg::Image image;
  image.width = 64;
  image.height = 32;
  image.step = 192;
  image.encoding = "bgr8";
  image.data.assign(64 * 32 * 3, 0);
  return image;
}

NnResult make_classification()
{
  vision_msgs::msg::Classification classification;
  vision_msgs::msg::ObjectHypothesis result;
  result.class_id = "cat";
  result.score = 0.9;
  classification.results.push_back(result);
  return classification;
}
}  // namespace

TEST(NnOverlay, ImageFirstWaitsForTheResult)
{
  NnOverlay overlay;
  EXPECT_EQ(overlay.set_image(make_image(), 7), nullptr);
  const auto result = overlay.set_result(make_classification(), 7);
  ASSERT_NE(result, nullptr);
  // The text was drawn on it
  EXPECT_NE(result->data, make_image().data);
}

TEST(NnOverlay, ResultFirstWaitsForTheImage)
{
  NnOverlay overlay;
  EXPECT_EQ(overlay.set_result(make_classification(), 3), nullptr);
  EXPECT_NE(overlay.set_image(make_image(), 3), nullptr);
}

TEST(NnOverlay, DifferentSequencesDoNotPair)
{
  NnOverlay overlay;
  EXPECT_EQ(overlay.set_image(make_image(), 1), nullptr);
  EXPECT_EQ(overlay.set_result(make_classification(), 2), nullptr);
  // The newer image replaces the waiting one, and then matches the waiting result
  EXPECT_NE(overlay.set_image(make_image(), 2), nullptr);
}

TEST(NnOverlay, AFinishedPairIsNotUsedTwice)
{
  NnOverlay overlay;
  overlay.set_image(make_image(), 5);
  ASSERT_NE(overlay.set_result(make_classification(), 5), nullptr);
  EXPECT_EQ(overlay.set_result(make_classification(), 5), nullptr);
}

TEST(NnOverlay, AnInstanceResultPairsLikeTheOthers)
{
  NnOverlay overlay;
  EXPECT_EQ(overlay.set_image(make_image(), 9), nullptr);

  InstanceResult instances;
  instances.mask.width = 1;
  instances.mask.height = 1;
  instances.mask.step = 1;
  instances.mask.encoding = "mono8";
  instances.mask.data = {0};
  const auto result = overlay.set_result(instances, 9);
  ASSERT_NE(result, nullptr);
  // The mask was blended over the frame
  EXPECT_NE(result->data, make_image().data);
}

}  // namespace raph_oak
