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

#include <stdexcept>

#include "raph_oak/nn_model.hpp"

namespace raph_oak
{

TEST(NnModel, YoloAndSsdHeadsAreDetection)
{
  EXPECT_EQ(get_nn_kind({"YOLO"}), NnKind::DETECTION);
  EXPECT_EQ(get_nn_kind({"YOLOExtendedParser"}), NnKind::DETECTION);
  EXPECT_EQ(get_nn_kind({"SSD"}), NnKind::DETECTION);
}

TEST(NnModel, AModelWithoutHeadsIsRejected) { EXPECT_THROW(get_nn_kind({}), std::runtime_error); }

TEST(NnModel, FastSamIsSegmentation)
{
  EXPECT_EQ(get_nn_kind({"FastSAMParser"}), NnKind::SEGMENTATION);
  EXPECT_TRUE(is_fastsam_parser("FastSAMParser"));
  EXPECT_FALSE(is_fastsam_parser("SegmentationParser"));
}

TEST(NnModel, OtherParsersAreRejected)
{
  // DetectionNetwork cannot parse these, so they must not be taken for detection models
  EXPECT_THROW(get_nn_kind({"KeypointParser"}), std::runtime_error);
  EXPECT_THROW(get_nn_kind({"MPPalmDetectionParser"}), std::runtime_error);
  EXPECT_THROW(get_nn_kind({"ImageOutputParser"}), std::runtime_error);
}

TEST(NnModel, SegmentationAndClassificationHeads)
{
  EXPECT_EQ(get_nn_kind({"SegmentationParser"}), NnKind::SEGMENTATION);
  EXPECT_EQ(get_nn_kind({"ClassificationParser"}), NnKind::CLASSIFICATION);
  // Repeated heads of one kind are one kind
  EXPECT_EQ(get_nn_kind({"ClassificationParser", "ClassificationParser"}), NnKind::CLASSIFICATION);
}

TEST(NnModel, MixedKindsAreRejected)
{
  EXPECT_THROW(get_nn_kind({"YOLO", "SegmentationParser"}), std::runtime_error);
  EXPECT_THROW(get_nn_kind({"SegmentationParser", "ClassificationParser"}), std::runtime_error);
}

TEST(NnModel, ClassificationSequencesAreRejected)
{
  EXPECT_THROW(get_nn_kind({"ClassificationSequenceParser"}), std::runtime_error);
}

TEST(NnModel, NamesTheKinds)
{
  EXPECT_STREQ(to_string(NnKind::DETECTION), "detection");
  EXPECT_STREQ(to_string(NnKind::CLASSIFICATION), "classification");
  EXPECT_STREQ(to_string(NnKind::SEGMENTATION), "segmentation");
}

}  // namespace raph_oak
