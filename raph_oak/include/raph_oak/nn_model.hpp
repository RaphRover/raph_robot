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

#include <string>
#include <vector>

namespace raph_oak
{

/// What a neural network model outputs, which decides how its results are parsed and published
enum class NnKind
{
  DETECTION,
  CLASSIFICATION,
  SEGMENTATION,
};

const char * to_string(NnKind kind);

/**
 * Tells the kind of a model from the parsers of the heads in its NNArchive config: YOLO and SSD
 * heads are detection (what DetectionNetwork takes), classification heads are classification,
 * segmentation and FastSAM heads are segmentation.
 *
 * @throws std::runtime_error if the model has no heads, heads of more than one kind or a head
 *   that is not supported (any other parser, such as keypoints or a classification sequence)
 */
NnKind get_nn_kind(const std::vector<std::string> & parsers);

/// FastSAM heads are segmentation, but have a parser of their own: it marks every instance with
/// its own index, so the indices are no classes
bool is_fastsam_parser(const std::string & parser);

}  // namespace raph_oak
