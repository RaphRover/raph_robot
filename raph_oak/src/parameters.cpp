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

#include "raph_oak/parameters.hpp"

#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "raph_oak/oak_wrapper_parameters.hpp"

namespace raph_oak
{

void update_depth_config_from_params(
  dai::RawStereoDepthConfig & depth_config, const Params & params)
{
  depth_config.costMatching.confidenceThreshold = params.depth.confidence_threshold;
  depth_config.algorithmControl.enableLeftRightCheck = params.depth.lr_check_enabled;
  depth_config.algorithmControl.leftRightCheckThreshold = params.depth.lr_check_threshold;
  depth_config.algorithmControl.enableSubpixel = params.depth.subpixel_enabled;
  depth_config.algorithmControl.subpixelFractionalBits = params.depth.subpixel_fractional_bits;
  depth_config.postProcessing.thresholdFilter.minRange =
    static_cast<int>(params.depth.min_distance * 1000.0);
  depth_config.postProcessing.thresholdFilter.maxRange =
    static_cast<int>(params.depth.max_distance * 1000.0);
}

}  // namespace raph_oak
