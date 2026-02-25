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

#include "depthai/depthai.hpp"

#include "raph_oak/oak_wrapper_parameters.hpp"

namespace raph_oak
{

constexpr int kMonoWidth = 1280;
constexpr int kMonoHeight = 800;
constexpr int kMonoIspScaleNumerator = 1;
constexpr int kMonoIspScaleDenominator = 1;
constexpr float kMonoFps = 10.0F;
constexpr int kMonoCompressedQuality = 80;

constexpr int kRgbWidth = 1344;
constexpr int kRgbHeight = 1008;
constexpr int kRgbIspScaleNumerator = 1;
constexpr int kRgbIspScaleDenominator = 3;
constexpr float kRgbFps = 15.0F;
constexpr int kRgbCompressedQuality = 80;

dai::Pipeline create_dai_pipeline(const Params & params);

}   // namespace raph_oak
