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

#include "raph_oak/nn_model.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace raph_oak
{
namespace
{
std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}
}  // namespace

const char * to_string(NnKind kind)
{
  switch (kind) {
    case NnKind::DETECTION:
      return "detection";
    case NnKind::CLASSIFICATION:
      return "classification";
    case NnKind::SEGMENTATION:
      return "segmentation";
  }
  return "unknown";
}

bool is_fastsam_parser(const std::string & parser) { return contains(to_lower(parser), "fastsam"); }

NnKind get_nn_kind(const std::vector<std::string> & parsers)
{
  if (parsers.empty()) {
    throw std::runtime_error("The model has no heads, so its output cannot be parsed");
  }

  std::set<NnKind> kinds;
  for (const auto & parser : parsers) {
    const auto name = to_lower(parser);
    if (contains(name, "sequence")) {
      throw std::runtime_error("The " + parser + " head is not supported");
    }
    if (contains(name, "yolo") || contains(name, "ssd")) {
      kinds.insert(NnKind::DETECTION);
    } else if (contains(name, "classification")) {
      kinds.insert(NnKind::CLASSIFICATION);
    } else if (contains(name, "segmentation") || is_fastsam_parser(parser)) {
      kinds.insert(NnKind::SEGMENTATION);
    } else {
      throw std::runtime_error(
        "The " + parser +
        " head is not supported, only detection, classification and "
        "segmentation models are");
    }
  }

  if (kinds.size() > 1) {
    throw std::runtime_error("The model has heads of more than one kind");
  }
  return *kinds.begin();
}

}  // namespace raph_oak
