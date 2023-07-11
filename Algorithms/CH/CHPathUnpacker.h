/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

#include "Algorithms/CH/CH.h"
#include "Tools/Constants.h"

// This class unpacks an up-down path in a contraction hierarchy (which generally contains
// shortcuts) into the corresponding path in the original graph (which contains only original
// edges). A path is represented as a dynamic array of edge identifiers. For efficiency, we do not
// allocate a new array for each path we unpack. Instead, the caller passes an array that gets
// filled with the unpacked path. Therefore, arrays can be reused in successive calls.
class CHPathUnpacker {
 public:
  using Path = std::vector<int32_t>;

  explicit CHPathUnpacker(const CH& ch) : ch(ch) {}

  void unpackUpDownPath(const Path& upPath, const Path& downPath, Path& unpackedPath) {
    assert(packedPath.empty());
    packedPath.insert(packedPath.end(), downPath.rbegin(), downPath.rend());
    std::for_each(packedPath.begin(), packedPath.end(), [](int& edge) { edge = -(edge + 1); });
    packedPath.insert(packedPath.end(), upPath.begin(), upPath.end());

    while (!packedPath.empty()) {
      const auto edge = packedPath.back();
      packedPath.pop_back();
      const auto& unpackingInfo = edge >= 0 ?
          ch.upwardGraph().unpackingInfo(edge) : ch.downwardGraph().unpackingInfo(-edge - 1);
      if (unpackingInfo.second == INVALID_EDGE) {
        unpackedPath.push_back(unpackingInfo.first);
      } else {
        packedPath.push_back(unpackingInfo.second);
        packedPath.push_back(-(unpackingInfo.first + 1));
      }
    }
  }

 private:
  const CH& ch;
  Path packedPath;
};
