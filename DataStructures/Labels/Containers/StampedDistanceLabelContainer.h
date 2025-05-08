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
#include <vector>

#include "Tools/Simd/AlignedVector.h"
#include "Tools/CompilerSpecific.h"
#include "Tools/Constants.h"

// A container maintaining distance labels. It stores a global clock and a timestamp for each
// distance label. The timestamp indicates whether a distance label has a valid value or not.
template <typename DistanceLabelT>
class StampedDistanceLabelContainer {
 public:
  // Constructs a distance label container using timestamps.
  explicit StampedDistanceLabelContainer(const int numVertices) : clock(0) {
    resize(numVertices);
  }

  auto size() const {
      return distanceLabels.size();
  }

  // Ensures that this container can hold the specified number of distance labels.
  void resize(const int numVertices) {
    const auto currentSize = distanceLabels.size();
    if (numVertices < currentSize) {
      distanceLabels.erase(distanceLabels.begin() + numVertices, distanceLabels.end());
      timestamps.erase(timestamps.begin() + numVertices, timestamps.end());
    } else {
      distanceLabels.insert(distanceLabels.end(), numVertices - currentSize, DistanceLabelT());
      timestamps.insert(timestamps.end(), numVertices - currentSize, 0);
    }
  }

  // Initializes all distance labels to infinity.
  void init() {
    ++clock;
    if (UNLIKELY(clock < 0)) {
      // Clock overflow occurred. Extremely unlikely.
      std::fill(timestamps.begin(), timestamps.end(), 0);
      clock = 1;
    }
  }

  // Returns a reference to the distance label of v.
  DistanceLabelT& operator[](const int v) {
    assert(v >= 0); assert(v < distanceLabels.size());
    if (timestamps[v] != clock) {
      assert(timestamps[v] < clock);
      distanceLabels[v] = INFTY;
      timestamps[v] = clock;
    }
    return distanceLabels[v];
  }

  // Return the last value written to the distance label of v without checking if it is stale.
  // Use when certain that value has already been accessed since last call to init().
  const DistanceLabelT& readDistanceWithoutStaleCheck(const int v) const {
      return distanceLabels[v];
  }

 private:
  AlignedVector<DistanceLabelT> distanceLabels; // The distance labels of the vertices.
  std::vector<int> timestamps;                  // The timestamps indicating if a label is valid.
  int clock;                                    // The global clock.
};
