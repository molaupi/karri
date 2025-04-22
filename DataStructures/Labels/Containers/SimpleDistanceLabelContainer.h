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
#include "Tools/Constants.h"

// The simplest implementation of a container maintaining distance labels during a shortest-path
// search. The distance labels are explicitly initialized with a linear sweep over all labels.
template <typename DistanceLabelT>
class SimpleDistanceLabelContainer {
 public:
  // Constructs a distance label container with explicit initialization.
  explicit SimpleDistanceLabelContainer(const int numLabels) {
    resize(numLabels);
  }

  // Ensures that this container can hold the specified number of distance labels.
  void resize(const int numLabels) {
    const auto currentSize = distanceLabels.size();
    if (numLabels < currentSize)
      distanceLabels.erase(distanceLabels.begin() + numLabels, distanceLabels.end());
    else
      distanceLabels.insert(distanceLabels.end(), numLabels - currentSize, INFTY);
  }

  // Initializes all distance labels to infinity.
  void init() {
    std::fill(distanceLabels.begin(), distanceLabels.end(), INFTY);
  }

  // Returns a reference to the distance label of vertex v.
  DistanceLabelT& operator[](const int v) {
    assert(v >= 0); assert(v < distanceLabels.size());
    return distanceLabels[v];
  }

    // Returns distance at v. Implements StampedDistanceLabelContainer interface.
    DistanceLabelT readDistance(const int v) const {
        assert(v >= 0);
        assert(v < distanceLabels.size());
        return distanceLabels[v];
    }

    // Returns distance at v. Implements StampedDistanceLabelContainer interface.
    DistanceLabelT readDistanceWithoutStaleCheck(const int v) const {
        assert(v >= 0);
        assert(v < distanceLabels.size());
        return distanceLabels[v];
    }

    // Returns true. Implements StampedDistanceLabelContainer interface.
    bool isStale(const int ) const {
        return true;
    }

 private:
  AlignedVector<DistanceLabelT> distanceLabels; // The distance labels of the vertices.
};
