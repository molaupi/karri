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

#include <cstdint>
#include <limits>

// This class represents an entry in the bucket of a vertex v. It can be thought of as a shortcut
// from v to targetId with length distToTarget. Note that the terminology was developed with
// one-to-many queries in mind. In many-to-one queries, targetId stores a source ID and distToTarget
// stores the distance from the corresponding source to v.
struct BucketEntry {
  BucketEntry() noexcept = default;

  BucketEntry(const int targetId, const int distToTarget) noexcept
      : targetId(targetId), distToTarget(distToTarget) {}

  constexpr bool operator==(const BucketEntry& rhs) const noexcept {
    return targetId == rhs.targetId;
  }

  int32_t targetId = std::numeric_limits<int32_t>::max();
  int32_t distToTarget;
};
