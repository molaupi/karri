/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <cassert>
#include <vector>

#include "Tools/Constants.h"

// A FIFO queue over a finite universe of IDs in [0, n) that, unlike std::queue, also supports removing an
// arbitrary element by ID in O(1). Pushing an ID that is already contained and popping/removing from an
// empty queue or for an ID that is not contained are no-ops (except popFront(), which requires a non-empty
// queue). Implemented as an intrusive doubly linked list over a fixed-size universe.
class AddressableFIFOQueue {
 public:
  // Constructs an empty queue over a finite universe of the specified size.
  explicit AddressableFIFOQueue(const int size)
      : prevId(size, INVALID_INDEX), nextId(size, INVALID_INDEX), contained(size, false),
        headId(INVALID_INDEX), tailId(INVALID_INDEX), numContained(0) {}

  // Returns true if the queue is empty.
  bool empty() const {
    return numContained == 0;
  }

  // Returns the number of elements in the queue.
  int size() const {
    return numContained;
  }

  // Returns true if the queue contains the specified ID.
  bool contains(const int id) const {
    assert(id >= 0); assert(id < static_cast<int>(contained.size()));
    return contained[id];
  }

  // Appends the specified ID to the back of the queue. No-op if the ID is already contained.
  void push(const int id) {
    assert(id >= 0); assert(id < static_cast<int>(contained.size()));
    if (contained[id])
      return;
    contained[id] = true;
    prevId[id] = tailId;
    nextId[id] = INVALID_INDEX;
    if (tailId != INVALID_INDEX)
      nextId[tailId] = id;
    else
      headId = id;
    tailId = id;
    ++numContained;
  }

  // Returns the ID at the front of the queue. The queue must not be empty.
  int front() const {
    assert(!empty());
    return headId;
  }

  // Removes and returns the ID at the front of the queue. The queue must not be empty.
  int popFront() {
    const int id = front();
    remove(id);
    return id;
  }

  // Removes the specified ID from the queue, wherever it currently is. No-op if the ID is not contained.
  void remove(const int id) {
    assert(id >= 0); assert(id < static_cast<int>(contained.size()));
    if (!contained[id])
      return;
    const auto p = prevId[id];
    const auto n = nextId[id];
    if (p != INVALID_INDEX)
      nextId[p] = n;
    else
      headId = n;
    if (n != INVALID_INDEX)
      prevId[n] = p;
    else
      tailId = p;
    prevId[id] = INVALID_INDEX;
    nextId[id] = INVALID_INDEX;
    contained[id] = false;
    --numContained;
  }

 private:
  std::vector<int> prevId;      // The ID preceding each ID in the queue, or INVALID_INDEX.
  std::vector<int> nextId;      // The ID succeeding each ID in the queue, or INVALID_INDEX.
  std::vector<bool> contained;  // Whether each ID is currently contained in the queue.
  int headId;                   // The ID at the front of the queue, or INVALID_INDEX if empty.
  int tailId;                   // The ID at the back of the queue, or INVALID_INDEX if empty.
  int numContained;             // The number of IDs currently in the queue.
};
