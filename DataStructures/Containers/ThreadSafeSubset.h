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

#include "Parallel/thread_safe_fast_reset_flag_array.h"
#include <tbb/concurrent_vector.h>

// based on http://upcoder.com/9/fast-resettable-flag-vector/

namespace karri {

using namespace tbb;
// This class represents a subset of a finite set of size n. Inserting elements, removing elements,
// and testing elements for membership take constant time. Iterating through and clearing a subset
// of size k both take time O(k).
class ThreadSafeSubset {
 public:
  // Constructs an empty subset of a finite set of the specified size.
  explicit ThreadSafeSubset(const int size) 
            : flags(size)
        {
            elements.reserve(size);
        }

  // Returns an iterator referring to the first element in the subset.
  concurrent_vector<int32_t>::const_iterator begin() const noexcept {
    return elements.begin();
  }

  // Returns the past-the-end iterator for the subset.
  concurrent_vector<int32_t>::const_iterator end() const noexcept {
    return elements.end();
  }

  // Returns the number of elements in the subset.
  int size() const noexcept {
	  return elements.size();
  }

  // Inserts the specified element into the subset. Invalidates only the past-the-end iterator.
  bool insert(const int element) {

      if (!flags.compare_and_set_to_true(element))
          return false;

      elements.push_back(element);
      return true;

  }


  // Removes all elements in the subset. May invalidate all iterators. Not thread safe.
  void clear() {
    elements.clear();
    flags.reset();
  }

  // Returns true if the subset contains the specified element.
  bool contains(const int element) const {
    assert(element >= 0); assert(element < flags.size());
    return flags[element];
  }

 private:
  concurrent_vector<int32_t> elements;           // The elements contained in the subset.
  ThreadSafeFastResetFlagArray<> flags;         // The flags whether the element is set successfully in elements.
};

}  // namespace karri
