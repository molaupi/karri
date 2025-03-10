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

#include <cassert>
#include <cstdint>
#include <vector>

#include "Tools/Constants.h"
#include "BitVector.h"

// This class offers similar functionality to Subset with two differences:
//  1. Individual elements cannot be removed.
//  2. The insert operation invalidates all iterators.
// In comparison to Subset, this class requires less memory and is faster for insertions. It should be used if
// deduplication is desired when adding elements one by one, and elements are always removed all at once using clear().
// Inserting elements, and testing elements for membership take constant time. Iterating through and clearing a
// NonAddressableSubset of size k both take time O(k).
class LightweightSubset {
 public:
  // Constructs an empty subset of a finite set of the specified size.
  explicit LightweightSubset(const int size) : hasElement(size) {}

  // Returns an iterator referring to the first element in the subset.
  std::vector<int32_t>::const_iterator begin() const noexcept {
    return elements.begin();
  }

  // Returns the past-the-end iterator for the subset.
  std::vector<int32_t>::const_iterator end() const noexcept {
    return elements.end();
  }

  // Returns the number of elements in the subset.
  int size() const noexcept {
	  return elements.size();
  }

  // Inserts the specified element into the subset. Invalidates only the past-the-end iterator.
  bool insert(const int element) {
    if (contains(element))
      return false;
    hasElement[element] = true;
    elements.push_back(element);
    return true;
  }

  // Removes all elements in the subset. May invalidate all iterators.
  void clear() {
    for (const auto element : elements)
      hasElement[element] = false;
    elements.clear();
  }

  // Returns true if the subset contains the specified element.
  bool contains(const int element) const {
    KASSERT(element >= 0); assert(element < hasElement.size());
    return hasElement[element];
  }

 private:
  std::vector<int32_t> elements;          // The elements contained in the subset.
  BitVector hasElement;
};
