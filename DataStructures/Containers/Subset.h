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

// This class represents a subset of a finite set of size n. Inserting elements, removing elements,
// and testing elements for membership take constant time. Iterating through and clearing a subset
// of size k both take time O(k).
class Subset {
 public:
  // Constructs an empty subset of a finite set of the specified size.
  explicit Subset(const int size) : elementsToIndices(size, INVALID_INDEX) {
    elements.reserve(size);
  }

  void resizeUnderlyingSet(const int newSize) {
      elements.reserve(newSize);
      elementsToIndices.resize(newSize, INVALID_INDEX);
  }

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
    elementsToIndices[element] = elements.size();
    elements.push_back(element);
    return true;
  }

  // Removes the specified element from the subset. May invalidate all iterators.
  bool remove(const int element) {
    if (!contains(element))
      return false;
    elements[elementsToIndices[element]] = elements.back();
    elementsToIndices[elements.back()] = elementsToIndices[element];
    elements.pop_back();
    elementsToIndices[element] = INVALID_INDEX;
    return true;
  }

  // Removes all elements in the subset. May invalidate all iterators.
  void clear() {
    for (const auto element : elements)
      elementsToIndices[element] = INVALID_INDEX;
    elements.clear();
  }

  // Returns true if the subset contains the specified element.
  bool contains(const int element) const {
    assert(element >= 0); assert(element < elementsToIndices.size());
    return elementsToIndices[element] != INVALID_INDEX;
  }

 private:
  std::vector<int32_t> elements;          // The elements contained in the subset.
  std::vector<int32_t> elementsToIndices; // The index in the element array of each element.
};
