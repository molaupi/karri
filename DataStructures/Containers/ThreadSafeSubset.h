/*******************************************************************************
 * MIT License
 *
 * This file is part of Mt-KaHyPar.
 *
 * Copyright (C) 2020 Tobias Heuer <tobias.heuer@kit.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <oneapi/tbb/concurrent_vector.h>
#include <mutex>


// based on http://upcoder.com/9/fast-resettable-flag-vector/

namespace karri {

using namespace oneapi::tbb;

// thread_safe_fast_reset_flag_array
template <typename Type = std::uint16_t>
class ThreadSafeFastResetFlagArray {

public:
  explicit ThreadSafeFastResetFlagArray(const size_t size) :
    _v(std::make_unique<Type[]>(size)),
    _threshold(1),
    _size(size) {
    initialize();
  }

  ThreadSafeFastResetFlagArray() :
    _v(nullptr),
    _threshold(1),
    _size(0) { }

  ThreadSafeFastResetFlagArray(const ThreadSafeFastResetFlagArray&) = delete;
  ThreadSafeFastResetFlagArray& operator= (const ThreadSafeFastResetFlagArray&) = delete;

  ThreadSafeFastResetFlagArray(ThreadSafeFastResetFlagArray&&) = default;
  ThreadSafeFastResetFlagArray& operator= (ThreadSafeFastResetFlagArray&&) = default;

  ~ThreadSafeFastResetFlagArray() = default;

  void swap(ThreadSafeFastResetFlagArray& other) {
    using std::swap;
    swap(_v, other._v);
    swap(_threshold, other._threshold);
  }

  bool operator[] (const size_t i) const {
    return isSet(i);
  }

  // ! Changes value of entry i from false to true and returns true, if the value
  // ! hold on position i was false and was successfully set to true
  bool compare_and_set_to_true(const size_t i) {
    Type expected = __atomic_load_n(&_v[i], __ATOMIC_RELAXED);
    Type desired = _threshold;
    if ( expected != _threshold &&
        __atomic_compare_exchange(&_v[i], &expected, &desired,
          false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED) ) {
      // Value was successfully set from false to true
      return true;
    } else {
      // Either expected == _threshold or compare_exchange_strong failed, which means that
      // an other thread set _v[i] to true before.
      return false;
    }
  }

  void set(const size_t i, const bool value) {
    __atomic_store_n(&_v[i], value ? _threshold : 0, __ATOMIC_RELAXED);
  }

  void setUnsafe(const size_t i, const bool value) {
   _v[i] = value ? _threshold : 0;
  }

  void reset() {
    if (_threshold == std::numeric_limits<Type>::max()) {
      initialize();
      _threshold = 0;
    }
    ++_threshold;
  }

  size_t size() const {
    return _size;
  }

  void setSize(const size_t size, const bool init = false) {
    ASSERT(_v == nullptr, "Error");
    _v = std::make_unique<Type[]>(size);
    _size = size;
    initialize(init);
  }

  void resize(const size_t size, const bool init = false) {
    if ( size > _size ) {
      std::unique_ptr<Type[]> tmp_v =
        std::make_unique<Type[]>(size);
      std::swap(_v, tmp_v);
      _size = size;
      initialize(init);
    } else {
      _size = size;
    }
  }

 private:
  bool isSet(size_t i) const {
    return __atomic_load_n(&_v[i], __ATOMIC_RELAXED) == _threshold;
  }

  void initialize(const bool init = false) {
    const Type init_value = init ? _threshold : 0;
    for ( size_t i = 0; i < _size; ++i ) {
      __atomic_store_n(&_v[i], init_value, __ATOMIC_RELAXED);
    }
  }

  std::unique_ptr<Type[]> _v;
  Type _threshold;
  size_t _size;
};

template <typename Type>
void swap(ThreadSafeFastResetFlagArray<Type>& a,
          ThreadSafeFastResetFlagArray<Type>& b) {
  a.swap(b);
}

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
