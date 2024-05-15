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
#include <ostream>
#include <vector>

#include "DataStructures/Geometry/Helpers.h"
#include "DataStructures/Geometry/Point.h"
#include "Tools/custom_assertion_levels.h"

// A polygon defines a two-dimensional region enclosed by a single closed polygonal chain.
class Polygon {
 public:
  // Iterators referring to vertices of this polygon.
  using VertexIterator = std::vector<Point>::iterator;
  using ConstVertexIterator = std::vector<Point>::const_iterator;
  using ReverseVertexIterator = std::vector<Point>::reverse_iterator;
  using ConstReverseVertexIterator = std::vector<Point>::const_reverse_iterator;

  // Constructs an empty polygon.
  Polygon() = default;

  // Constructs a polygon with vertices from the specified sequence.
  template <typename VertexIteratorT>
  Polygon(VertexIteratorT first, VertexIteratorT last) {
    add(first, last);
  }

  // Writes a character representation to the specified output stream.
  friend std::ostream& operator<<(std::ostream& os, const Polygon& polygon) {
    os << "(";
    for (int i = 0; i < polygon.size(); ++i) {
      if (i != 0)
        os << ", ";
      os << polygon.vertices[i];
    }
    os << ")";
    return os;
  }

  // Returns the vertex with the specified index.
  const Point& operator[](const int idx) const {
    assert(idx >= 0); assert(idx < size());
    return vertices[idx];
  }

  // Returns true if this polygon is of size 0.
  bool empty() const {
    return vertices.empty();
  }

  // Returns the number of vertices.
  int size() const {
    return vertices.size();
  }

  // Returns the vertex with the lowest index.
  const Point& front() const {
    assert(!empty());
    return vertices.front();
  }

  // Returns the vertex with the highest index.
  const Point& back() const {
    assert(!empty());
    return vertices.back();
  }

  // Returns an iterator referring to the first vertex of this polygon.
  ConstVertexIterator begin() const {
    return vertices.begin();
  }

  // Returns an iterator which is the past-the-end value for this polygon.
  ConstVertexIterator end() const {
    return vertices.end();
  }

  ConstReverseVertexIterator rbegin() const {
    return vertices.rbegin();
  }

  ConstReverseVertexIterator rend() const {
    return vertices.rend();
  }

  // Appends the specified vertex to this polygon.
  void add(const Point& p) {
    vertices.push_back(p);
  }

  // Appends all of the specified vertices to this polygon.
  template <typename VertexIteratorT>
  void add(VertexIteratorT first, VertexIteratorT last) {
    vertices.insert(vertices.end(), first, last);
  }

  // Removes the vertex with the highest index from this polygon.
  void removeBack() {
    assert(!empty());
    vertices.pop_back();
  }

  // Removes the vertex pointed to by pos from this polygon.
  VertexIterator remove(ConstVertexIterator pos) {
    assert(vertices.begin() <= pos); assert(pos < vertices.end());
    return vertices.erase(pos);
  }

  // Removes from this polygon all of the vertices between first, inclusive, and last, exclusive.
  VertexIterator removeRange(ConstVertexIterator first, ConstVertexIterator last) {
    assert(first >= vertices.begin());
    assert(last <= vertices.end());
    assert(first <= last);
    return vertices.erase(first, last);
  }

  // Returns the index of a lowest of the leftmost vertices.
  // Precondition: The polygon must not be empty.
  int leftmostVertex() const {
    assert(!empty());
    int minIdx = 0;
    Point min = vertices[0];
    for (int i = 1; i < size(); ++i)
      if (vertices[i].x() < min.x() || (vertices[i].x() == min.x() && vertices[i].y() < min.y())) {
        minIdx = i;
        min = vertices[i];
      }
    return minIdx;
  }

  // Returns 1 or -1 as this polygon is counterclockwise or clockwise oriented.
  // Precondition: The polygon must be simple.
  int orientation() const {
    HEAVY_KASSERT(simple());
    const int q = leftmostVertex();
    const int p = (q - 1 + size()) % size();
    const int r = (q + 1) % size();
    return ::orientation(vertices[p], vertices[q], vertices[r]);
  }

  // Reverses the orientation of this polygon.
  void reverseOrientation() {
    std::reverse(vertices.begin(), vertices.end());
  }

  // Returns true if this polygon is simple, i.e., its boundary does not intersect itself.
  // Note: The implementation takes quadratic time, whereas the problem can be solved in O(nlog⁡n).
  bool simple() const {
    if (size() < 3)
      return false;
    // Test each pair of edges. Edges that are consecutive must not form an angle of zero degrees.
    // Non-consecutive edges must not intersect.
    for (int i = size() - 1, j = 0; j < size() - 1; i = j++) {
      // The consecutive edges ij and j(j + 1) must not form an angle of zero degrees.
      if (::orientation(vertices[i], vertices[j], vertices[j + 1]) == 0 &&
          (vertices[i] - vertices[j]) * (vertices[j + 1] - vertices[j]) > 0)
        return false;
      for (int k = j + 1; k < size() - 1; ++k)
        if (k + 1 != i) {
          // The non-consecutive edges ij and k(k + 1) must not intersect.
          if (intersection(vertices[i], vertices[j], vertices[k], vertices[k + 1]))
            return false;
        } else {
          // The consecutive edges ki and ij must not form an angle of zero degrees.
          if (::orientation(vertices[k], vertices[i], vertices[j]) == 0 &&
              (vertices[k] - vertices[i]) * (vertices[j] - vertices[i]) > 0)
            return false;
        }
    }
    return true;
  }

  // Returns the signed doubled area of this polygon.
  int64_t doubledArea() const noexcept {
    const auto n = vertices.size();
    if (n < 3)
      return 0;
    int64_t area = 0;
    for (auto i = 0; i < n - 2; ++i)
      area += vertices[i + 1].x() * int64_t{vertices[i + 2].y() - vertices[i].y()};
    area += vertices[n - 1].x() * int64_t{vertices[0].y() - vertices[n - 2].y()};
    area += vertices[0].x() * int64_t{vertices[1].y() - vertices[n - 1].y()};
    return area;
  }

  // Returns true if r is inside the boundary of this polygon.
  bool contains(const Point& r) const {
    bool inside = false;
    for (int i = size() - 1, j = 0; j < size(); i = j++)
      if ((r.y() < vertices[i].y()) != (r.y() < vertices[j].y()) &&
          ::orientation(vertices[i], vertices[j], r) * (vertices[j].y()-vertices[i].y()) > 0)
        inside = !inside;
    return inside;
  }

 private:
  std::vector<Point> vertices; // The vertices of this polygon.
};
