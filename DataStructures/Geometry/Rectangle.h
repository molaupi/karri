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
#include <ostream>

#include "DataStructures/Geometry/Point.h"
#include "Tools/Constants.h"

// A rectangle on a two-dimensional plane with sides parallel to the x- and y-axis.
class Rectangle {
 public:
  // Constructs an empty bounding box that can be gradually extended.
  Rectangle() noexcept : sw(INFTY, INFTY), ne(-INFTY, -INFTY) {}

  // Constructs a rectangle from a single point.
  explicit Rectangle(const Point& p) noexcept : sw(p), ne(p) {}

  // Constructs a rectangle from the points at its south-west and north-east corners.
  Rectangle(const Point& sw, const Point& ne) noexcept : sw(sw), ne(ne) {
    assert(sw.x() <= ne.x());
    assert(sw.y() <= ne.y());
  }

  // Constructs a bounding box containing all specified points.
  template <typename PointIteratorT>
  Rectangle(PointIteratorT first, PointIteratorT last) : Rectangle() {
    extend(first, last);
  }

  // Returns a reference to the south-west corner.
  Point& southWest() noexcept {
    return sw;
  }

  // Returns the south-west corner.
  const Point& southWest() const noexcept {
    return sw;
  }

  // Returns a reference to the north-east corner.
  Point& northEast() noexcept {
    return ne;
  }

  // Returns the north-east corner.
  const Point& northEast() const noexcept {
    return ne;
  }

  // Returns true if p is inside the boundary of this rectangle.
  bool contains(const Point& p) const {
    return sw.x() <= p.x() && p.x() <= ne.x() && sw.y() <= p.y() && p.y() <= ne.y();
  }

  // Returns true if this and the specified rectangle intersect.
  bool intersects(const Rectangle& rect) const {
    return ne.x() >= rect.sw.x() && sw.x() <= rect.ne.x() &&
        ne.y() >= rect.sw.y() && sw.y() <= rect.ne.y();
  }

  // Writes a character representation to the specified output stream.
  friend std::ostream& operator<<(std::ostream& os, const Rectangle& rect) {
    os << "(SW=" << rect.sw << ", NE=" << rect.ne << ")";
    return os;
  }

  // Returns true if all corners of lhs and rhs coincide.
  friend bool operator==(const Rectangle& lhs, const Rectangle& rhs) {
    return lhs.sw == rhs.sw && lhs.ne == rhs.ne;
  }

  // Returns true if at least one corner of lhs and rhs does not coincide.
  friend bool operator!=(const Rectangle& lhs, const Rectangle& rhs) {
    return !(lhs == rhs);
  }

  // Extends this rectangle to contain the point p.
  void extend(const Point& p) {
    sw.min(p);
    ne.max(p);
  }

  // Extends this rectangle to contain the specified points.
  template <typename PointIteratorT>
  void extend(PointIteratorT first, PointIteratorT last) {
    for (; first != last; ++first)
      extend(*first);
  }

 private:
  Point sw; // The south-west corner.
  Point ne; // The north-east corner.
};
