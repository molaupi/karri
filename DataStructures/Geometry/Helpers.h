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

#include "DataStructures/Geometry/Point.h"
#include "DataStructures/Geometry/Rectangle.h"
#include "Tools/Math.h"

// Returns the normal vector for the line l through p and q, oriented towards the left of l.
inline Point normal(const Point& p, const Point& q) {
  return {p.y() - q.y(), q.x() - p.x()};
}

// Returns 1, -1, or 0 as r lies to the left of, to the right of, or on the line through p and q.
inline int orientation(const Point& p, const Point& q, const Point& r) {
  return signum(normal(p, q) * (r - p));
}

// Returns true if the line segments pq and rs intersect.
inline bool intersection(const Point& p, const Point& q, const Point& r, const Point& s) {
  const int o1 = orientation(p, q, r);
  const int o2 = orientation(p, q, s);
  if (o1 != o2 && orientation(r, s, p) != orientation(r, s, q))
    return true;
  if (o1 != 0 || o2 != 0)
    return false;
  // All points are collinear.
  Rectangle pq(p);
  Rectangle rs(r);
  pq.extend(q);
  rs.extend(s);
  return pq.intersects(rs);
}
