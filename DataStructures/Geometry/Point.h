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
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ostream>

// A point on a two-dimensional plane.
class Point {
 public:
  // Constructs a point at the origin.
  Point() noexcept : coordinates{0, 0} {}

  // Constructs a point at the specified location.
  Point(const int x, const int y) noexcept : coordinates{x, y} {}

  // Returns a reference to the i-th coordinate.
  int& operator[](const int i) {
    assert(i == 0 || i == 1);
    return coordinates[i];
  }

  // Returns the i-th coordinate.
  int operator[](const int i) const {
    assert(i == 0 || i == 1);
    return coordinates[i];
  }

  // Returns a reference to the x-coordinate.
  int& x() {
    return coordinates[0];
  }

  // Returns the x-coordinate.
  int x() const {
    return coordinates[0];
  }

  // Returns a reference to the y-coordinate.
  int& y() {
    return coordinates[1];
  }

  // Returns the y-coordinate.
  int y() const {
    return coordinates[1];
  }

  // Some useful arithmetic operators.
  friend Point operator+(const Point& lhs, const Point& rhs) {
    return {lhs.x() + rhs.x(), lhs.y() + rhs.y()};
  }

  friend Point operator-(const Point& lhs, const Point& rhs) {
    return {lhs.x() - rhs.x(), lhs.y() - rhs.y()};
  }

  friend int64_t operator*(const Point& lhs, const Point& rhs) {
    return static_cast<int64_t>(lhs.x()) * rhs.x() + static_cast<int64_t>(lhs.y()) * rhs.y();
  }

  // Writes a character representation to the specified output stream.
  friend std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << '(' << p.x() << ", " << p.y() << ')';
    return os;
  }

  // Returns true if lhs and rhs coincide.
  friend bool operator==(const Point& lhs, const Point& rhs) {
    return lhs.coordinates == rhs.coordinates;
  }

  // Returns true if lhs and rhs do not coincide.
  friend bool operator!=(const Point& lhs, const Point& rhs) {
    return !(lhs == rhs);
  }

  // Takes the coordinate-wise minimum of this and the specified point.
  void min(const Point& p) {
    x() = std::min(x(), p.x());
    y() = std::min(y(), p.y());
  }

  // Takes the coordinate-wise maximum of this and the specified point.
  void max(const Point& p) {
    x() = std::max(x(), p.x());
    y() = std::max(y(), p.y());
  }

  // Returns the Manhattan or "city block" distance to the specified point.
  int getManhattanDistanceTo(const Point& p) const {
    return std::abs(x() - p.x()) + std::abs(y() - p.y());
  }

  // Returns the Euclidean distance to the specified point.
  double getEuclideanDistanceTo(const Point& p) const {
    return std::sqrt(getSquaredEuclideanDistanceTo(p));
  }

  // Returns the squared Euclidean distance to the specified point.
  int64_t getSquaredEuclideanDistanceTo(const Point& p) const {
    int64_t deltaX = x() - p.x();
    int64_t deltaY = y() - p.y();
    return deltaX * deltaX + deltaY * deltaY;
  }

  // Returns the chessboard, Chebyshev, or sup norm distance to the specified point.
  int getChebyshevDistanceTo(const Point& p) const {
    return std::max(std::abs(x() - p.x()), std::abs(y() - p.y()));
  }

 private:
  std::array<int, 2> coordinates; // The coordinates of this point.
};
