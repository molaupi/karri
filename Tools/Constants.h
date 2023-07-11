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

#include <limits>

// A special value representing infinity.
constexpr int INFTY = std::numeric_limits<int>::max() / 2;

// Special values representing an invalid (vertex/edge) ID.
constexpr int INVALID_ID = -1;
constexpr int INVALID_INDEX = -1;
constexpr int INVALID_VERTEX = -1;
constexpr int INVALID_EDGE = -1;

// This enum provides constants to specify the direction in which a road segment is open.
enum class RoadDirection { OPEN_IN_BOTH, FORWARD, REVERSE, CLOSED };

// This enum provides constants to specify the direction of a search.
enum class SearchDirection { FORWARD, REVERSE };

// The earth's mean radius in meters.
constexpr int EARTH_RADIUS = 6371000;

// The maximum number of shortest paths computed simultaneously during traffic assignment.
#ifndef TA_LOG_K
# define TA_LOG_K 5
#endif

// The maximum number of source vertices that are to be substituted into radiation model's formula.
#ifndef DC_MAX_NUM_SOURCES
# define DC_MAX_NUM_SOURCES std::numeric_limits<int>::max()
#endif
