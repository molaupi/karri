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

#include "DataStructures/Graph/Attributes/AbstractAttribute.h"

// Road categories defined by the XATF file format.
enum class XatfRoadCategory {
  MOTORWAY_FAST        = 1,
  MOTORWAY_MEDIUM      = 2,
  MOTORWAY_SLOW        = 3,
  NATIONAL_ROAD_FAST   = 4,
  NATIONAL_ROAD_MEDIUM = 5,
  NATIONAL_ROAD_SLOW   = 6,
  REGIONAL_ROAD_FAST   = 7,
  REGIONAL_ROAD_MEDIUM = 8,
  REGIONAL_ROAD_SLOW   = 9,
  URBAN_STREET_FAST    = 10,
  URBAN_STREET_MEDIUM  = 11,
  URBAN_STREET_SLOW    = 12,
  FERRY                = 13,
  UNUSED               = 14,
  FOREST_ROAD          = 15,
};

// An attribute associating an XATF road category with each edge of a graph.
class XatfRoadCategoryAttribute : public AbstractAttribute<XatfRoadCategory> {
 public:
  // Returns the attribute's default value.
  static Type defaultValue() {
    return XatfRoadCategory::UNUSED;
  }

  // Returns the XATF road category of edge e.
  const Type& xatfRoadCategory(const int e) const {
    assert(e >= 0); assert(e < values.size());
    return values[e];
  }

  // Returns a reference to the XATF road category of edge e.
  Type& xatfRoadCategory(const int e) {
    assert(e >= 0); assert(e < values.size());
    return values[e];
  }

 protected:
  static constexpr const char* NAME = "xatf_road_category"; // The attribute's unique name.
};
