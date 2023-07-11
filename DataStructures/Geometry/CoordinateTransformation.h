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
#include <string>

#include <proj.h>

// This class provides coordinate transformation between two coordinate reference systems, which
// are specified as EPSG codes.
class CoordinateTransformation {
 public:
  // EPSG codes for some widely used coordinate systems.
  static constexpr int ETRS89_LAEA_EUROPE = 3035;
  static constexpr int WGS_84 = 4326;
  static constexpr int OSGB_1936_BRITISH_NATIONAL_GRID = 27700;
  static constexpr int DHDN_GAUSS_KRUGER_ZONE_3 = 31467;
  static constexpr int DHDN_GAUSS_KRUGER_ZONE_4 = 31468;

  // Constructs a transformation between the two coordinate systems specified as EPSG codes.
  CoordinateTransformation(const int sourceCrs, const int targetCrs) {
    threadingContext = proj_context_create();
    const auto src = "EPSG:" + std::to_string(sourceCrs);
    const auto dst = "EPSG:" + std::to_string(targetCrs);
    const auto trans = proj_create_crs_to_crs(threadingContext, src.c_str(), dst.c_str(), nullptr);
    assert(trans != 0);
    transformation = proj_normalize_for_visualization(threadingContext, trans);
    assert(transformation != 0);
    proj_destroy(trans);
  }

  // Copy constructor.
  CoordinateTransformation(const CoordinateTransformation& other) {
    threadingContext = proj_context_create();
    transformation = proj_normalize_for_visualization(threadingContext, other.transformation);
    assert(transformation != 0);
  }

  // Releases all resources.
  ~CoordinateTransformation() {
    proj_destroy(transformation);
    proj_context_destroy(threadingContext);
  }

  // Transforms a coordinate given in the source CRS into a coordinate in the target CRS.
  void forward(const double srcX, const double srcY, double& dstX, double& dstY) {
    const auto result = proj_trans(transformation, PJ_FWD, {{srcX, srcY, 0, 0}});
    dstX = result.xy.x;
    dstY = result.xy.y;
  }

  // Transforms a coordinate given in the target CRS into a coordinate in the source CRS.
  void reverse(const double srcX, const double srcY, double& dstX, double& dstY) {
    const auto result = proj_trans(transformation, PJ_INV, {{srcX, srcY, 0, 0}});
    dstX = result.xy.x;
    dstY = result.xy.y;
  }

 private:
  PJ_CONTEXT* threadingContext; // The PROJ threading context.
  PJ* transformation;           // The PROJ transformation object.
};
