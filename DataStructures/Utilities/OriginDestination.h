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
#include <vector>

#include <csv.h>

#include "Tools/Constants.h"

// An origin-destination (OD) pair, representing a travel demand or a query.
struct OriginDestination {
  // Constructs an OD-pair from o to d.
  OriginDestination(const int o, const int d)
      : origin(o), destination(d) {}

  // Compares this OD-pair with rhs lexicographically.
  bool operator<(const OriginDestination& rhs) const {
    return origin < rhs.origin || (origin == rhs.origin && destination < rhs.destination);
  }

  int origin;
  int destination;
};

// An origin-destination (OD) pair that additionally stores an origin zone and a destination zone.
// Zones or traffic cells represent for example residential or commercial areas.
struct ClusteredOriginDestination : public OriginDestination {
  // Constructs a clustered OD-pair from o to d.
  ClusteredOriginDestination(const int o, const int d, const int oZone, const int dZone)
      : OriginDestination(o, d), originZone(oZone), destinationZone(dZone) {}

  // Compares this clustered OD-pair with rhs lexicographically.
  bool operator<(const ClusteredOriginDestination& rhs) const {
    if (originZone < rhs.originZone)
      return true;
    if (originZone > rhs.originZone)
      return false;
    if (destinationZone < rhs.destinationZone)
      return true;
    if (destinationZone > rhs.destinationZone)
      return false;
    return OriginDestination::operator<(rhs);
  }

  // Returns true if the OD-pair has the same origin and destination zone as the specified one.
  bool hasSameZones(const ClusteredOriginDestination& other) const {
    return originZone == other.originZone && destinationZone == other.destinationZone;
  }

  int originZone;
  int destinationZone;
};

// Reads the specified file into a vector of OD-pairs.
std::vector<OriginDestination> importODPairsFrom(const std::string& infile) {
  std::vector<OriginDestination> pairs;
  int origin, destination;
  using TrimPolicy = io::trim_chars<>;
  using QuotePolicy = io::no_quote_escape<','>;
  using OverflowPolicy = io::throw_on_overflow;
  using CommentPolicy = io::single_line_comment<'#'>;
  io::CSVReader<2, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> in(infile);
  in.read_header(io::ignore_extra_column, "origin", "destination");
  while (in.read_row(origin, destination)) {
    assert(origin >= 0);
    assert(destination >= 0);
    pairs.emplace_back(origin, destination);
  }
  return pairs;
}

// Reads the specified file into a vector of clustered OD-pairs.
std::vector<ClusteredOriginDestination> importClusteredODPairsFrom(const std::string& infile) {
  std::vector<ClusteredOriginDestination> pairs;
  int origin, destination, originZone = INVALID_ID, destinationZone = INVALID_ID;
  using TrimPolicy = io::trim_chars<>;
  using QuotePolicy = io::no_quote_escape<','>;
  using OverflowPolicy = io::throw_on_overflow;
  using CommentPolicy = io::single_line_comment<'#'>;
  io::CSVReader<4, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> in(infile);
  const io::ignore_column ignore = io::ignore_extra_column | io::ignore_missing_column;
  in.read_header(ignore, "origin", "destination", "origin_zone", "destination_zone");
  while (in.read_row(origin, destination, originZone, destinationZone)) {
    assert(origin >= 0);
    assert(destination >= 0);
    pairs.emplace_back(origin, destination, originZone, destinationZone);
  }
  return pairs;
}
