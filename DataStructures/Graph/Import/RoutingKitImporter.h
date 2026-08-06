/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler
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

#include <string>

#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include <External/RoutingKit/include/routingkit/vector_io.h>

#include "DataStructures/Geometry/LatLng.h"
#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "kassert/kassert.hpp"

// An importer for reading graphs in RoutingKit file format.
class RoutingKitImporter {
 public:
  // Constructs an importer to read graphs in RoutingKit file format.
  RoutingKitImporter() : first_out(1, 0), currentVertex(0), currentEdge(0), currentEdgeTail(0) {
  }

  // Returns the number of vertices in the graph, or 0 if the number is not yet known.
  int numVertices() const {
    return first_out.size() - 1;
  }

  // Returns the number of edges in the graph, or 0 if the number is not yet known.
  int numEdges() const {
    return head.size();
  }

  // Reads the RoutingKit vectors.
  void init(const std::string& filename) {
    currentVertex = -1;
    currentEdge = -1;
    currentEdgeTail = -1;

    first_out = RoutingKit::load_vector<uint32_t>(filename + "first_out");
    head = RoutingKit::load_vector<uint32_t>(filename + "head");
    travel_time = RoutingKit::load_vector<uint32_t>(filename + "travel_time");
    try {
      capacity = RoutingKit::load_vector<uint32_t>(filename + "capacity");
    } catch (const std::runtime_error&) {
      capacity = std::vector<uint32_t>(head.size(), CapacityAttribute::defaultValue());
    }
    const auto latitude = RoutingKit::load_vector<float>(filename + "latitude");
    const auto longitude = RoutingKit::load_vector<float>(filename + "longitude");
    const auto numVertices = first_out.size() - 1;
    KASSERT(latitude.size() == numVertices);
    KASSERT(longitude.size() == numVertices);
    lat_lng = std::vector<LatLng>(numVertices);
    for (int i = 0; i < numVertices; ++i) {
      lat_lng[i] = LatLng(latitude[i], longitude[i]);
    }
  }

  // Go to next vertex and return whether a valid vertex remains.
  bool nextVertex() {
    ++currentVertex;
    return currentVertex < numVertices();
  }

  // Returns the ID of the current vertex. Vertices must have sequential IDs from 0 to n − 1.
  int vertexId() const {
    return currentVertex;
  }

  // Go to next edge and return whether a valid edge remains.
  bool nextEdge() {
    ++currentEdge;
    if (currentEdge >= numEdges()) {
      return false;
    }
    while (currentEdge >= first_out[currentEdgeTail + 1]) {
      ++currentEdgeTail;
    }
    return true;
  }

  // Returns the tail vertex of the current edge.
  int edgeTail() const {
    return currentEdgeTail;
  }

  // Returns the head vertex of the current edge.
  int edgeHead() const {
    return head[currentEdge];
  }

  // Returns the value of the specified attribute for the current vertex/edge, or the attribute's
  // default value if the attribute is not part of the file format.
  template <typename Attr>
  typename Attr::Type getValue() const {
    return Attr::defaultValue();
  }

  void close() {}

 private:

  std::vector<uint32_t> first_out;
  std::vector<uint32_t> head;
  std::vector<uint32_t> travel_time;
  std::vector<uint32_t> capacity;
  std::vector<LatLng> lat_lng;

  int currentVertex; // The vertex read by the last call of nextVertex.
  int currentEdge;     // The edge read by the last call of nextEdge.
  int currentEdgeTail; // Tail of current edge.
};

// Returns the value of the LatLng attribute for the current vertex.
template <>
inline LatLngAttribute::Type RoutingKitImporter::getValue<LatLngAttribute>() const {
  return lat_lng[currentVertex];
}

// Returns the value of the travel time attribute for the current edge.
template <>
inline TravelTimeAttribute::Type RoutingKitImporter::getValue<TravelTimeAttribute>() const {
  return static_cast<TravelTimeAttribute::Type>(travel_time[currentEdge]);
}

// Returns the value of the capacity attribute for the current edge.
template <>
inline CapacityAttribute::Type RoutingKitImporter::getValue<CapacityAttribute>() const {
  return static_cast<CapacityAttribute::Type>(capacity[currentEdge]);
}