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

#include <cstdint>
#include <string>
#include <vector>

#include <routingkit/vector_io.h>

#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

// An exporter for writing graphs in RoutingKit file format. Writes the graph topology as a CSR
// (first_out, head) pair, plus the latitude/longitude of each vertex and the travel time and
// capacity of each edge, each as a separate RoutingKit vector file.
class RoutingKitExporter {
 public:
  RoutingKitExporter(bool /*compress*/) {}

  void ignoreAttribute(const std::string& /*attrName*/) {
    // No op, as RoutingKitExporter always outputs the fixed set of files expected by RoutingKit.
  }

  void init(std::string filename) {
    this->filename = std::move(filename);
  }

  void writeHeader(const int numVertices, const int numEdges) {
    firstOut.assign(numVertices + 1, 0);
    head.reserve(numEdges);
    travelTime.reserve(numEdges);
    capacity.reserve(numEdges);
    latitude.reserve(numVertices);
    longitude.reserve(numVertices);
  }

  void startVertex(const int v) {
    // Vertices are visited in order of increasing ID, so the number of edges output so far is
    // exactly the first out-edge of v.
    firstOut[v] = static_cast<uint32_t>(head.size());
  }

  template<typename Attr, typename DUMMY = void>
  struct SetCurrentVertexAttributeValue {
    static void set(RoutingKitExporter&, const typename Attr::Type&) {
      // no op
    }
  };

  template<typename DUMMY>
  struct SetCurrentVertexAttributeValue<LatLngAttribute, DUMMY> {
    static void set(RoutingKitExporter& exporter, const typename LatLngAttribute::Type& value) {
      exporter.currentVertexLat = static_cast<float>(value.latInDeg());
      exporter.currentVertexLng = static_cast<float>(value.lngInDeg());
    }
  };

  // Set the given attribute value for the current vertex.
  // Default implementation does nothing.
  template<typename Attr>
  void setCurrentVertexAttributeValue(const typename Attr::Type& value) {
    SetCurrentVertexAttributeValue<Attr>::set(*this, value);
  }

  void finalizeVertex() {
    latitude.push_back(currentVertexLat);
    longitude.push_back(currentVertexLng);
  }

  void startEdge(const int /*tail*/, const int head) {
    currentEdgeHead = static_cast<uint32_t>(head);
  }

  template<typename Attr, typename DUMMY = void>
  struct SetCurrentEdgeAttributeValue {
    static void set(RoutingKitExporter&, const typename Attr::Type&) {
      // no op
    }
  };

  template<typename DUMMY>
  struct SetCurrentEdgeAttributeValue<TravelTimeAttribute, DUMMY> {
    static void set(RoutingKitExporter& exporter, const typename TravelTimeAttribute::Type& value) {
      exporter.currentEdgeTravelTime = static_cast<uint32_t>(value);
    }
  };

  template<typename DUMMY>
  struct SetCurrentEdgeAttributeValue<CapacityAttribute, DUMMY> {
    static void set(RoutingKitExporter& exporter, const typename CapacityAttribute::Type& value) {
      exporter.currentEdgeCapacity = static_cast<uint32_t>(value);
    }
  };

  // Set the given attribute value for the current edge.
  // Default implementation does nothing.
  template<typename Attr>
  void setCurrentEdgeAttributeValue(const typename Attr::Type& value) {
    SetCurrentEdgeAttributeValue<Attr>::set(*this, value);
  }

  void finalizeEdge() {
    head.push_back(currentEdgeHead);
    travelTime.push_back(currentEdgeTravelTime);
    capacity.push_back(currentEdgeCapacity);
  }

  void finalize() {
    firstOut.back() = static_cast<uint32_t>(head.size());

    RoutingKit::save_vector(filename + "first_out", firstOut);
    RoutingKit::save_vector(filename + "head", head);
    RoutingKit::save_vector(filename + "travel_time", travelTime);
    RoutingKit::save_vector(filename + "capacity", capacity);
    RoutingKit::save_vector(filename + "latitude", latitude);
    RoutingKit::save_vector(filename + "longitude", longitude);
  }

 private:
  std::string filename;

  std::vector<uint32_t> firstOut;
  std::vector<uint32_t> head;
  std::vector<uint32_t> travelTime;
  std::vector<uint32_t> capacity;
  std::vector<float> latitude;
  std::vector<float> longitude;

  float currentVertexLat;
  float currentVertexLng;

  uint32_t currentEdgeHead;
  uint32_t currentEdgeTravelTime;
  uint32_t currentEdgeCapacity;
};
