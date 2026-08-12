/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler
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
#include <fstream>

#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "Tools/StringHelpers.h"

#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"

// When converting to LinTim infrastructure format, you must use attributes lat_lng, length, capacity.
class LinTimInfrastructureExporter {
public:
    LinTimInfrastructureExporter(bool /*compress*/) {}

    void ignoreAttribute(const std::string&) {
        // No op, as LinTimInfrastructureExporter always prints same attributes.
    }

    void init(std::string filename) {
        if (endsWith(filename, ".giv"))
            filename = filename.substr(filename.size() - 4);

        outnode.open(filename + ".Infrastructure-Node.giv");
        if (!outnode.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

        outnodegeo.open(filename + ".Infrastructure-Node.giv.geo");
        if (!outnodegeo.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

        outlink.open(filename + ".Infrastructure-Link.giv");
        if (!outlink.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }
    }

    void finalize() {
        // no op
    }

    void writeHeader(int, int) {
        outnode << "# node-id; name; x-coordinate; y-coordinate; possible-stop-for-modalities\n";
        outnodegeo << "# node-id; latitude; longitude\n";
        outlink << "# link-id; left-node-id; right-node-id; length; capacity; modalities; modalities-direction\n";
    }

    void startVertex(const int v) {
        currentVertex = v + 1; // Vertex IDs are 1-based in DIMACS format.
    }

    template<typename Attr, typename DUMMY = void>
    struct SetCurrentVertexAttributeValue {
        static void set(LinTimInfrastructureExporter&, const typename Attr::Type&) {
            // no op
        }
    };

    template<typename DUMMY>
    struct SetCurrentVertexAttributeValue<LatLngAttribute, DUMMY> {
        static void set(LinTimInfrastructureExporter& exporter, const typename LatLngAttribute::Type& value) {
            exporter.currentVertexLatLng = value;
        }
    };

    // Set the given attribute value for the current vertex.
    // Default implementation does nothing.
    template<typename Attr>
    void setCurrentVertexAttributeValue(const typename Attr::Type& value) {
        SetCurrentVertexAttributeValue<Attr>::set(*this, value);
    }

    void finalizeVertex() {
        outnode
        << currentVertex << "; "
        << currentVertex << "; "
        << currentVertexLatLng.lngInDeg() << "; "
        << currentVertexLatLng.latInDeg() << "; "
        << "[bus]" << "\n";

        outnodegeo
        << currentVertex << "; "
        << currentVertexLatLng.latInDeg() << "; "
        << currentVertexLatLng.lngInDeg() << "\n";
    }

    void startOutEdges(const int /*tail*/) {}

    void finalizeOutEdges(const int /*tail*/) {}

    void startEdge(const int tail, const int head) {
        currentEdgeTail = tail + 1; // Vertex IDs are 1-based in LinTim format.
        currentEdgeHead = head + 1;
    }

    template<typename Attr, typename DUMMY = void>
    struct SetCurrentEdgeAttributeValue {
        static void set(LinTimInfrastructureExporter&, const typename Attr::Type&) {
            // no op
        }
    };

    // template<typename DUMMY>
    // struct SetCurrentEdgeAttributeValue<TravelTimeAttribute, DUMMY> {
    //     static void set(LinTimInfrastructureExporter& exporter, const typename TravelTimeAttribute::Type& value) {
    //         exporter.currentTravelTime = value;
    //     }
    // };

    template<typename DUMMY>
    struct SetCurrentEdgeAttributeValue<LengthAttribute, DUMMY> {
        static void set(LinTimInfrastructureExporter& exporter, const typename LengthAttribute::Type& value) {
            exporter.currentLength = value;
        }
    };

    template<typename DUMMY>
    struct SetCurrentEdgeAttributeValue<CapacityAttribute, DUMMY> {
        static void set(LinTimInfrastructureExporter& exporter, const typename CapacityAttribute::Type& value) {
            exporter.currentCapacity = value;
        }
    };

    // Set the given attribute value for the current edge.
    // Default implementation does nothing.
    template<typename Attr>
    void setCurrentEdgeAttributeValue(const typename Attr::Type& value) {
        SetCurrentEdgeAttributeValue<Attr>::set(*this, value);
    }

    void finalizeEdge() {
        static int linkId = 0;
        outlink
        << (++linkId) << "; "
        << currentEdgeTail << "; "
        << currentEdgeHead << "; "
        << currentLength << "; "
        << currentCapacity << "; "
        << "[bus]" << "; "
        << "[BOTH, BOTH]" << "\n";
    }

private:

    std::ofstream outnode; // Output stream for LinTim infrastructure node
    std::ofstream outnodegeo; // Output stream for LinTim infrastructure node geo
    std::ofstream outlink; // Output stream for LinTim infrastructure link

    int currentVertex;
    typename LatLngAttribute::Type currentVertexLatLng;

    int currentEdgeTail;
    int currentEdgeHead;
    // typename TravelTimeAttribute::Type currentTravelTime;
    typename LengthAttribute::Type currentLength;
    typename CapacityAttribute::Type currentCapacity;
};
