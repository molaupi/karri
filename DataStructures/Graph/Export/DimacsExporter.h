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
#include <iostream>
#include "Tools/StringHelpers.h"

#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"

// WeightAttrT can be any graph attribute with a scalar type, e.g. TravelTimeAttribute or LengthAttribute.
// CoordAttributeT is either LatLngAttribute or CoordinateAttribute.
template<typename WeightAttrT, typename CoordAttrT>
class DimacsExporter {
public:
    DimacsExporter(bool /*compress*/) {}

    void ignoreAttribute(const std::string&) {
        // No op, as DimacsExporter always prints only graph topology and coordinates.
    }

    void init(std::string filename) {
        if (endsWith(filename, ".gr"))
            filename = filename.substr(filename.size() - 3);
        if (endsWith(filename, ".co"))
            filename = filename.substr(filename.size() - 3);

        outgr.open(filename + ".gr");
        if (!outgr.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

        outco.open(filename + ".co");
        if (!outco.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

//        outgr << "c weight attribute: " << WeightAttrT::NAME << "\n";
//        outco << "c coordinate attribute: " << CoordAttrT::NAME << "\n";
        if constexpr (std::is_same_v<CoordAttrT, LatLngAttribute>)
            outco << "c Coordinates (longitude, latitude) in WGS84 system stored as integers with unit 1/" << LatLng::PRECISION << " degrees\n";
    }

    void writeHeader(int numVertices, int numEdges) {
        outgr << "p sp " << numVertices << " " << numEdges << "\n";
        outco << "p aux sp co " << numVertices << "\n";
    }

    void startVertex(const int v) {
        currentVertex = v + 1; // Vertex IDs are 1-based in DIMACS format.
    }

    template<typename Attr, typename DUMMY = void>
    struct SetCurrentVertexAttributeValue {
        static void set(DimacsExporter&, const typename Attr::Type&) {
            // no op
        }
    };

    template<typename DUMMY>
    struct SetCurrentVertexAttributeValue<CoordAttrT, DUMMY> {
        static void set(DimacsExporter& exporter, const typename CoordAttrT::Type& value) {
            exporter.currentVertexCoord = value;
        }
    };

    // Set the given attribute value for the current vertex.
    // Default implementation does nothing.
    template<typename Attr>
    void setCurrentVertexAttributeValue(const typename Attr::Type& value) {
        SetCurrentVertexAttributeValue<Attr>::set(*this, value);
    }

//    // Specialization for chosen coordinate attribute.
//    template<>
//    void setCurrentVertexAttributeValue<CoordAttrT>(const typename CoordAttrT::Type& coord) {
//        currentVertexCoord = coord;
//    }

    void finalizeVertex() requires (std::is_same_v<CoordAttrT, LatLngAttribute>) {
        outco << "v " << currentVertex << " " << currentVertexCoord.longitude() << " " << currentVertexCoord.latitude() << "\n";
    }

    void finalizeVertex() requires (std::is_same_v<CoordAttrT, CoordinateAttribute>) {
        outco << "v " << currentVertex << " " << currentVertexCoord.x() << " " << currentVertexCoord.y() << "\n";
    }

    void startEdge(const int tail, const int head) {
        currentEdgeTail = tail + 1; // Vertex IDs are 1-based in DIMACS format.
        currentEdgeHead = head + 1;
    }

    template<typename Attr, typename DUMMY = void>
    struct SetCurrentEdgeAttributeValue {
        static void set(DimacsExporter&, const typename Attr::Type&) {
            // no op
        }
    };

    template<typename DUMMY>
    struct SetCurrentEdgeAttributeValue<WeightAttrT, DUMMY> {
        static void set(DimacsExporter& exporter, const typename CoordAttrT::Type& value) {
            exporter.currentEdgeWeight = value;
        }
    };

    // Set the given attribute value for the current edge.
    // Default implementation does nothing.
    template<typename Attr>
    void setCurrentEdgeAttributeValue(const typename Attr::Type& value) {
        SetCurrentVertexAttributeValue<Attr>::set(*this, value);
    }

//    // Write the given attribute value for the given edge to file.
//    // Default implementation does nothing.
//    template<typename Attr>
//    void setCurrentEdgeAttributeValue(const typename Attr::Type& /*value*/) {}
//
//    template<>
//    void setCurrentEdgeAttributeValue<WeightAttrT>(const typename WeightAttrT::Type& weight) {
//        currentEdgeWeight = weight;
//    }

    void finalizeEdge() {
        outgr << "a " << currentEdgeTail << " " << currentEdgeHead << " " << currentEdgeWeight << "\n";
    }

private:

    std::ofstream outgr; // Output stream for graph topology file
    std::ofstream outco; // Output stream for coordinates file

    int currentVertex;
    typename CoordAttrT::Type currentVertexCoord;

    int currentEdgeTail;
    int currentEdgeHead;
    typename WeightAttrT::Type currentEdgeWeight;
};
