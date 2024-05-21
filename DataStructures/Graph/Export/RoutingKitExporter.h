/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <routingkit/vector_io.h>

class RoutingKitExporter {
public:
    RoutingKitExporter(const bool /* compress */) {}


    void setBaseOutName(const std::string &newBaseOutName) {
        baseOutName = newBaseOutName;
    }

    void ignoreAttribute(const std::string &attrName) {
        attrsToIgnore.push_back(attrName);
    }

    template<typename OutEdgeRangesT, typename EdgeHeadsT>
    void writeTopology(const OutEdgeRangesT &outEdges, const EdgeHeadsT &edgeHeads) {
        writeFirstOut(outEdges);
        writeHead(edgeHeads);
    }

    // Overload for POD attribute types
    template<bool IsVertexAttr,
            typename AttrType,
            typename = std::enable_if<std::is_pod_v<AttrType>>>
    void
    writeAttribute(const AlignedVector<AttrType> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        const auto &outName = baseOutName + attrName;
        // RoutingKit::save_vector expects vector with default allocator so transform it.
        std::vector<AttrType> transformed(values.begin(), values.end());
        if constexpr (IsVertexAttr)
            transformed.push_back(AttrType()); // RoutingKit expects vertex attribute vectors to have length n + 1
        RoutingKit::save_vector(outName, transformed);
    }

    // Overload for LatLng attribute type
    template<bool IsVertexAttr = true>
    void writeAttribute(const AlignedVector<LatLng> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        std::vector<float> lng(values.size());
        std::vector<float> lat(values.size());
        for (int i = 0; i < values.size(); ++i) {
            lng[i] = static_cast<float>(values[i].lngInDeg());
            lat[i] = static_cast<float>(values[i].latInDeg());
        }
        if constexpr (IsVertexAttr) {
            lng.push_back(0.0f); // RoutingKit expects vertex attribute vectors to have length n + 1
            lat.push_back(0.0f);
        }
        const auto &lngOutName = baseOutName + attrName + ".longitude";
        const auto &latOutName = baseOutName + attrName + ".latitude";
        RoutingKit::save_vector(lngOutName, lng);
        RoutingKit::save_vector(latOutName, lat);
    }

    // Overload for Point attribute type
    template<bool IsVertexAttr = true>
    void writeAttribute(const AlignedVector<Point> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        std::vector<int> x(values.size());
        std::vector<int> y(values.size());
        for (int i = 0; i < values.size(); ++i) {
            x[i] = values[i].x();
            y[i] = values[i].y();
        }
        if constexpr (IsVertexAttr) {
            x.push_back(0); // RoutingKit expects vertex attribute vectors to have length n + 1
            y.push_back(0);
        }
        const auto &xOutName = baseOutName + attrName + ".x";
        const auto &yOutName = baseOutName + attrName + ".y";
        RoutingKit::save_vector(xOutName, x);
        RoutingKit::save_vector(yOutName, y);
    }

    // Overload for OsmRoadCategory attribute type
    template<bool IsVertexAttr = false>
    void writeAttribute(const AlignedVector<OsmRoadCategory> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        std::vector<std::string> roadCatNames(values.size());
        for (int i = 0; i < values.size(); ++i)
            roadCatNames[i] = nameOfOsmRoadCategory(values[i]);
        if constexpr (IsVertexAttr)
            roadCatNames.emplace_back(""); // RoutingKit expects vertex attribute vectors to have length n + 1
        const auto &outName = baseOutName + "." + attrName;
        RoutingKit::save_vector(outName, roadCatNames);
    }

    // Overload for std::pair<T1, T2> attribute type
    template<bool IsVertexAttr, typename T1, typename T2>
    void writeAttribute(const AlignedVector<std::pair<T1, T2>> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        static_assert(std::is_trivially_copyable_v<T1> && std::is_trivially_copyable_v<T2>);
        std::vector<T1> first;
        std::vector<T2> second;
        for (int i = 0; i < values.size(); ++i) {
            first.emplace_back(values[i].first);
            second.emplace_back(values[i].second);
        }
        const auto &firstName = attrName + ".first";
        const auto &secondName = attrName + ".second";
        writeAttribute<IsVertexAttr>(first, firstName);
        writeAttribute<IsVertexAttr>(second, secondName);
    }

    // Overload for XatfRoadCategoryTypeAttribute
    template<bool = false>
    void writeAttribute(const AlignedVector<XatfRoadCategory> &, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        throw std::invalid_argument("Cannot write " + attrName + " to RoutingKit format.");
    }

    // Overload for RoadGeometryAttribute
    template<bool = false>
    void writeAttribute(const AlignedVector<std::vector<LatLng>> &, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        throw std::invalid_argument("Cannot write " + attrName + " to RoutingKit format.");
    }

private:

    template<typename OutEdgeRangesT>
    void writeFirstOut(const OutEdgeRangesT &outEdgeRanges) {
        const auto &outName = baseOutName + "first_out";
        std::vector<unsigned> firstOut(outEdgeRanges.size() + 1);
        firstOut[0] = 0;
        for (int v = 0; v < outEdgeRanges.size(); ++v)
            firstOut[v + 1] = static_cast<unsigned>(outEdgeRanges[v].last());
        RoutingKit::save_vector(outName, firstOut);
    }

    template<typename EdgeHeadsT>
    void writeHead(const EdgeHeadsT &edgeHeads) {
        const auto &outName = baseOutName + "head";
        std::vector<unsigned> head(edgeHeads.size());
        for (int e = 0; e < edgeHeads.size(); ++e)
            head[e] = static_cast<unsigned>(edgeHeads[e]);
        RoutingKit::save_vector(outName, head);
    }


    std::string baseOutName;
    std::vector<std::string> attrsToIgnore;

};