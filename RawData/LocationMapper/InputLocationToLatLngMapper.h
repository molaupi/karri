/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


#include "DataStructures/Geometry/CoordinateTransformation.h"

namespace input_location_to_lat_lng {

    // Transforms a given vertex to its latitude/longitude coordinates.
    template<typename GraphT>
    struct VertexToLatLngMapper {


        static_assert(GraphT::template has<LatLngAttribute>() &&
                      "Graph needs to contain information about LatLng attribute.");

        // Input is a vertex ID
        using InputType = int;

        VertexToLatLngMapper(const GraphT& graph) : graph(graph) {}

        LatLng operator()(const InputType vertexId) {
            return graph.latLng(vertexId);
        }

    private:
        const GraphT &graph;
    };

    // Transforms a given edge to the latitude/longitude coordinates of its head vertex.
    template<typename GraphT>
    struct EdgeToLatLngMapper {
        static_assert(GraphT::template has<LatLngAttribute>() &&
                "Graph needs to contain information about LatLng attribute.");

        // Input is an edge ID
        using InputType = int;


        EdgeToLatLngMapper(const GraphT& graph) : graph(graph) {}

        LatLng operator()(const InputType& edgeId) {
            return graph.latLng(graph.edgeHead(edgeId));
        }

    private:
        const GraphT &graph;
    };

    // Identity Mapper
    struct LatLngToLatLngMapper {
        using InputType = LatLng;
        LatLng operator()(const LatLng &latLng) {
            return latLng;
        }
    };

    // Transforms EPSG-31467 coordinates to latitude/longitude coordinates
    struct Epsg31467ToLatLngMapper {

        static constexpr int EPSG_CODE = 31467;
        // Input is point of X = easting and Y = northing coordinates in EPSG-31467
        using InputType = std::pair<double, double>;

        Epsg31467ToLatLngMapper() : trans(EPSG_CODE, CoordinateTransformation::WGS_84) {}

        LatLng operator()(const InputType &coordinates) {
            double lng, lat;
            trans.forward(coordinates.first, coordinates.second, lng, lat);
            return {lat, lng};
        }

    private:
        CoordinateTransformation trans;
    };

}