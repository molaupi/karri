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

#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "Utils.h"

// Given a latitude/longitude coordinate, finds the closest vertex to that coordinate in a target road network.
// Uses the euclidian distance between latitude/longitude points in a 2-D plane as the distance metric.
// Note that this cannot accurately represent the sphere geometry of the earth which means that a source vertex may
// be mapped to a target vertex that does not have the smallest great-circle distance (distance 'as the crow flies').
//
// May be given an additional eligibility criterion for vertices in the target graph.
template<typename GraphT, typename IsVertexEligibleT = AlwaysEligible, typename LoggerT = NullLogger>
class LatLngToTargetVertexMapper {

public:

    LatLngToTargetVertexMapper(const GraphT &targetGraph,
                               const double maxVertexMatchDist,
                               const IsVertexEligibleT& isVertexEligible = {},
                               Area const *const studyArea = nullptr)
            : targetGraph(targetGraph),
              maxVertexMatchDist(maxVertexMatchDist),
              isVertexEligible(isVertexEligible),
              eligibleVerticesInTar(),
              kdTree(buildKDTree(studyArea)),
              matchLogger(LogManager<LoggerT>::getLogger("vertexmatches.csv",
                                                         "src_lat_lng,"
                                                         "tar_vertex_id,"
                                                         "tar_lat_lng,"
                                                         "tar_osm_node_id,"
                                                         "dist\n")) {}

    // Maps a given LatLng coordinate to the closest vertex in the target network if one exists that is closer
    // than the maximum distance.
    // Returns the vertex in the target graph if a valid mapping is found or INVALID_VERTEX otherwise.
    // Returns false otherwise. In this case v is not changed.
    int mapToTargetVertex(const LatLng& latLng) {
        const auto pointIdx = kdTree.findClosestPoint(Point(latLng.longitude(), latLng.latitude()));
        const auto closestVertexInTar = eligibleVerticesInTar[pointIdx];
        const auto greatCircleDist = targetGraph.latLng(closestVertexInTar).getGreatCircleDistanceTo(latLng);

        logMatch(latLng, closestVertexInTar, greatCircleDist);

        if (greatCircleDist > maxVertexMatchDist)
            return INVALID_VERTEX;
        return closestVertexInTar;
    }


private:

    // Optional: If all vertices lie within a known area, this area can be passed as a parameter for faster mapping.
    KDTree buildKDTree(Area const *const studyArea) {
        std::vector<Point> points;
        if (!studyArea) {
            std::cout << "Building KD-tree for all vertices in target graph..." << std::flush;
            FORALL_VERTICES(targetGraph, u) {
                // All eligible vertices become point in KD-tree.
                if (isVertexEligible(u)) {
                    eligibleVerticesInTar.push_back(u);
                    const auto latLng = targetGraph.latLng(u);
                    points.emplace_back(latLng.longitude(), latLng.latitude());
                }
            }
        } else {
            std::cout << "Building KD-tree for all vertices in study area of target graph..." << std::flush;
            const auto box = studyArea->boundingBox();
            FORALL_VERTICES(targetGraph, u) {
                const auto latLng = targetGraph.latLng(u);
                const Point p(latLng.longitude(), latLng.latitude());
                // All vertices that are in the study area (and passenger accessible if needed) are eligible.
                if (box.contains(p) && studyArea->contains(p) && isVertexEligible(u)) {
                    eligibleVerticesInTar.push_back(u);
                    points.push_back(p);
                }
            }
        }

        KDTree tree(points);
        std::cout << " done.\n";
        return tree;
    }

    void logMatch(const LatLng& srcLatLng, const int vInTar, const double greatCircleDist) {
        matchLogger
                << latLngForCsv(srcLatLng) << ","
                << vInTar << ","
                << latLngForCsv(targetGraph.latLng(vInTar)) << ","
                << targetGraph.osmNodeId(vInTar) << ","
                << greatCircleDist << "\n";
    }

    const GraphT &targetGraph;
    double maxVertexMatchDist;
    IsVertexEligibleT isVertexEligible;

    std::vector<int> eligibleVerticesInTar;
    KDTree kdTree;

    LoggerT &matchLogger;
};