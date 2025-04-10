/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <array>
#include <vector>
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/Containers/StampedDistanceLabelContainer.h"

template<typename SourceGraphT, typename TargetGraphT, typename WeightT, typename LabelSetT,
        template<typename> class DistanceLabelContainerT = StampedDistanceLabelContainer>
class PHASTQuery {

    static constexpr int K = LabelSetT::K;
    using DistanceLabel = typename LabelSetT::DistanceLabel;
    using LabelMask = typename LabelSetT::LabelMask;

public:
    PHASTQuery(const SourceGraphT& sourceGraph, const TargetGraphT& targetGraph, const std::vector<int>& vertexMapping)
            : sourceGraph(sourceGraph), targetGraph(targetGraph), vertexMapping(vertexMapping),
              upwardSearch(sourceGraph),
              distances(0),
              meetingVertices(0) {}

    void run(const std::array<int, K>& sources, const std::array<int, K>& offsets = {}) {
        sanityCheckTargetGraphValidity();
        distances.resize(targetGraph.numVertices() + 1);
        meetingVertices.resize(targetGraph.numVertices() + 1);
        upwardSearch.runWithOffset(sources, offsets);
        numVerticesSettled = upwardSearch.getNumVerticesSettled();
        numEdgesRelaxed = upwardSearch.getNumEdgeRelaxations();
        initDownwardSweep();
        runDownwardSweep();
    }

    int getDistance(const int v, const int i = 0) {
        KASSERT(distances[targetGraph.numVertices()][i] == INFTY);
        return distances[vertexMapping[v]][i];
    }

    DistanceLabel getDistances(const int v) {
        KASSERT(allSet(distances[targetGraph.numVertices()] == INFTY));
        return distances[vertexMapping[v]];
    }

    int getMeetingVertex(const int v, const int i = 0) {
        KASSERT(meetingVertices[targetGraph.numVertices()][i] == INVALID_VERTEX);
        return meetingVertices[vertexMapping[v]][i];
    }

    DistanceLabel getMeetingVertices(const int v) {
        KASSERT(allSet(meetingVertices[targetGraph.numVertices()] == INVALID_VERTEX));
        return meetingVertices[vertexMapping[v]];
    }

    int getNumVerticesSettled() const {
        return numVerticesSettled;
    }

    int getNumEdgeRelaxations() const {
        return numEdgesRelaxed;
    }

private:

    void initDownwardSweep() {
        KASSERT(vertexMapping.size() == sourceGraph.numVertices());
        KASSERT(distances.size() == targetGraph.numVertices() + 1);
        distances.init();
        FORALL_VERTICES(sourceGraph, v) {
            const auto w = vertexMapping[v];
            KASSERT(w >= 0 && w < targetGraph.numVertices() + 1);
            distances[w] = upwardSearch.getDistances(v);
            meetingVertices[w] = v;
        }

        // All vertices which are not present in target graph have distance INFTY.
        distances[targetGraph.numVertices()] = INFTY;
        meetingVertices[targetGraph.numVertices()] = INVALID_VERTEX;
    }

    void runDownwardSweep() {
        // Vertices in target graph are ordered by decreasing rank. Downward sweep simply settles vertices in order.
        for (int v = 0; v < targetGraph.numVertices(); ++v) {
            ++numVerticesSettled;
            auto& distAtV = distances[v];
            auto& meetingVerticesAtV = meetingVertices[v];
            FORALL_INCIDENT_EDGES(targetGraph, v, e) {
                ++numEdgesRelaxed;
                const auto w = targetGraph.edgeHead(e);
                const auto distViaW = distances[w] + targetGraph.template get<WeightT>(e);
                const auto improved = distViaW < distAtV;
                if (anySet(improved)) {
                    distAtV.setIf(distViaW, improved);
                    meetingVerticesAtV.setIf(meetingVertices[w], improved);
                }
            }
        }
    }

    void sanityCheckTargetGraphValidity() {
        KASSERT(vertexMapping.size() == sourceGraph.numVertices());
        int numValidMapping = 0;
        FORALL_VERTICES(sourceGraph, v) {
            const auto w = vertexMapping[v];
            numValidMapping += (w < targetGraph.numVertices());
            KASSERT(w >= 0 && w < targetGraph.numVertices() + 1);
        }
        KASSERT(numValidMapping == targetGraph.numVertices());
    }



    const SourceGraphT &sourceGraph;
    const TargetGraphT &targetGraph;
    // Maps vertex IDs of source graph to vertex IDs of target graph. Vertices v with no equivalent in the targetGraph
    // should have vertexMapping[v] == targetGraph.numVertices().
    const std::vector<int>& vertexMapping;

    Dijkstra<SourceGraphT, WeightT, LabelSetT, dij::NoCriterion, dij::NoCriterion, DistanceLabelContainerT> upwardSearch;

    DistanceLabelContainerT<DistanceLabel> distances;
    DistanceLabelContainerT<DistanceLabel> meetingVertices;

    // Stats on last call to run()
    int numVerticesSettled;
    int numEdgesRelaxed;

};