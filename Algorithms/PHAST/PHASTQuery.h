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
#include "Algorithms/Dijkstra/DagShortestPaths.h"
#include "DataStructures/Labels/Containers/StampedDistanceLabelContainer.h"
#include "RPHASTSelectionPhase.h"

template<typename SourceGraphT, typename WeightT, typename LabelSetT,
        bool StoreMeetingVertices = false,
        template<typename> class DistanceLabelContainerT = StampedDistanceLabelContainer>
class PHASTQuery {

    static constexpr int K = LabelSetT::K;
    using DistanceLabel = typename LabelSetT::DistanceLabel;
    using LabelMask = typename LabelSetT::LabelMask;


    struct SetDistanceInDownwardArray {

        SetDistanceInDownwardArray(DistanceLabelContainerT<DistanceLabel> &downwardArray)
                : downwardArray(downwardArray), sourceToTargetMapping(nullptr) {}

        template<typename DistLabelT, typename DistLabelContT>
        bool operator()(const int v, DistLabelT &upDistToV, const DistLabelContT &) {
            downwardArray[sourceToTargetMapping->at(v)] = upDistToV;
            return false;
        }


        void setSourceToTargetMapping(std::vector<int> const *mapping) {
            sourceToTargetMapping = mapping;
        }

    private:
        DistanceLabelContainerT<DistanceLabel> &downwardArray;
        std::vector<int> const *sourceToTargetMapping;

    };

public:
    PHASTQuery(const SourceGraphT &sourceGraph)
            : sourceGraph(sourceGraph),
              distances(0),
              meetingVertices(0),
              upwardSearch(sourceGraph, SetDistanceInDownwardArray(distances)) {}

    void run(const RPHASTSelection& selection, const std::array<int, K> &sources, const std::array<int, K> &offsets = {}) {
        sanityCheckTargetGraphValidity(selection);
        runUpwardSearchAndInitializeDownwardDistances(selection, sources, offsets);
        runDownwardSweep(selection);
    }

    // Returns distance from i-th source to vertex v found in last call to run().
    // Vertex ID of v has to be ID in subgraph of selection passed to last call of run().
    int getDistance(const int v, const int i = 0) const {
        KASSERT(v >= 0 && v < distances.size());
        return distances.readDistance(v)[i];
    }

    // Returns distance from all K sources to vertex v found in last call to run().
    // Vertex ID of v has to be ID in subgraph of selection passed to last call of run().
    DistanceLabel getDistances(const int v) const {
        KASSERT(v >= 0 && v < distances.size());
        return distances.readDistance(v);
    }

    int getMeetingVertex(const int v, const int i = 0) const requires StoreMeetingVertices {
        KASSERT(v >= 0 && v < meetingVertices.size());
        return meetingVertices[v][i];
    }

    DistanceLabel getMeetingVertices(const int v) const requires StoreMeetingVertices {
        KASSERT(v >= 0 && v < meetingVertices.size());
        return meetingVertices[v];
    }

    int getNumVerticesSettled() const {
        return numVerticesSettled;
    }

    int getNumEdgeRelaxations() const {
        return numEdgesRelaxed;
    }

private:

    void runUpwardSearchAndInitializeDownwardDistances(const RPHASTSelection& selection,
                                                       const std::array<int, K> &sources,
                                                       const std::array<int, K> &offsets = {}) {
        KASSERT(selection.fullToSubMapping.size() == sourceGraph.numVertices());
        distances.resize(selection.subGraph.numVertices() + 1);
        distances.init();
        if constexpr (StoreMeetingVertices) {
            meetingVertices.resize(selection.subGraph.numVertices() + 1);
        }

        upwardSearch.getPruningCriterion().setSourceToTargetMapping(&selection.fullToSubMapping);
        upwardSearch.runWithOffset(sources, offsets);

        // All vertices which are not present in target graph have distance INFTY.
        distances[selection.subGraph.numVertices()] = INFTY;

        numVerticesSettled = upwardSearch.getNumVerticesSettled();
        numEdgesRelaxed = upwardSearch.getNumEdgeRelaxations();
    }

    void runDownwardSweep(const RPHASTSelection& selection) {
        // Vertices in target graph are ordered by decreasing rank. Downward sweep simply settles vertices in order.
        const int numVertices = selection.subGraph.numVertices();
        for (int v = 0; v < numVertices; ++v) {
            ++numVerticesSettled;
            auto &distAtV = distances[v];

            if constexpr (StoreMeetingVertices) {
                meetingVertices[v] = selection.subToFullMapping[v];
            }

            // Relax all incoming edges to finalize the distances and meeting vertices at v.
            FORALL_INCIDENT_EDGES(selection.subGraph, v, e) {
                ++numEdgesRelaxed;
                const auto w = selection.subGraph.edgeHead(e);
                // TODO: There could be a way to mark vertices as irrelevant based on leeways and known best costs.
                //  After settling w, w is irrelevant if:
                //  1. Distance to w is guaranteed to lead to higher costs then best known.
                //  2. Distance to w is larger than largest remaining leeway at w.
                //  Condition 1 may not help prune very much and condition 2 requires overhead in tracking maximum
                //  remaining leeway at every vertex. This would require replacing BFS for target selection with a
                //  topological upward search rooted at every stop. However, we run target selection pretty rarely
                //  so the overhead may be worth it. This could also help to prune the target graph: Every vertex
                //  with maximum remaining leeway of less than 0 does not have to be part of target graph.
                //  Irrelevant vertices can be treated as if having distance INFTY. We can store a isIrrelevant flag
                //  for every vertex and when relaxing edge (w, v) we can check if w is irrelevant. If it is, we
                //  don't have to perform most of the work (reading distance, adding weight, updating distance).
                KASSERT(!distances.isStale(w));
                const auto distToW = distances.readDistanceWithoutStaleCheck(w);
                const auto distViaW = distToW + selection.subGraph.template get<WeightT>(e);
                if constexpr (StoreMeetingVertices) {
                    const auto improved = distViaW < distAtV;
                    if (anySet(improved)) {
                        distAtV.setIf(distViaW, improved);
                        meetingVertices[v].setIf(meetingVertices[w], improved);
                    }
                } else {
                    distAtV.min(distViaW);
                }
            }
        }
    }

    void sanityCheckTargetGraphValidity(const RPHASTSelection& selection) {
        KASSERT(selection.subToFullMapping.size() == selection.subGraph.numVertices());
        KASSERT(selection.fullToSubMapping.size() == sourceGraph.numVertices());
        int numValidMapping = 0;
        FORALL_VERTICES(sourceGraph, v) {
            const auto w = selection.fullToSubMapping[v];
            numValidMapping += (w < selection.subGraph.numVertices());
            KASSERT(w >= 0 && w < selection.subGraph.numVertices() + 1);
        }
        KASSERT(numValidMapping == selection.subGraph.numVertices());
    }


    const SourceGraphT &sourceGraph;


    DistanceLabelContainerT<DistanceLabel> distances;
    AlignedVector<DistanceLabel> meetingVertices;

    using UpwardSearch = DagShortestPaths<SourceGraphT, WeightT, LabelSetT, SetDistanceInDownwardArray, DistanceLabelContainerT>;
    UpwardSearch upwardSearch;


    // Stats on last call to run()
    int numVerticesSettled;
    int numEdgesRelaxed;

};