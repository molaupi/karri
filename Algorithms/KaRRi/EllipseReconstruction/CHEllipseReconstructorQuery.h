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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"
#include "DataStructures/Containers/TimestampedVector.h"

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename EllipticBucketsEnvironmentT, typename LabelSet, typename WeightT = TraversalCostAttribute>
    class CHEllipseReconstructorQuery {

        using DistanceLabel = typename LabelSet::DistanceLabel;
        using LabelMask = typename LabelSet::LabelMask;
        static constexpr int K = LabelSet::K;

    public:

        CHEllipseReconstructorQuery(const CH &ch,
                                    const typename CH::SearchGraph &reverseDownGraph,
                                    const typename CH::SearchGraph &forwardUpGraph,
                                    const Permutation &topDownRankPermutation,
                                    const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                    const RouteState &routeState)
                : ch(ch),
                  numVertices(reverseDownGraph.numVertices()),
                  reverseDownGraph(reverseDownGraph),
                  forwardUpGraph(forwardUpGraph),
                  topDownRankPermutation(topDownRankPermutation),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  hasBeenInitializedInToSearch(numVertices),
                    hasBeenInitializedInFromSearch(numVertices),
                  relevantInToSearch(numVertices),
                  relevantInFromSearch(numVertices) {
            KASSERT(reverseDownGraph.numVertices() == numVertices);
            KASSERT(forwardUpGraph.numVertices() == numVertices);
            verticesInAnyEllipse.reserve(numVertices);
        }

        std::vector<std::vector<VertexInEllipse>> run(const std::vector<int> &stopIds,
                                                      int &numVerticesSettled, int &numEdgesRelaxed,
                                                      int64_t &initTime,
                                                      int64_t &topoSearchTime,
                                                      int64_t &postprocessTime) {
            KASSERT(stopIds.size() <= K);

            Timer timer;
            initializeDistanceArrays();
            DistanceLabel leeways = 0;
            const auto numEllipses = stopIds.size();
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesForStopBasedOnBuckets(stopIds[i], i);
                leeways[i] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            timer.restart();
            for (int r = 0; r < numVertices; ++r) {
                ++numVerticesSettled;
                settleVertexInTopodownSearch(r, leeways, numEdgesRelaxed);
            }
            topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();

            // Accumulate result per ellipse
            timer.restart();
            std::vector<std::vector<VertexInEllipse>> ellipses(numEllipses);
            for (auto &ellipse: ellipses)
                ellipse.reserve(verticesInAnyEllipse.size());
            for (const auto &r: verticesInAnyEllipse) {
                const auto originalRank = numVertices - r - 1; // Reverse permutation in search graphs
                const int vertex = ch.contractionOrder(originalRank);

                const auto &distToVertex = distTo[r];
                const auto &distFromVertex = distFrom[r];
                const auto breaksLeeway = distToVertex + distFromVertex > leeways;
                KASSERT(!allSet(breaksLeeway));
                for (int j = 0; j < numEllipses; ++j) {
                    if (!breaksLeeway[j]) {
                        KASSERT(distToVertex[j] < INFTY && distFromVertex[j] < INFTY);
                        ellipses[j].emplace_back(vertex, distToVertex[j], distFromVertex[j]);
                    }
                }
            }
            for (auto &ellipse: ellipses)
                ellipse.shrink_to_fit();
            postprocessTime += timer.elapsed<std::chrono::nanoseconds>();

            return ellipses;
        }


    private:

        void initializeDistanceArrays() {
            KASSERT(distTo.size() == distFrom.size());
            relevantInToSearch.reset();
            relevantInFromSearch.reset();
            hasBeenInitializedInToSearch.reset();
            hasBeenInitializedInFromSearch.reset();

            verticesInAnyEllipse.clear();
        }

        void initializeDistancesForStopBasedOnBuckets(const int stopId, const int ellipseIdx) {
            KASSERT(ellipseIdx >= 0 && ellipseIdx < K);
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto ranksWithSourceBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithSourceBucketEntries(vehId, stopIdx,
                                                                             enumerateBucketEntriesSearchSpace);
            const auto ranksWithTargetBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithTargetBucketEntries(vehId, stopIdx + 1,
                                                                             enumerateBucketEntriesSearchSpace);

            for (const auto &e: ranksWithSourceBucketEntries) {

                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];
                if (!hasBeenInitializedInToSearch.isSet(r)) {
                    distTo[r] = INFTY;
                    hasBeenInitializedInToSearch.set(r);
                }
//                relevantInToSearch.set(r);
                distTo[r][ellipseIdx] = e.distance;
            }

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];
                if (!hasBeenInitializedInFromSearch.isSet(r)) {
                    distFrom[r] = INFTY;
                    hasBeenInitializedInFromSearch.set(r);
                }
//                relevantInFromSearch.set(r);
                distFrom[r][ellipseIdx] = e.distance;
            }
        }

        void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway,
                                          int &numEdgesRelaxed) {

            static const DistanceLabel InftyLabel = DistanceLabel(INFTY);

            auto &distToV = distTo[v];
            auto &distFromV = distFrom[v];
            // Vertices with initial entries are starting points of search. Other vertices are initialized to INFTY
            // and only settled if they have relevant upward neighbors.
            bool hasRelevantUpwardNeighborsInToSearch = hasBeenInitializedInToSearch.isSet(v);
            bool hasRelevantUpwardNeighborsInFromSearch = hasBeenInitializedInFromSearch.isSet(v);
            if (!hasRelevantUpwardNeighborsInToSearch)
                distToV = InftyLabel;
            if (!hasRelevantUpwardNeighborsInFromSearch)
                distFromV = InftyLabel;

            FORALL_INCIDENT_EDGES(reverseDownGraph, v, e) {
                const auto head = reverseDownGraph.edgeHead(e);
                hasRelevantUpwardNeighborsInToSearch |= relevantInToSearch.isSet(head);
            }

            FORALL_INCIDENT_EDGES(forwardUpGraph, v, e) {
                const auto head = forwardUpGraph.edgeHead(e);
                hasRelevantUpwardNeighborsInFromSearch |= relevantInFromSearch.isSet(head);
            }

            // Skip if irrelevant on both sides.
            if (!hasRelevantUpwardNeighborsInToSearch && !hasRelevantUpwardNeighborsInFromSearch) {
                return;
            }

            if (hasRelevantUpwardNeighborsInToSearch) {
                relaxIncomingEdgesInToSearch(v, distToV, numEdgesRelaxed);
            }

            if (hasRelevantUpwardNeighborsInFromSearch) {
                relaxIncomingEdgesInFromSearch(v, distFromV, numEdgesRelaxed);
            }

            const auto sum = distToV + distFromV;
            const auto sumGreaterLeeway = sum > leeway;
            const auto sumSmallerInfty = sum < InftyLabel;
            const auto toGreaterLeeway = distToV > leeway;
            const auto fromGreaterLeeway = distFromV > leeway;

            // We can prune if for all stop pairs one of the following holds:
            // 1. Both distance to v and from v are greater than the leeway (including unknown distances = INFTY).
            // 2. Both distance to v and from v are known and the sum breaks the leeway.
            const auto canPrune = allSet((toGreaterLeeway & fromGreaterLeeway) | (sumSmallerInfty & sumGreaterLeeway));
            if (canPrune)
                return;

            // If vertex is in any ellipse, store it
            const bool allSumGreaterLeeway = allSet(sumGreaterLeeway);
            if (!allSumGreaterLeeway) {
                verticesInAnyEllipse.push_back(v);
            }

            if (!allSet(toGreaterLeeway)) {
//                markDownwardNeighborsInToSearchAsPotentiallyRelevant(v);
                relevantInToSearch.set(v);
            }
            if (!allSet(fromGreaterLeeway)) {
//                markDownwardNeighborsInFromSearchAsPotentiallyRelevant(v);
                relevantInFromSearch.set(v);
            }
        }

        inline void relaxIncomingEdgesInToSearch(const int v, DistanceLabel &distToV, int &numEdgesRelaxed) const {
            FORALL_INCIDENT_EDGES(reverseDownGraph, v, e) {
                const auto head = reverseDownGraph.edgeHead(e);

                ++numEdgesRelaxed;
                const auto &distToHead = distTo[head];
                const auto distViaHead = distToHead + reverseDownGraph.template get<WeightT>(e);
                distToV.min(distViaHead);
            }
        }

        inline void relaxIncomingEdgesInFromSearch(const int v, DistanceLabel &distFromV, int &numEdgesRelaxed) const {
            FORALL_INCIDENT_EDGES(forwardUpGraph, v, e) {
                const auto head = forwardUpGraph.edgeHead(e);

                ++numEdgesRelaxed;
                const auto &distFromHead = distFrom[head];
                const auto distViaHead = distFromHead + forwardUpGraph.template get<WeightT>(e);
                distFromV.min(distViaHead);
            }
        }

        const CH &ch;
        const size_t numVertices;
        const CH::SearchGraph &reverseDownGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        const CH::SearchGraph &forwardUpGraph; // Forward upward edges in CH. Vertices ordered by decreasing rank.
        const Permutation &topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        FastResetFlagArray<> hasBeenInitializedInToSearch; // Flags that mark whether vertex distance has been initialized in to search.
        FastResetFlagArray<> hasBeenInitializedInFromSearch; // Flags that mark whether vertex distance has been initialized in from search.
        FastResetFlagArray<> relevantInToSearch; // Flags that mark whether vertex is relevant in to search.
        FastResetFlagArray<> relevantInFromSearch; // Flags that mark whether vertex is relevant in from search.
    };

} // karri
