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
                                    const typename CH::SearchGraph &downGraph,
                                    const typename CH::SearchGraph &upGraph,
                                    const Permutation &topDownRankPermutation,
                                    const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                    const RouteState &routeState,
                                    const std::vector<int> &eliminationTree)
                : ch(ch),
                  numVertices(downGraph.numVertices()),
                  downGraph(downGraph),
                  upGraph(upGraph),
                  topDownRankPermutation(topDownRankPermutation),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  eliminationTree(eliminationTree),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  relevantInToSearch(numVertices),
                  relevantInFromSearch(numVertices),
                  highestRelInElimTreeBranchToSearch(numVertices, -1),
                  highestRelInElimTreeBranchFromSearch(numVertices, -1) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);
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
            int highestInitializedVertex = -1;
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesForStopBasedOnBuckets(stopIds[i], i, highestInitializedVertex);
                leeways[i] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            timer.restart();
            for (int r = 0; r < highestInitializedVertex + 1; ++r) {
                ++numVerticesSettled;
                settleVertexInTopodownSearch(r, leeways, numEdgesRelaxed);
            }
            for (int r = highestInitializedVertex + 1; r < numVertices; ++r) {
                ++numVerticesSettled;
                settleVertexBelowAnyInitial(r, leeways, numEdgesRelaxed);
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
            for (auto &ellipse: ellipses) {
                ellipse.shrink_to_fit();
            }
            postprocessTime += timer.elapsed<std::chrono::nanoseconds>();

            return ellipses;
        }


    private:

        void initializeDistanceArrays() {
            KASSERT(distTo.size() == distFrom.size());
            while (numVertices > distTo.size()) {
                distTo.resize(std::max(1ul, distTo.size() * 2));
                distFrom.resize(std::max(1ul, distFrom.size() * 2));
            }

            relevantInToSearch.reset();
            relevantInFromSearch.reset();

            verticesInAnyEllipse.clear();

            std::fill(highestRelInElimTreeBranchToSearch.begin(), highestRelInElimTreeBranchToSearch.end(), -1);
            std::fill(highestRelInElimTreeBranchFromSearch.begin(), highestRelInElimTreeBranchFromSearch.end(), -1);
        }

        void initializeDistancesForStopBasedOnBuckets(const int stopId, const int ellipseIdx, int& highestInitialized) {
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

                if (!relevantInToSearch.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                }
                relevantInToSearch.set(r);

                distTo[r][ellipseIdx] = e.distance;
                highestInitialized = std::max(highestInitialized, r);
            }

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!relevantInFromSearch.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distFrom[r] = INFTY;
                }
                relevantInFromSearch.set(r);

                // Distances for input ranks are initialized here. Distances of other ranks are initialized
                // on-the-fly later.
                distFrom[r][ellipseIdx] = e.distance;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        inline void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway,
                                          int &numEdgesRelaxed) {

            auto &distToV = distTo[v];
            const bool notInitInToSearch = !relevantInToSearch.isSet(v);
            if (notInitInToSearch) {
                distToV = INFTY;
            }

            auto &distFromV = distFrom[v];
            const bool notInitInFromSearch = !relevantInFromSearch.isSet(v);
            if (notInitInFromSearch) {
                distFromV = INFTY;
            }

            // Propagate information on highest relevant rank along elimination tree edge.
            const auto &parentElimTree = eliminationTree[v];
            auto &highestRelToV = highestRelInElimTreeBranchToSearch[v];
            auto &highestRelFromV = highestRelInElimTreeBranchFromSearch[v];
            highestRelToV = highestRelInElimTreeBranchToSearch[parentElimTree];
            highestRelFromV = highestRelInElimTreeBranchFromSearch[parentElimTree];

            const bool needToSettleInToSearch = downGraph.lastEdge(v) > downGraph.firstEdge(v) &&
                                                downGraph.edgeHead(downGraph.firstEdge(v)) <= highestRelToV;
            const bool needToSettleInFromSearch = upGraph.lastEdge(v) > upGraph.firstEdge(v) &&
                                                  upGraph.edgeHead(upGraph.firstEdge(v)) <= highestRelFromV;

            if (needToSettleInToSearch) {
                FORALL_INCIDENT_EDGES(downGraph, v, e) {
                    const auto head = downGraph.edgeHead(e);

                    ++numEdgesRelaxed;
                    const auto &distToHead = distTo[head];
                    const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                    distToV.min(distViaHead);
                }
            }

            if (needToSettleInFromSearch) {
                FORALL_INCIDENT_EDGES(upGraph, v, e) {
                    const auto head = upGraph.edgeHead(e);

                    ++numEdgesRelaxed;
                    const auto &distFromHead = distFrom[head];
                    const auto distViaHead = distFromHead + upGraph.template get<WeightT>(e);
                    distFromV.min(distViaHead);
                }
            }

            // If vertex is in any ellipse, store it
            const auto sum = distToV + distFromV;
            const auto sumGreaterLeeway = sum > leeway;
            if (!allSet(sumGreaterLeeway)) {
                verticesInAnyEllipse.push_back(v);
                highestRelToV = v;
                highestRelFromV = v;
                return;
            }

            // If for any of the K ellipses, distance from vertex is INFTY and distance to vertex is smaller
            // than the leeway, then the vertex is relevant in the to search.
            if (anySet((distFromV == INFTY) & (distToV <= leeway))) {
                highestRelToV = v;
            }

            // If for any of the K ellipses distance to vertex is INFTY and distance from vertex is smaller
            // than the leeway, then the vertex is relevant in the from search.
            if (anySet((distToV == INFTY) & (distFromV <= leeway))) {
                highestRelFromV = v;
            }
        }

        inline void settleVertexBelowAnyInitial(const int v, const DistanceLabel &leeway,
                                         int &numEdgesRelaxed) {

            KASSERT(!relevantInToSearch.isSet(v) && !relevantInFromSearch.isSet(v));

            auto &distToV = distTo[v];
            distToV = INFTY;

            auto &distFromV = distFrom[v];
            distFromV = INFTY;

            // Propagate information on highest relevant rank along elimination tree edge.
            const auto &parentElimTree = eliminationTree[v];
            auto &highestRelToV = highestRelInElimTreeBranchToSearch[v];
            auto &highestRelFromV = highestRelInElimTreeBranchFromSearch[v];
            highestRelToV = highestRelInElimTreeBranchToSearch[parentElimTree];
            highestRelFromV = highestRelInElimTreeBranchFromSearch[parentElimTree];

            const bool needToSettleInToSearch = downGraph.lastEdge(v) > downGraph.firstEdge(v) &&
                                                downGraph.edgeHead(downGraph.firstEdge(v)) <= highestRelToV;
            const bool needToSettleInFromSearch = upGraph.lastEdge(v) > upGraph.firstEdge(v) &&
                                                  upGraph.edgeHead(upGraph.firstEdge(v)) <= highestRelFromV;

            if (!needToSettleInToSearch && !needToSettleInFromSearch)
                return;

            if (needToSettleInToSearch) {
                FORALL_INCIDENT_EDGES(downGraph, v, e) {
                    const auto head = downGraph.edgeHead(e);

                    ++numEdgesRelaxed;
                    const auto &distToHead = distTo[head];
                    const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                    distToV.min(distViaHead);
                }
            }

            if (needToSettleInFromSearch) {
                FORALL_INCIDENT_EDGES(upGraph, v, e) {
                    const auto head = upGraph.edgeHead(e);

                    ++numEdgesRelaxed;
                    const auto &distFromHead = distFrom[head];
                    const auto distViaHead = distFromHead + upGraph.template get<WeightT>(e);
                    distFromV.min(distViaHead);
                }
            }

            // If vertex is in any ellipse, store it
            const auto sum = distToV + distFromV;
            const auto sumGreaterLeeway = sum > leeway;
            if (!allSet(sumGreaterLeeway)) {
                verticesInAnyEllipse.push_back(v);
                highestRelToV = v;
                highestRelFromV = v;
                return;
            }

            // If for any of the K ellipses, distance from vertex is INFTY and distance to vertex is smaller
            // than the leeway, then the vertex is relevant in the to search.
            if (anySet((distFromV == INFTY) & (distToV <= leeway))) {
                highestRelToV = v;
            }

            // If for any of the K ellipses distance to vertex is INFTY and distance from vertex is smaller
            // than the leeway, then the vertex is relevant in the from search.
            if (anySet((distToV == INFTY) & (distFromV <= leeway))) {
                highestRelFromV = v;
            }
        }


        const CH &ch;
        const size_t numVertices;
        const CH::SearchGraph &downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        const CH::SearchGraph &upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        const Permutation &topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;
        const std::vector<int> &eliminationTree; // Elimination tree with vertices ordered by decreasing rank.

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        FastResetFlagArray<> relevantInToSearch; // Flags that mark whether vertex is relevant in to search.
        FastResetFlagArray<> relevantInFromSearch; // Flags that mark whether vertex is relevant in from search.

        // highestRelInElimTreeBranchToSearch[v]/highestRelInElimTreeFromSearch[v] is the highest rank
        // (permuted, so least important vertex) that is relevant in the to/from search. When settling vertex v, we only
        // have to relax edges with head ranks up to highestRelInElimTreeBranchToSearch[v]/highestRelInElimTreeFromSearch[v].
        std::vector<int> highestRelInElimTreeBranchToSearch;
        std::vector<int> highestRelInElimTreeBranchFromSearch;
    };

} // karri
