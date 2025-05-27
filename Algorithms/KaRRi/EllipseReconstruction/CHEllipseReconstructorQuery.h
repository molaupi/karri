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
                  lowestNeighbor(numVertices),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  initialized(numVertices),
                  highestRelInElimTreeBranch(numVertices, -1) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);
            verticesInAnyEllipse.reserve(numVertices);
            for (int v = 0; v < numVertices; ++v) {
                const int lowestNeighborInDown =
                        downGraph.lastEdge(v) == downGraph.firstEdge(v) ? numVertices : downGraph.edgeHead(
                                downGraph.firstEdge(v));
                const int lowestNeighborInUp =
                        upGraph.lastEdge(v) == upGraph.firstEdge(v) ? numVertices : upGraph.edgeHead(
                                upGraph.firstEdge(v));
                lowestNeighbor[v] = std::min(lowestNeighborInDown, lowestNeighborInUp);
            }

        }

        std::vector<std::vector<VertexInEllipse>> run(const std::vector<int> &stopIds,
                                                      int &numVerticesSettled, int &totalNumEdgesRelaxed,
                                                      int &numVerticesInAnyEllipse,
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
                const auto leewayI = routeState.leewayOfLegStartingAt(stopIds[i]);
                initializeDistancesWithSourceBuckets(stopIds[i], i, leewayI, highestInitializedVertex);
                leeways[i] = leewayI;
            }
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesWithTargetBuckets(stopIds[i], i, leeways[i], highestInitializedVertex);
            }
            initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            static constexpr bool TRACK_NUM_EDGES_RELAXED = false;
            timer.restart();
            int numEdgesRelaxed = 0;
            for (int r = 0; r < numVertices; ++r) {
                settleVertexInTopodownSearch<TRACK_NUM_EDGES_RELAXED>(r, leeways, numEdgesRelaxed);
            }
            topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();
            totalNumEdgesRelaxed += numEdgesRelaxed;
            numVerticesSettled += numVertices;
            numVerticesInAnyEllipse += verticesInAnyEllipse.size();

            // Accumulate result per ellipse
            timer.restart();
            const auto ellipses = constructEllipses(numEllipses, leeways);
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

            initialized.reset();

            verticesInAnyEllipse.clear();
        }

        void initializeDistancesWithSourceBuckets(const int stopId, const int ellipseIdx, const int leeway,
                                                  int &highestInitialized) {
            KASSERT(ellipseIdx >= 0 && ellipseIdx < K);
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto ranksWithSourceBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithSourceBucketEntries(vehId, stopIdx,
                                                                             enumerateBucketEntriesSearchSpace);

            for (const auto &e: ranksWithSourceBucketEntries) {

                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];
                KASSERT(e.distance <= leeway);

                if (!initialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                    distFrom[r] = INFTY;
                    initialized.set(r);
                }

                distTo[r][ellipseIdx] = e.distance;
                highestRelInElimTreeBranch[eliminationTree[r]] = -1;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        void initializeDistancesWithTargetBuckets(const int stopId, const int ellipseIdx, const int leeway,
                                                  int &highestInitialized) {
            KASSERT(ellipseIdx >= 0 && ellipseIdx < K);
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto ranksWithTargetBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithTargetBucketEntries(vehId, stopIdx + 1,
                                                                             enumerateBucketEntriesSearchSpace);

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];
                KASSERT(e.distance <= leeway);

                if (!initialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                    distFrom[r] = INFTY;
                    initialized.set(r);
                }

                KASSERT(distFrom[r][ellipseIdx] == INFTY);
                distFrom[r][ellipseIdx] = e.distance;
                highestRelInElimTreeBranch[eliminationTree[r]] = -1;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        template<bool TRACK_NUM_EDGES = false>
        void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway, int &numEdgesRelaxed) {

            auto &distToV = distTo[v];
            auto &distFromV = distFrom[v];
            const bool initV = initialized.isSet(v);
            if (!initV) {
                distToV = INFTY;
                distFromV = INFTY;
            }

            // Propagate information on highest relevant rank along elimination tree edge.
            auto &highestRel = highestRelInElimTreeBranch[v];
            highestRel = highestRelInElimTreeBranch[eliminationTree[v]];
            const bool needToSettle = lowestNeighbor[v] <= highestRel;

            // If there are no relevant distances at upward neighbors (!needToSettleX) and there are no distances
            // from bucket entries (!initV), then skip this vertex.
            if (!initV && !needToSettle)
                return;

            if (needToSettle) {
                FORALL_INCIDENT_EDGES(downGraph, v, e) {
                    const auto head = downGraph.edgeHead(e);

                    if constexpr (TRACK_NUM_EDGES) ++numEdgesRelaxed;
                    const auto &distToHead = distTo[head];
                    const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                    distToV.min(distViaHead);
                }

                FORALL_INCIDENT_EDGES(upGraph, v, e) {
                    const auto head = upGraph.edgeHead(e);

                    if constexpr (TRACK_NUM_EDGES) ++numEdgesRelaxed;
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
                highestRel = v;
                return;
            }
        }

        std::vector<std::vector<VertexInEllipse>>
        constructEllipses(const int numEllipses, const DistanceLabel &leeways) {
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

            return ellipses;
        }

        const CH &ch;
        const size_t numVertices;
        const CH::SearchGraph &downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        const CH::SearchGraph &upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        const Permutation &topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;
        const std::vector<int> &eliminationTree; // Elimination tree with vertices ordered by decreasing rank.

        std::vector<int> lowestNeighbor;

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        // Flags that indicate whether distances have been initialized with bucket entries
        FastResetFlagArray <> initialized;

        // highestRelInElimTreeBranch[v] is the highest rank (permuted, so least important vertex) on the
        // elimination-tree branch of v that is in an ellipse. When settling vertex v, we only
        // have to relax edges with head ranks up to highestRelInElimTreeBranch[v].
        std::vector<int> highestRelInElimTreeBranch;
    };

} // karri
