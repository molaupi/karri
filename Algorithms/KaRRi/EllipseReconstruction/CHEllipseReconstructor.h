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
    template<typename CHEnvT, typename EllipticBucketsEnvironmentT, typename WeightT = TraversalCostAttribute,
            typename LoggerT = NullLogger>
    class CHEllipseReconstructor {

        using LabelSet = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>;
//        using LabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using DistanceLabel = typename LabelSet::DistanceLabel;
        using LabelMask = typename LabelSet::LabelMask;
        static constexpr int K = LabelSet::K;

    public:

        CHEllipseReconstructor(const CHEnvT &chEnv, EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                               const RouteState &routeState)
                : ch(chEnv.getCH()),
                  numVertices(ch.downwardGraph().numVertices()),
                  chDownGraph(ch.downwardGraph().getReverseGraph()),
                  chRevUpGraph(ch.upwardGraph().getReverseGraph()),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  topDownRankPermutation(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  reachedInToSearch(numVertices),
                  reachedInFromSearch(numVertices),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                              "num_ellipses,"
                                                              "init_time,"
                                                              "topo_search_time,"
                                                              "postprocess_time,"
                                                              "topo_search_num_vertices_settled,"
                                                              "topo_search_num_edges_relaxed\n")) {
            KASSERT(chDownGraph.numVertices() == numVertices);
            KASSERT(chRevUpGraph.numVertices() == numVertices);
            for (int r = 0; r < numVertices; ++r)
                topDownRankPermutation[r] = numVertices - r - 1;

            chDownGraph.permuteVertices(topDownRankPermutation);
            chRevUpGraph.permuteVertices(topDownRankPermutation);

            verticesInAnyEllipse.reserve(numVertices);
        }

        std::vector<std::vector<VertexInEllipse>>
        getVerticesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds, int &numVerticesSettled,
                                              int &numEdgesRelaxed) {

            numVerticesSettled = 0;
            numEdgesRelaxed = 0;
            if (stopIds.empty())
                return {};

            const size_t numEllipses = stopIds.size();

            Timer timer;
            initializeDistanceArrays(numEllipses);
            AlignedVector<DistanceLabel> leeways(numLabelsPerVertex, 0);
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesForStopBasedOnBuckets(stopIds[i], i);
                leeways[i / K][i % K] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            const auto initTime = timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            timer.restart();
            for (int r = 0; r < numVertices; ++r) {
                ++numVerticesSettled;
                settleVertexInTopodownSearch(r, leeways, numEdgesRelaxed);
            }
            const auto topoSearchTime = timer.elapsed<std::chrono::nanoseconds>();

            // Accumulate result per ellipse
            timer.restart();
            std::vector<std::vector<VertexInEllipse>> ellipses(numEllipses);
            for (const auto &r: verticesInAnyEllipse) {
                const auto originalRank = numVertices - r - 1; // Reverse permutation in search graphs
                const int vertex = ch.contractionOrder(originalRank);
                for (int i = 0; i < numLabelsPerVertex; ++i) {
                    const auto &leewayBatch = leeways[i];
                    const auto &distToBatch = distTo[r * numLabelsPerVertex + i];
                    const auto &distFromBatch = distFrom[r * numLabelsPerVertex + i];
                    const auto breaksLeeway = (distToBatch + distFromBatch > leewayBatch);
                    if (allSet(breaksLeeway))
                        continue;
                    for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                        if (!breaksLeeway[j]) {
                            KASSERT(distToBatch[j] < INFTY && distFromBatch[j] < INFTY);
                            ellipses[i * K + j].emplace_back(vertex, distToBatch[j], distFromBatch[j]);
                        }
                    }
                }
            }
            const auto postprocessTime = timer.elapsed<std::chrono::nanoseconds>();

            logger << numEllipses << "," << initTime << "," << topoSearchTime << "," << postprocessTime << ","
                   << numVerticesSettled << "," << numEdgesRelaxed << "\n";

            return ellipses;
        }


    private:

        void initializeDistanceArrays(const size_t numEllipses) {
            numLabelsPerVertex = numEllipses / K + (numEllipses % K != 0);
            const size_t numEntries = numVertices * numLabelsPerVertex;
            KASSERT(distTo.size() == distFrom.size());
            while (numEntries > distTo.size()) {
                distTo.resize(std::max(1ul, distTo.size() * 2));
                distFrom.resize(std::max(1ul, distFrom.size() * 2));
            }

            reachedInToSearch.reset();
            reachedInFromSearch.reset();

            verticesInAnyEllipse.clear();
        }

        void initializeDistancesForStopBasedOnBuckets(const int stopId, const int ellipseIdx) {
            static const DistanceLabel InftyLabel = DistanceLabel(INFTY);
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto ranksWithSourceBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithSourceBucketEntries(vehId, stopIdx);
            const auto ranksWithTargetBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithTargetBucketEntries(vehId, stopIdx + 1);

            for (const auto &e: ranksWithSourceBucketEntries) {

                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!reachedInToSearch.isSet(r)) {
                    // Initialize all distances at r to INFTY
                    for (int i = 0; i < numLabelsPerVertex; ++i)
                        distTo[r * numLabelsPerVertex + i] = InftyLabel;
                }
                reachedInToSearch.set(r);

                distTo[r * numLabelsPerVertex + ellipseIdx / K][ellipseIdx % K] = e.distance;
            }

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!reachedInFromSearch.isSet(r)) {
                    // Initialize all distances at r to INFTY
                    for (int i = 0; i < numLabelsPerVertex; ++i)
                        distFrom[r * numLabelsPerVertex + i] = InftyLabel;
                }
                reachedInFromSearch.set(r);

                distFrom[r * numLabelsPerVertex + ellipseIdx / K][ellipseIdx % K] = e.distance;
            }
        }

        void settleVertexInTopodownSearch(const int v, const AlignedVector<DistanceLabel> &leeway,
                                          int &numEdgesRelaxed) {

            static const DistanceLabel InftyLabel = DistanceLabel(INFTY);

            if (!reachedInToSearch.isSet(v) && !reachedInFromSearch.isSet(v)) {
                return;
            }

            // If vertex is in any ellipse, store it
            bool allSumGreaterLeeway = true;
            bool allToGreaterLeeway = true;
            bool allFromGreaterLeeway = true;
            bool canPrune = true;
            for (int i = 0; i < numLabelsPerVertex; ++i) {
                const auto &distToV = distTo[v * numLabelsPerVertex + i];
                const auto &distFromV = distFrom[v * numLabelsPerVertex + i];
                KASSERT(allSet(distToV <= InftyLabel));
                KASSERT(allSet(distFromV <= InftyLabel));
                const auto sum = distToV + distFromV;
                allSumGreaterLeeway &= allSet(sum > leeway[i]);
                allToGreaterLeeway &= allSet(distToV > leeway[i]);
                allFromGreaterLeeway &= allSet(distFromV > leeway[i]);

                // We can prune if for all stop pairs one of the following holds:
                // 1. Both distance to v and from v are unknown (== INFTY).
                // 2. Both distance to v and from v are known and the sum breaks the leeway.
                canPrune &= allSet(((distToV == InftyLabel) & (distFromV == InftyLabel)) |
                                   ((distToV < InftyLabel) & (distFromV < InftyLabel) & (sum > leeway[i])));
            }

            if (canPrune) {
                return;
            }

            // TODO: Try constructing ellipse entries here, so we don't have to look at distances again later.
            if (!allSumGreaterLeeway && reachedInToSearch.isSet(v) && reachedInFromSearch.isSet(v)) {
                verticesInAnyEllipse.push_back(v);
            }

            // If distance to v is smaller than leeway in any ellipse, relax in downward graph.
            if (!allToGreaterLeeway && reachedInToSearch.isSet(v)) {
                FORALL_INCIDENT_EDGES(chDownGraph, v, e) {
                    ++numEdgesRelaxed;
                    const auto head = chDownGraph.edgeHead(e);

                    const bool notReachedHeadBefore = !reachedInToSearch.isSet(head);
                    for (int i = 0; i < numLabelsPerVertex; ++i) {
                        const auto &distToV = distTo[v * numLabelsPerVertex + i];
                        auto newDist = distToV + chDownGraph.template get<WeightT>(e);
                        auto &distToHead = distTo[head * numLabelsPerVertex + i];

                        // If we have not reached head before, initialize distance.
                        if (notReachedHeadBefore) {
                            distToHead = InftyLabel;
                        }
                        distToHead.min(newDist);
                        KASSERT(allSet(distToHead >= 0));
                    }

                    reachedInToSearch.set(head);
                }
            }

            // If distance from v is smaller than leeway in any ellipse, relax in reverse upward graph.
            if (!allFromGreaterLeeway && reachedInFromSearch.isSet(v)) {
                FORALL_INCIDENT_EDGES(chRevUpGraph, v, e) {
                    ++numEdgesRelaxed;
                    const auto head = chRevUpGraph.edgeHead(e);

                    const bool notReachedHeadBefore = !reachedInFromSearch.isSet(head);
                    for (int i = 0; i < numLabelsPerVertex; ++i) {
                        const auto &distFromV = distFrom[v * numLabelsPerVertex + i];
                        auto newDist = distFromV + chRevUpGraph.template get<WeightT>(e);
                        auto &distFromHead = distFrom[head * numLabelsPerVertex + i];

                        // If we have not reached head before, initialize distance.
                        if (notReachedHeadBefore) {
                            distFromHead = InftyLabel;
                        }
                        distFromHead.min(newDist);
                        KASSERT(allSet(distFromHead >= 0));
                    }

                    reachedInFromSearch.set(head);
                }
            }
        }


        const CH &ch;
        const size_t numVertices;
        CH::SearchGraph chDownGraph; // Actual downward edges in CH, not the same as ch.downwardGraph() which contains reverse downward edges.
        CH::SearchGraph chRevUpGraph; // Reverse upward edges in CH.
        EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.

        size_t numLabelsPerVertex; // Number of labels of size K needed to represent distances for each ellipse at a vertex.
        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        FastResetFlagArray <> reachedInToSearch; // Flags that mark whether vertex has been reached in to search.
        FastResetFlagArray <> reachedInFromSearch; // Flags that mark whether vertex has been reached in from search.

        LoggerT &logger;
    };

} // karri
