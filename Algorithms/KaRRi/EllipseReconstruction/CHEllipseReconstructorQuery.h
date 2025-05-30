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
                                    const Permutation &inverseTopDownRankPermutation,
                                    const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                    const RouteState &routeState,
                                    const std::vector<int> &elimTreeParent,
                                    const std::vector<int> &firstElimTreeChild,
                                    const std::vector<int> &elimTreeChildren,
                                    const std::vector<int> &lowestNeighbor,
                                    const std::vector<int> &lowestNeighborInSubtree,
                                    const int maxLevel,
                                    const std::vector<int> &level,
                                    const std::vector<int> &levelStart)
                : ch(ch),
                  numVertices(downGraph.numVertices()),
                  downGraph(downGraph),
                  upGraph(upGraph),
                  topDownRankPermutation(topDownRankPermutation),
                  inverseTopDownRankPermutation(inverseTopDownRankPermutation),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  elimTreeParent(elimTreeParent),
                  firstElimTreeChild(firstElimTreeChild),
                  elimTreeChildren(elimTreeChildren),
                  lowestNeighbor(lowestNeighbor),
                  lowestNeighborInSubtree(lowestNeighborInSubtree),
                  maxLevel(maxLevel),
                  level(level),
                  levelStart(levelStart),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  endToSettleInLevel(maxLevel + 1),
                  toSettleInLevel(numVertices),
                  initInLevel(maxLevel + 1),
                  distTo(numVertices),
                  distFrom(numVertices),
                  verticesInAnyEllipse(),
                  initialized(numVertices),
                  highestRelInElimTreeBranch(numVertices, -1) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);
            verticesInAnyEllipse.reserve(numVertices);
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

            for (int l = 0; l <= maxLevel; ++l) {
                endToSettleInLevel[l] = levelStart[l];
                initInLevel[l].clear();
            }

            DistanceLabel leeways = 0;
            const auto numEllipses = stopIds.size();
            int highestInitializedVertex = -1;
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesWithSourceBuckets(stopIds[i], i, highestInitializedVertex);
                leeways[i] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesWithTargetBuckets(stopIds[i], i, highestInitializedVertex);
            }

            initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            static constexpr bool TRACK_NUM_EDGES_RELAXED = true;
            timer.restart();
            int numEdgesRelaxed = 0;
            for (int l = 0; l <= maxLevel; ++l) {

                const auto startOfLevel = toSettleInLevel.begin() + levelStart[l];
                auto endOfLevel = toSettleInLevel.begin() + endToSettleInLevel[l];
                KASSERT(std::is_sorted(startOfLevel, endOfLevel));

                // Make sure vertices in this level that were initialized with bucket entries are settled. Insert them
                // at the right spot in range of vertices that need to be settled if they are not included yet.
                // In almost all cases, either all initialized vertices will already be included or the range will be
                // empty before this, so the overhead is small.
                auto& initInThisLevel = initInLevel[l];
                std::sort(initInThisLevel.begin(), initInThisLevel.end());
                auto it = startOfLevel;
                for (const auto& v : initInThisLevel) {
                    while (it != endOfLevel && *it < v) {
                        ++it;
                    }
                    if (it != endOfLevel && *it == v)
                        continue; // Already included.
                    std::copy_backward(it, endOfLevel, endOfLevel + 1);
                    *it = v;
                    ++endToSettleInLevel[l];
                    ++endOfLevel;
                    ++it;
                }
                KASSERT(endToSettleInLevel[l] <= levelStart[l + 1]);
                KASSERT(std::is_sorted(startOfLevel, endOfLevel));

                // Settle all required vertices at the current level.
                for (auto vIt = startOfLevel; vIt < endOfLevel; ++vIt) {
                    const auto v = *vIt;
                    settleVertexInTopodownSearch<TRACK_NUM_EDGES_RELAXED>(v, leeways, numEdgesRelaxed);
                    ++numVerticesSettled;
                }
            }

            topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();
            totalNumEdgesRelaxed += numEdgesRelaxed;
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
            distTo.init();
            distFrom.init();

            verticesInAnyEllipse.clear();
        }

        void initializeDistancesWithSourceBuckets(const int stopId, const int ellipseIdx,
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

                if (!initialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                    distFrom[r] = INFTY;
                    initialized.set(r);
                    KASSERT(level[r] <= maxLevel);
                    initInLevel[level[r]].push_back(r);
                }

                distTo[r][ellipseIdx] = e.distance;
                highestRelInElimTreeBranch[elimTreeParent[r]] = -1;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        void initializeDistancesWithTargetBuckets(const int stopId, const int ellipseIdx,
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

                if (!initialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                    distFrom[r] = INFTY;
                    initialized.set(r);
                    KASSERT(level[r] <= maxLevel);
                    initInLevel[level[r]].push_back(r);
                }

                KASSERT(distFrom[r][ellipseIdx] == INFTY);
                distFrom[r][ellipseIdx] = e.distance;
                highestRelInElimTreeBranch[elimTreeParent[r]] = -1;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        template<bool TRACK_NUM_EDGES = false>
        void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway, int &numEdgesRelaxed) {

            const auto init = initialized.isSet(v);
            auto &distToV = distTo.getWithoutStaleCheck(v);
            auto &distFromV = distFrom.getWithoutStaleCheck(v);
            if (!init) {
                static const DistanceLabel InftyLabel = DistanceLabel(INFTY);
                distToV = InftyLabel;
                distFromV = InftyLabel;
            }

            // Propagate information on highest relevant rank along elimination tree edge.
            auto &highestRel = highestRelInElimTreeBranch[v];
            highestRel = highestRelInElimTreeBranch[elimTreeParent[v]];
            const bool needToSettle = lowestNeighbor[v] <= highestRel;

            // If there are no relevant distances at upward neighbors (!needToSettle) and there are no distances
            // from bucket entries (!initV), then skip relaxations and check for ellipse membership.
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
            if (init || needToSettle) {
                const auto sum = distToV + distFromV;
                const auto sumGreaterLeeway = sum > leeway;
                if (!allSet(sumGreaterLeeway)) {
                    verticesInAnyEllipse.push_back(v);
                    highestRel = v;
                }
            }

            // If no vertex in the sub elimination tree rooted at v has a neighbor lower than the highest relevant
            // rank on the elimination tree branch, then we do not have to keep going on this branch.
            const bool needToPropagate = lowestNeighborInSubtree[v] <= highestRel;
            if (needToPropagate) {
                // Store all elimination tree children to be settled
                for (int i = firstElimTreeChild[v]; i < firstElimTreeChild[v + 1]; ++i) {
                    const auto child = elimTreeChildren[i];
                    KASSERT(child > v);
                    toSettleInLevel[endToSettleInLevel[level[child]]++] = child;
                }
            }
        }

        std::vector<std::vector<VertexInEllipse>>
        constructEllipses(const int numEllipses, const DistanceLabel &leeways) {
            std::vector<std::vector<VertexInEllipse>> ellipses(numEllipses);
            for (auto &ellipse: ellipses)
                ellipse.reserve(verticesInAnyEllipse.size());
            for (const auto &r: verticesInAnyEllipse) {
                const auto originalRank = inverseTopDownRankPermutation[r]; // Reverse permutation in search graphs
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
        const Permutation &inverseTopDownRankPermutation;
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;
        const std::vector<int> &elimTreeParent; // Elimination tree with vertices ordered by decreasing rank.
        const std::vector<int> &firstElimTreeChild; // First in range of children in out elimination tree
        const std::vector<int> &elimTreeChildren; // Array of children in out elimination tree

        const std::vector<int> &lowestNeighbor; // lowest (i.e. most important) neighbor in either graph
        const std::vector<int> &lowestNeighborInSubtree; // lowest (i.e. most important) neighbor in subtree of elimination tree rooted at vertex

        const int maxLevel;
        const std::vector<int> &level; // Level of vertex in elimination tree from root (lower level means more important)
        const std::vector<int> &levelStart; // Top-down rank at which level starts

        Subset enumerateBucketEntriesSearchSpace;

        // When reaching level l, all vertices in the range toSettleInLevel[levelStart[l] .. endToSettleInLevel[l])
        // must be settled where always endToSettleInLevel[l] <= levelStart[l + 1].
        std::vector<int> endToSettleInLevel;
        std::vector<int> toSettleInLevel;

        std::vector<std::vector<int>> initInLevel; // For each level, the vertices that have been initialized with bucket entries.

//        AlignedVector <DistanceLabel> distTo;
//        AlignedVector <DistanceLabel> distFrom;
        StampedDistanceLabelContainer<DistanceLabel> distTo;
        StampedDistanceLabelContainer<DistanceLabel> distFrom;
//        SimpleDistanceLabelContainer<DistanceLabel> distTo;
//        SimpleDistanceLabelContainer<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        // Flags that indicate whether distances have been initialized with bucket entries
        FastResetFlagArray<> initialized;

        // highestRelInElimTreeBranch[v] is the highest rank (permuted, so least important vertex) on the
        // elimination-tree branch of v that is in an ellipse. When settling vertex v, we only
        // have to relax edges with head ranks up to highestRelInElimTreeBranch[v].
        std::vector<int> highestRelInElimTreeBranch;
    };

} // karri
