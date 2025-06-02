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
                                    const std::vector<int> &nextIndependentSubtree,
                                    const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                    const RouteState &routeState,
                                    const std::vector<int> &elimTreeParent,
                                    const std::vector<int> &lowestNeighbor,
                                    const std::vector<int> &lowestNeighborInSubtree)
                : ch(ch),
                  numVertices(downGraph.numVertices()),
                  downGraph(downGraph),
                  upGraph(upGraph),
                  traversalPermutation(topDownRankPermutation),
                  permutedToInputGraphId(inverseTopDownRankPermutation),
                  nextIndependentSubtree(nextIndependentSubtree),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  elimTreeParent(elimTreeParent),
                  lowestNeighbor(lowestNeighbor),
                  lowestNeighborInSubtree(lowestNeighborInSubtree),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  isInitialized(numVertices),
                  highestRelInElimTreeBranch(numVertices, -1) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);

            verticesInAnyEllipse.reserve(numVertices);
            settled.reserve(numVertices);

            beginAfterPrune.push(numVertices); // Sentinel
            endOfRestart.push(numVertices); // Sentinel
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
                initializeDistancesWithSourceBuckets(stopIds[i], i, highestInitializedVertex);
                leeways[i] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesWithTargetBuckets(stopIds[i], i, highestInitializedVertex);
            }

            initTime += timer.elapsed<std::chrono::nanoseconds>();

            static constexpr bool TRACK_NUM_EDGES_RELAXED = false;
            timer.restart();
            int numEdgesRelaxed = 0;

            KASSERT(!initialized.empty());
            std::sort(initialized.begin(), initialized.end());
            initialized.push_back(numVertices); // Sentinel

            // Traverse elimination tree in inverted post-traversal order, starting at first initialized vertex.
            // Whenever we reach a vertex v at which we can prune, continue with root of next independent subtree w or
            // restart at the next initialized vertex r if r < w. In the latter case, memorize w as well as the next
            // root u of the next independent subtree of r. When we reach u after restarting at r, we can proceed to w.
            while (beginAfterPrune.size() > 1) {
                KASSERT(beginAfterPrune.top() == numVertices);
                beginAfterPrune.pop();
            }
            while (endOfRestart.size() > 1) {
                KASSERT(endOfRestart.top() == numVertices);
                endOfRestart.pop();
            }
            int v = initialized[0];
            auto nextInitialized = initialized.begin() + 1;
            while (v < numVertices) {

                // If we have reached the next initialized vertex, step over it
                if (v == *nextInitialized)
                    ++nextInitialized;

                // If we reach the end of the last restart (the root of the next independent subtree of the restart),
                // then finish the restart by proceeding to the next vertex after the latest prune or restart at the
                // next initialized vertex if it is smaller.
                if (endOfRestart.top() == v) {
                    endOfRestart.pop();
                    if (*nextInitialized < beginAfterPrune.top()) {
                        v = *nextInitialized;
                        ++nextInitialized;
                        endOfRestart.push(nextIndependentSubtree[v]);
                    } else {
                        v = beginAfterPrune.top();
                        beginAfterPrune.pop();
                    }
                    continue;
                }

                // Settle v
                settleVertexInTopodownSearch<TRACK_NUM_EDGES_RELAXED>(v, leeways, numEdgesRelaxed);
                ++numVerticesSettled;
                settled.push_back(v);

                // If no vertex in the sub elimination tree rooted at v has a neighbor lower than the highest relevant
                // rank on the elimination tree branch, then we can prune at v.
                const bool canPrune = lowestNeighborInSubtree[v] > highestRelInElimTreeBranch[v];
                if (canPrune) {
                    // If we can prune, proceed to the root of the next independent subtree from v or restart at the
                    // next initialized vertex if it is smaller.
                    v = nextIndependentSubtree[v];
                    if (*nextInitialized < v) {
                        beginAfterPrune.push(v);
                        v = *nextInitialized;
                        ++nextInitialized;
                        endOfRestart.push(nextIndependentSubtree[v]);
                    }
                    continue;
                }

                ++v;
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
            KASSERT(distTo.size() == numVertices);
            KASSERT(distFrom.size() == numVertices);

            initialized.clear();
            isInitialized.reset();

            static const DistanceLabel InftyLabel = DistanceLabel(INFTY);
            for (const auto &v: settled) {
                distTo[v] = InftyLabel;
                distFrom[v] = InftyLabel;
            }
            settled.clear();
            KASSERT(allLabelsInfty(distTo));
            KASSERT(allLabelsInfty(distFrom));

            verticesInAnyEllipse.clear();
        }

        static bool allLabelsInfty(const auto &vals) {
            static const DistanceLabel InftyLabel = DistanceLabel(INFTY);
            for (int i = 0; i < vals.size(); ++i) {
                if (!allSet(vals[i] == InftyLabel)) return false;
            }
            return true;
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
                const auto r = traversalPermutation[e.rank];

                if (!isInitialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    KASSERT(allSet(distTo[r] == INFTY));
                    KASSERT(allSet(distFrom[r] == INFTY));
                    isInitialized.set(r);
                    initialized.push_back(r);
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
                const auto r = traversalPermutation[e.rank];

                if (!isInitialized.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    KASSERT(allSet(distTo[r] == INFTY));
                    KASSERT(allSet(distFrom[r] == INFTY));
                    isInitialized.set(r);
                    initialized.push_back(r);
                }

                KASSERT(distFrom[r][ellipseIdx] == INFTY);
                distFrom[r][ellipseIdx] = e.distance;
                highestRelInElimTreeBranch[elimTreeParent[r]] = -1;
                highestInitialized = std::max(highestInitialized, r);
            }
        }

        template<bool TRACK_NUM_EDGES = false>
        void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway, int &numEdgesRelaxed) {

            auto &distToV = distTo[v];
            auto &distFromV = distFrom[v];

            // Propagate information on highest relevant rank along elimination tree edge.
            auto &highestRel = highestRelInElimTreeBranch[v];
            highestRel = highestRelInElimTreeBranch[elimTreeParent[v]];
            const bool needToSettle = lowestNeighbor[v] <= highestRel;

            const auto init = isInitialized.isSet(v);
            if (!init && !needToSettle)
                return;

            // If there are no relevant distances at upward neighbors (!needToSettle) and there are no distances
            // from bucket entries (!initV), then skip relaxations and check for ellipse membership.
            if (needToSettle) {
                const int lastDownEdge = downGraph.lastEdge(v);
                for (int e = downGraph.firstEdge(v); e < lastDownEdge; ++e) {
                    const auto head = downGraph.edgeHead(e);

                    if constexpr (TRACK_NUM_EDGES) ++numEdgesRelaxed;
                    const auto &distToHead = distTo[head];
                    const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                    distToV.min(distViaHead);
                }

                const int lastUpEdge = upGraph.lastEdge(v);
                for (int e = upGraph.firstEdge(v); e < lastUpEdge; ++e) {
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
            }
        }

        std::vector<std::vector<VertexInEllipse>>
        constructEllipses(const int numEllipses, const DistanceLabel &leeways) {
            std::vector<std::vector<VertexInEllipse>> ellipses(numEllipses);
            for (auto &ellipse: ellipses)
                ellipse.reserve(verticesInAnyEllipse.size());
            for (const auto &r: verticesInAnyEllipse) {
                const int vertex = permutedToInputGraphId[r]; // vertex ID in input graph

                const auto &distFromVertex = distFrom[r];
                const auto &distToVertex = distTo[r];
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
        const Permutation &traversalPermutation; // Maps vertex rank in CH to rank in traversal order.
        const Permutation &permutedToInputGraphId; // Maps rank in traversal order to vertex ID in input graph
        const std::vector<int> &nextIndependentSubtree;
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;
        const std::vector<int> &elimTreeParent; // Elimination tree with vertices ordered by decreasing rank.

        const std::vector<int> &lowestNeighbor; // lowest (i.e. most important) neighbor in either graph
        const std::vector<int> &lowestNeighborInSubtree; // lowest (i.e. most important) neighbor in subtree of elimination tree rooted at vertex

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;

        // Flags that indicate whether distances have been initialized with bucket entries
        FastResetFlagArray<> isInitialized;
        std::vector<int> initialized;
        std::vector<int> settled;

        std::stack<int, std::vector<int>> beginAfterPrune;
        std::stack<int, std::vector<int>> endOfRestart;

        // highestRelInElimTreeBranch[v] is the highest rank (permuted, so least important vertex) on the
        // elimination-tree branch of v that is in an ellipse. When settling vertex v, we only
        // have to relax edges with head ranks up to highestRelInElimTreeBranch[v].
        std::vector<int> highestRelInElimTreeBranch;
    };

} // karri
