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
#include "CHEllipseReconstructorQuery.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename InputGraphT, typename CHEnvT, typename EllipticBucketsEnvironmentT, typename WeightT = TraversalCostAttribute,
            typename LabelSetT = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>,
            typename LoggerT = NullLogger>
    class CHEllipseReconstructor {

        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        static constexpr int K = LabelSetT::K;

        using Query = CHEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSetT, WeightT>;

        struct QueryStats {
            int numVerticesSettled = 0;
            int numEdgesRelaxed = 0;
            int numVerticesInAnyEllipse = 0;
            int64_t initTime = 0;
            int64_t topoSearchTime = 0;
            int64_t postprocessTime = 0;
        };
    public:

        CHEllipseReconstructor(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                               const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                               const RouteState &routeState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  level(inputGraph.numVertices(), 0),
                  elimTreeParent(chEnv.getCCH().getEliminationTree()),
                  topDownRankPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  lowestNeighbor(inputGraph.numVertices()),
                  lowestNeighborInSubtree(inputGraph.numVertices()),
                  query([&]() {
                      return Query(ch, downGraph, upGraph, topDownRankPermutation, inverseTopDownRankPermutation,
                                   ellipticBucketsEnv, routeState, elimTreeParent, firstElimTreeChild, elimTreeChildren,
                                   lowestNeighbor, lowestNeighborInSubtree, maxLevel, level, levelStart);
                  }),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                        "num_ellipses,"
                                                        "init_time,"
                                                        "topo_search_time,"
                                                        "postprocess_time,"
                                                        "total_time,"
                                                        "topo_search_num_vertices_settled,"
                                                        "topo_search_num_edges_relaxed,"
                                                        "topo_search_num_vertices_in_any_ellipse\n")) {
            KASSERT(downGraph.numVertices() == upGraph.numVertices());
            const int numVertices = downGraph.numVertices();

            // Compute zones of bounded diameter based on elimination tree
            static constexpr int MAX_DIAM = 60;
            zone = deductCellsFromEliminationTree<MAX_DIAM>(elimTreeParent);

            // Compute levels of vertices in elimination tree (level[v] = tree height - height of subtree rooted at v)
            KASSERT(elimTreeParent[numVertices - 1] == -1); // assert that root has rank numVertices - 1
            for (int v = 0; v < numVertices - 1; ++v) {
                const auto levelV = level[v];
                auto &levelParent = level[elimTreeParent[v]];
                levelParent = std::max(levelParent, levelV + 1);
            }
            maxLevel = level[numVertices - 1];
            for (auto &l: level)
                l = maxLevel - l; // Reverse levels to have root at level 0.


            // Compute top-down rank permutation based on levels

            // Build out elimination tree
            convertInTreeToOutTree(elimTreeParent, firstElimTreeChild, elimTreeChildren);
            for (int v = 0; v < numVertices; ++v) {
                for (int i = firstElimTreeChild[v]; i < firstElimTreeChild[v + 1] - 1; ++i) {
                    KASSERT(elimTreeChildren[i] < elimTreeChildren[i + 1]);
                }
            }

            // Compute index where each level starts by finding level sizes and computing prefix sum
            levelStart.resize(maxLevel + 2);
            for (int v = 0; v < numVertices; ++v)
                ++levelStart[level[v]];
            int curStart = 0;
            for (int l = 0; l <= maxLevel + 1; ++l) {
                const int nextStart = curStart + levelStart[l];
                levelStart[l] = curStart;
                curStart = nextStart;
            }
            KASSERT(curStart == numVertices && levelStart[maxLevel + 1] == numVertices);

            // Traverse elimination tree level by level, constructing lower levels in order vertices are seen.
            auto nextInLevel = levelStart;
            std::vector<int> inversePermData(numVertices, INVALID_INDEX);
            inversePermData[0] = numVertices - 1; // root
            ++nextInLevel[0];
            for (int l = 0; l <= maxLevel; ++l) {
                // Level l is full. Loop over vertices in this level and add children to the current end of their
                // respective levels.
                KASSERT(nextInLevel[l] == levelStart[l + 1]);
                for (int i = levelStart[l]; i < levelStart[l + 1]; ++i) {
                    const auto v = inversePermData[i];
                    KASSERT(v >= 0 && v < numVertices);
                    KASSERT(level[v] == l);
                    // Add children to their respective levels
                    for (int j = firstElimTreeChild[v]; j < firstElimTreeChild[v + 1]; ++j) {
                        const auto child = elimTreeChildren[j];
                        KASSERT(child < v);
                        inversePermData[nextInLevel[level[child]]] = child;
                        ++nextInLevel[level[child]];
                    }
                }
            }

            inverseTopDownRankPermutation = Permutation(inversePermData.begin(), inversePermData.end());
            topDownRankPermutation = inverseTopDownRankPermutation.getInversePermutation();

            topDownRankPermutation.applyTo(level);
            KASSERT(std::is_sorted(level.begin(), level.end()));

            // Permute tails of in elimination tree
            topDownRankPermutation.applyTo(elimTreeParent);

            // Permute heads of in elimination tree
            for (auto &parent: elimTreeParent) {
                if (parent == -1) // Root of tree does not have a parent, no need to permute
                    continue;
                parent = topDownRankPermutation[parent];
            }

            // Our permuted in elimination tree marks root (rank 0) by being its own parent.
            elimTreeParent[0] = 0;

            // Permute out elimination tree
            convertInTreeToOutTree(elimTreeParent, firstElimTreeChild, elimTreeChildren);

            // Permute search graphs
            downGraph.permuteVertices(topDownRankPermutation);
            downGraph.sortOutgoingEdges();
            upGraph.permuteVertices(topDownRankPermutation);
            upGraph.sortOutgoingEdges();

            // Permute zone
            topDownRankPermutation.applyTo(zone);

            // Compute neighbor with lowest permuted rank (i.e. most important neighbor) in either graph for every
            // vertex. Additionally compute the lowest such neighbor in the subtree of the elimination tree rooted at
            // each vertex (subtree excluding the vertex itself).
            for (int v = 0; v < numVertices; ++v) {
                const int lowestNeighborInDown =
                        downGraph.lastEdge(v) == downGraph.firstEdge(v) ? numVertices : downGraph.edgeHead(
                                downGraph.firstEdge(v));
                const int lowestNeighborInUp =
                        upGraph.lastEdge(v) == upGraph.firstEdge(v) ? numVertices : upGraph.edgeHead(
                                upGraph.firstEdge(v));
                lowestNeighbor[v] = std::min(lowestNeighborInDown, lowestNeighborInUp);
            }

            std::fill(lowestNeighborInSubtree.begin(), lowestNeighborInSubtree.end(), INFTY);
            for (int v = numVertices - 1; v >= 0; --v) {
                const auto p = elimTreeParent[v];
                lowestNeighborInSubtree[p] = std::min(lowestNeighborInSubtree[p], lowestNeighbor[v]);
                lowestNeighborInSubtree[p] = std::min(lowestNeighborInSubtree[p], lowestNeighborInSubtree[v]);
            }
        }

        std::vector<std::vector<VertexInEllipse>>
        getVerticesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds, int &totalNumVerticesSettled,
                                              int &totalNumEdgesRelaxed) {

            totalNumVerticesSettled = 0;
            totalNumEdgesRelaxed = 0;
            if (stopIds.empty())
                return {};

            int totalNumVerticesInAnyEllipse = 0;
            Timer timer;

            const size_t numEllipses = stopIds.size();

            // Order stop pairs by zone to exploit locality
            std::vector<std::pair<int, int>> stopPairZones(stopIds.size());
            for (int i = 0; i < numEllipses; ++i) {
                const int stopId = stopIds[i];
                const int vehId = routeState.vehicleIdOf(stopId);
                const int stopIdx = routeState.stopPositionOf(stopId);
                const auto locs = routeState.stopLocationsFor(vehId);
                const int firstStopHead = topDownRankPermutation[ch.rank(inputGraph.edgeHead(locs[stopIdx]))];
                const int secondStopTail = topDownRankPermutation[ch.rank(inputGraph.edgeTail(locs[stopIdx + 1]))];
                stopPairZones[i] = {zone[firstStopHead], zone[secondStopTail]};
            }
            std::vector<int> order(numEllipses);
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](const auto &a, const auto &b) {
                return stopPairZones[a] < stopPairZones[b];
            });

            const size_t numBatches = numEllipses / K + (numEllipses % K != 0);

            std::vector<std::vector<VertexInEllipse>> ellipses;
            ellipses.resize(numEllipses);
            tbb::parallel_for(0ul, numBatches, [&](size_t i) {
                auto &localQuery = query.local();
                auto &localStats = queryStats.local();
                std::vector<int> batchStopIds;
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    batchStopIds.push_back(stopIds[order[i * K + j]]);
                }
                auto batchResult = localQuery.run(batchStopIds, localStats.numVerticesSettled,
                                                  localStats.numEdgesRelaxed, localStats.numVerticesInAnyEllipse,
                                                  localStats.initTime,
                                                  localStats.topoSearchTime, localStats.postprocessTime);
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    ellipses[order[i * K + j]].swap(batchResult[j]);
                }
            });

            int64_t totalInitTime = 0;
            int64_t totalTopoSearchTime = 0;
            int64_t totalPostprocessTime = 0;
            for (auto &localStats: queryStats) {
                totalNumVerticesSettled += localStats.numVerticesSettled;
                totalNumEdgesRelaxed += localStats.numEdgesRelaxed;
                totalNumVerticesInAnyEllipse += localStats.numVerticesInAnyEllipse;
                totalInitTime += localStats.initTime;
                totalTopoSearchTime += localStats.topoSearchTime;
                totalPostprocessTime += localStats.postprocessTime;
                localStats.numVerticesSettled = 0;
                localStats.numEdgesRelaxed = 0;
                localStats.numVerticesInAnyEllipse = 0;
                localStats.initTime = 0;
                localStats.topoSearchTime = 0;
                localStats.postprocessTime = 0;
            }

            const auto totalTime = timer.elapsed<std::chrono::nanoseconds>();

            logger << numEllipses << "," << totalInitTime << "," << totalTopoSearchTime << ","
                   << totalPostprocessTime << "," << totalTime << "," << totalNumVerticesSettled << ","
                   << totalNumEdgesRelaxed
                   << "," << totalNumVerticesInAnyEllipse << "\n";

            return ellipses;
        }


    private:

        // An active vertex during a DFS, i.e., a vertex that has been reached but not finished.
        struct ActiveVertex {
            // Constructs an active vertex.
            ActiveVertex(const int id, const int nextUnexploredEdge)
                    : id(id), nextUnexploredEdge(nextUnexploredEdge) {}

            int id;                 // The ID of the active vertex.
            int nextUnexploredEdge; // The next unexplored incident edge.
        };

        // Taken from Valentin Buchhold's routing-framework. Algorithm described in
        // "Real-time traffic assignment using engineered customizable contraction hierarchies", JEA, Vol. 24, 2019.
        // Given an in-elimination tree (vertex IDs ordered by increasing rank), this function decomposes the graph
        // into cells of bounded diameter and returns the cell ID for each rank.
        template<int maxDiam>
        static std::vector<int> deductCellsFromEliminationTree(const std::vector<int> &tree) {
            const auto numVertices = tree.size();
            // Build the elimination out-tree from the elimination in-tree.
            std::vector<int> firstChild;
            std::vector<int> children;
            convertInTreeToOutTree(tree, firstChild, children);

            // Decompose the elimination tree into as few cells with bounded diameter as possible.
            BitVector isRoot(numVertices);
            std::vector<int> height(numVertices); // height[v] is the height of the subtree rooted at v.
            for (auto v = 0; v < numVertices; ++v) {
                const auto first = firstChild[v];
                const auto last = firstChild[v + 1];
                std::sort(children.begin() + first, children.begin() + last, [&](const auto u, const auto v) {
                    assert(u >= 0);
                    assert(u < height.size());
                    assert(v >= 0);
                    assert(v < height.size());
                    return height[u] < height[v];
                });
                for (auto i = first; i < last; ++i)
                    if (height[v] + 1 + height[children[i]] <= maxDiam)
                        height[v] = 1 + height[children[i]];
                    else
                        isRoot[children[i]] = true;
            }

            // Number the cells in the order in which they are discovered during a DFS from the root.
            int freeCellId = 1; // The next free cell ID.
            std::vector<int> cellIds(numVertices);
            std::stack<ActiveVertex, std::vector<ActiveVertex>> activeVertices;
            activeVertices.emplace(numVertices - 1, firstChild[numVertices - 1]);
            while (!activeVertices.empty()) {
                auto &v = activeVertices.top();
                const auto head = children[v.nextUnexploredEdge];
                ++v.nextUnexploredEdge;
                cellIds[head] = isRoot[head] ? freeCellId++ : cellIds[v.id];
                if (v.nextUnexploredEdge == firstChild[v.id + 1])
                    activeVertices.pop();
                if (firstChild[head] != firstChild[head + 1])
                    activeVertices.emplace(head, firstChild[head]);
            }

            return cellIds;
        }

        static void convertInTreeToOutTree(const std::vector<int> &parent,
                                           std::vector<int> &firstChild,
                                           std::vector<int> &children) {
            const auto numVertices = parent.size();
            // Build the elimination out-tree from the elimination in-tree.
            firstChild = std::vector<int>(numVertices + 1);
            children = std::vector<int>(numVertices - 1);
            for (auto v = 0; v < numVertices; ++v) {
                const auto p = parent[v];
                if (p == -1 || p == v) // Root of tree may be marked by -1 or edge to self
                    continue;
                ++firstChild[parent[v]];
            }
            auto firstEdge = 0; // The index of the first edge out of the current/next vertex.
            for (auto v = 0; v <= numVertices; ++v) {
                std::swap(firstEdge, firstChild[v]);
                firstEdge += firstChild[v];
            }
            for (auto v = 0; v < numVertices; ++v) {
                const auto p = parent[v];
                if (p == -1 || p == v) // Root of tree may be marked by -1 or edge to self
                    continue;
                children[firstChild[p]++] = v;
            }
            for (auto v = numVertices - 1; v > 0; --v)
                firstChild[v] = firstChild[v - 1];
            firstChild[0] = 0;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        int maxLevel;
        std::vector<int> level; // Level of vertex in elimination tree from root (lower level means more important)
        std::vector<int> levelStart;
        std::vector<int> elimTreeParent; // Elimination tree of the CH with vertices ordered by decreasing rank (parent pointers).
        std::vector<int> firstElimTreeChild; // First in range of children in out elimination tree
        std::vector<int> elimTreeChildren; // Array of children in out elimination tree
        std::vector<int> zone; // Zone of each vertex, i.e., the cell ID of the cell that contains the vertex.

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        Permutation inverseTopDownRankPermutation;

        std::vector<int> lowestNeighbor; // lowest (i.e. most important) neighbor in either graph
        std::vector<int> lowestNeighborInSubtree; // lowest (i.e. most important) neighbor in subtree of elimination tree rooted at vertex excluding itself

        tbb::enumerable_thread_specific<QueryStats> queryStats;
        tbb::enumerable_thread_specific<Query> query;

        LoggerT &logger;
    };

} // karri
