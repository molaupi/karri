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
                  elimTreeParent(chEnv.getCCH().getEliminationTree()),
                  traversalPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  lowestNeighbor(inputGraph.numVertices()),
                  lowestNeighborInSubtree(inputGraph.numVertices()),
                  query([&]() {
                      return Query(ch, downGraph, upGraph, traversalPermutation, permutedToInputGraphId,
                                   nextIndependentSubtree, ellipticBucketsEnv, routeState, elimTreeParent,
                                   lowestNeighbor, lowestNeighborInSubtree);
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

            // Compute out elimination tree from in elimination tree
            convertInTreeToOutTree(elimTreeParent, firstElimTreeChild, elimTreeChildren);

            // Compute zones of bounded diameter based on elimination tree
            static constexpr int MAX_DIAM = 60;
            zone = deductCellsFromOutEliminationTree<MAX_DIAM>(firstElimTreeChild, elimTreeChildren);

            // Compute inverted post-traversal order for elimination tree
            auto [iPto, lastHorPred] = computeDfsPostTraversalOrderAndLastHorizontalPredecessor(firstElimTreeChild,
                                                                                                elimTreeChildren);
            for (auto &v: iPto)
                v = numVertices - 1 - v;
            for (auto &p: lastHorPred)
                p = numVertices - 1 - p;

            traversalPermutation = Permutation(iPto.begin(), iPto.end());
            permutedToInputGraphId = Permutation(numVertices);
            for (int v = 0; v < numVertices; ++v) {
                permutedToInputGraphId[traversalPermutation[ch.rank(v)]] = v;
            }
            KASSERT(permutedToInputGraphId.validate());

            // Permute last horizontal predecessor array due to new vertex order
            traversalPermutation.applyTo(lastHorPred);
            KASSERT(lastHorPred[0] == numVertices); // sanity check: root must have invalid last horizontal predecessor

            // w = nextIndependentSubtree[v] is the next vertex in the inverted PTO that is not contained in the subtree
            // of the elimination tree rooted at v. The subtree rooted at w is entirely independent of the one rooted at
            // v s.t. when a traversal of the tree in inverted PTO can be pruned at v, it is correct to continue at w.
            nextIndependentSubtree = lastHorPred;
            for (int v = 0; v < numVertices; ++v) {
                KASSERT(v < nextIndependentSubtree[v]);
            }

            // Permute tails of in elimination tree
            traversalPermutation.applyTo(elimTreeParent);

            // Permute heads of in elimination tree
            for (auto &parent: elimTreeParent) {
                if (parent == -1) // Root of tree does not have a parent, no need to permute
                    continue;
                parent = traversalPermutation[parent];
            }

            // Our permuted in elimination tree marks root (rank 0) by being its own parent.
            elimTreeParent[0] = 0;

            // Re-compute out elimination tree for new order
            convertInTreeToOutTree(elimTreeParent, firstElimTreeChild, elimTreeChildren);

            // Permute search graphs
            downGraph.permuteVertices(traversalPermutation);
            downGraph.sortOutgoingEdges();
            upGraph.permuteVertices(traversalPermutation);
            upGraph.sortOutgoingEdges();

            // Permute zone
            traversalPermutation.applyTo(zone);

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
                const int firstStopHead = traversalPermutation[ch.rank(inputGraph.edgeHead(locs[stopIdx]))];
                const int secondStopTail = traversalPermutation[ch.rank(inputGraph.edgeTail(locs[stopIdx + 1]))];
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

        // Taken from Valentin Buchhold's routing-framework. Algorithm described in
        // "Real-time traffic assignment using engineered customizable contraction hierarchies", JEA, Vol. 24, 2019.
        // Given an out-elimination tree (vertex IDs ordered by increasing rank), this function decomposes the graph
        // into cells of bounded diameter and returns the cell ID for each rank.
        template<int maxDiam>
        static std::vector<int> deductCellsFromOutEliminationTree(const std::vector<int> &firstChild,
                                                                  const std::vector<int> &childrenIn) {

            auto children = childrenIn; // Copy to allow sorting by height in algorithm
            const int numVertices = static_cast<int>(firstChild.size()) - 1;

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

        // Compute a DFS post traversal order for the given tree in out format.
        // Expects root of tree to be at index firstChild.size() - 1.
        static std::pair<std::vector<int>, std::vector<int>> computeDfsPostTraversalOrderAndLastHorizontalPredecessor(
                const std::vector<int> &firstChild,
                const std::vector<int> &children) {
            const int numVertices = static_cast<int>(firstChild.size()) - 1;
            std::vector<int> pto(numVertices, INVALID_VERTEX);
            int nextRank = 0;
            std::stack<ActiveVertex, std::vector<ActiveVertex>> activeVertices;
            // Contains PTO rank of the last horizontal predecessor, i.e. PTO of a previous sibling if one exists or
            // PTO of the last horizontal predecessor of the parent otherwise. If there is no horizontal predecessor
            // at all (first branch from the root), the placeholder -1 is used.
            std::vector<int> lastHorizontalPred(numVertices, -2);
            activeVertices.emplace(numVertices - 1, firstChild[numVertices - 1]); // add root
            lastHorizontalPred[numVertices - 1] = -1;
            while (!activeVertices.empty()) {
                auto &v = activeVertices.top();
                if (v.nextUnexploredEdge == firstChild[v.id + 1]) {
                    // Finished with this v, add to post traversal order.
                    pto[v.id] = nextRank++;
                    activeVertices.pop();
                    continue;
                }
                // Advance to next child
                const auto child = children[v.nextUnexploredEdge];
                const auto lastHor = v.nextUnexploredEdge == firstChild[v.id] ? lastHorizontalPred[v.id] : pto[children[
                        v.nextUnexploredEdge - 1]];
                lastHorizontalPred[child] = lastHor;
                ++v.nextUnexploredEdge; // When backtracking from child later, look at next sibling
                activeVertices.emplace(child, firstChild[child]);
            }
            KASSERT(nextRank == numVertices);
            KASSERT(pto[numVertices - 1] == numVertices - 1); // root must be last in post traversal order
            KASSERT(std::all_of(pto.begin(), pto.end(), [&](const auto r) { return r >= 0 && r < numVertices; }));
            KASSERT(std::all_of(lastHorizontalPred.begin(), lastHorizontalPred.end(),
                                [&](const auto r) { return r >= -1 && r < numVertices; }));

            return {pto, lastHorizontalPred};
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        std::vector<int> elimTreeParent; // Elimination tree of the CH with vertices ordered by decreasing rank (parent pointers).
        std::vector<int> firstElimTreeChild; // First in range of children in out elimination tree
        std::vector<int> elimTreeChildren; // Array of children in out elimination tree
        std::vector<int> zone; // Zone of each vertex, i.e., the cell ID of the cell that contains the vertex.

        Permutation traversalPermutation; // Maps vertex rank to traversal order of sweep.
        Permutation permutedToInputGraphId; // Maps vertex ID in permutation to vertex ID in input graph

        // w = nextIndependentSubtree[v] is the next vertex in the inverted PTO that is not contained in the subtree
        // of the elimination tree rooted at v. The subtree rooted at w is entirely independent of the one rooted at
        // v s.t. when a traversal of the tree in inverted PTO can be pruned at v, it is correct to continue at w.
        // If the traversal is done after processing the subtree rooted at v, then nextIndependentSubtree[v] == numVertices.
        std::vector<int> nextIndependentSubtree;

        std::vector<int> lowestNeighbor; // lowest (i.e. most important) neighbor in either graph
        std::vector<int> lowestNeighborInSubtree; // lowest (i.e. most important) neighbor in subtree of elimination tree rooted at vertex excluding itself

        tbb::enumerable_thread_specific<QueryStats> queryStats;
        tbb::enumerable_thread_specific<Query> query;

        LoggerT &logger;
    };

} // karri
