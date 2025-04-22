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
#include <kassert/kassert.hpp>
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Dijkstra/DagShortestPaths.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "PHASTQuery.h"

// Result of the RPHAST selection phase
struct RPHASTSelection {

    RPHASTSelection() = default;

    RPHASTSelection(const RPHASTSelection &) = delete;

    RPHASTSelection &operator=(const RPHASTSelection &) = delete;

    RPHASTSelection &operator=(RPHASTSelection &&other) {
        if (this != &other) {
            subGraph = std::move(other.subGraph);
            fullToSubMapping = std::move(other.fullToSubMapping);
            subToFullMapping = std::move(other.subToFullMapping);
        }
        return *this;
    }

    RPHASTSelection(RPHASTSelection &&other)
            : subGraph(std::move(other.subGraph)),
              fullToSubMapping(std::move(other.fullToSubMapping)),
              subToFullMapping(std::move(other.subToFullMapping)) {}

    // Subgraph induced by the targets. Vertices ordered by decreasing rank.
    typename CH::SearchGraph subGraph;

    // Mapping from full graph vertex IDs to subgraph vertex IDs. Vertices that are not in subgraph have mapping
    // subGraph.numVertices() (i.e. one past last valid vertex).
    std::vector<int> fullToSubMapping;

    // Mapping from subgraph vertex IDs to full graph vertex IDs.
    std::vector<int> subToFullMapping;


};

template<typename PruningCriterionT = dij::NoCriterion,
        bool OrderByLevels = false>
class RPHASTSelectionPhase {

    static constexpr bool WithPruning = !std::is_same_v<PruningCriterionT, dij::NoCriterion>;

    using SearchGraph = typename CH::SearchGraph;
//    using SelectionSearch = DagShortestPaths<SearchGraph, typename CH::Weight, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, dij::CompoundCriterion<PruningCriterionT, RememberSearchSpace>>;


    struct SubgraphVertexWithLevel {
        int vertex = INVALID_VERTEX;
        int level = -1;
    };

    bool validateVerticesInIncreasingRankOrder(const SearchGraph &graph) {
        FORALL_VERTICES(graph, u) {
            int prevEdgeHead = -1;
            FORALL_INCIDENT_EDGES(graph, u, e) {
                const auto v = graph.edgeHead(e);
                KASSERT(u < v);
                if (u > v)
                    return false;
                KASSERT(v > prevEdgeHead);
                if (v < prevEdgeHead)
                    return false;
                prevEdgeHead = v;
            }
        }
        return true;
    }

public:

    explicit RPHASTSelectionPhase(const SearchGraph &fullGraph,
                                  const CH &ch,
                                  PruningCriterionT prune = {})
            : fullGraph(fullGraph),
              ch(ch),
              verticesInSubgraph(fullGraph.numVertices()),
              upSearchPriorityQueue(fullGraph.numVertices()),
              upSearchLevels(fullGraph.numVertices(), -1),
              upSearchDistances(0),
              pruneUpSearch(prune) {
        if constexpr (WithPruning) {
            upSearchDistances.resize(fullGraph.numVertices());
        } else {
            upBfsQueue.reserve(fullGraph.numVertices());
        }
        KASSERT(validateVerticesInIncreasingRankOrder(fullGraph));
    }

    RPHASTSelection run(const std::vector<int> &targets, const std::vector<int> &offsets = {}) {

        if constexpr (WithPruning || OrderByLevels) {
            findVerticesInSubgraphOrderedByLevels(targets, offsets);
        } else {
            findVerticesInSubgraphOrderedByRanks(targets);
        }

        RPHASTSelection result;
        result.subGraph = constructOrderedSubgraph(verticesInSubgraph, result.fullToSubMapping);
        result.subToFullMapping.resize(verticesInSubgraph.size());
        for (const auto &v: verticesInSubgraph) {
            result.subToFullMapping[result.fullToSubMapping[v]] = v;
        }
        return result;
    }

    RPHASTSelection runForKnownVertices(const std::vector<int>& subgraphVertices) {
        verticesInSubgraph.clear();
        for (const auto &v: subgraphVertices) {
            verticesInSubgraph.insert(v);
        }
        // Order vertices by decreasing rank
        std::sort(verticesInSubgraph.begin(), verticesInSubgraph.end(), std::greater<>());

        RPHASTSelection result;
        result.subGraph = constructOrderedSubgraph(verticesInSubgraph, result.fullToSubMapping);
        result.subToFullMapping.resize(verticesInSubgraph.size());
        for (const auto &v: verticesInSubgraph) {
            result.subToFullMapping[result.fullToSubMapping[v]] = v;
        }
        return result;
    }

    // Given a rank in the full graph, this returns the distance from v to the closest target in the last call to run()
    // or INFTY if v was not reached.
    int getDistanceToClosestTarget(const int v) const requires WithPruning {
        KASSERT(v >= 0);
        KASSERT(v < fullGraph.numVertices());
        return upSearchDistances.readDistance(v);
    }

private:

    void findVerticesInSubgraphOrderedByRanks(const std::vector<int> &targets) {
        findVerticesUsingBfs(targets);

        // Order vertices in subgraph by decreasing rank.
        std::sort(verticesInSubgraph.begin(), verticesInSubgraph.end(), std::greater<>());
    }

    void findVerticesInSubgraphOrderedByLevels(const std::vector<int> &targets, const std::vector<int> &offsets) {
        findVerticesUsingTopoSearch<WithPruning, true>(targets, offsets);

        // Order vertices in subgraph by decreasing level (and by input order within each level).
        std::sort(verticesInSubgraph.begin(), verticesInSubgraph.end(), [&](const auto &v1, const auto &v2) {
            const auto l1 = upSearchLevels[v1];
            const auto l2 = upSearchLevels[v2];
            return l1 > l2 || (l1 == l2 && ch.contractionOrder(v1) < ch.contractionOrder(v2));
        });
    }

    void findVerticesUsingBfs(const std::vector<int> &targets) {
        verticesInSubgraph.clear();
        upBfsQueue.resize(0);
        for (const auto &t: targets) {
            if (verticesInSubgraph.insert(t)) {
                upBfsQueue.push_back(t);
            }
        }
        LIGHT_KASSERT(upBfsQueue.capacity() >= fullGraph.numVertices());
        for (auto it = upBfsQueue.begin(); it != upBfsQueue.end(); ++it) {
            const auto v = *it;
            FORALL_INCIDENT_EDGES(fullGraph, v, e) {
                const auto w = fullGraph.edgeHead(e);
                if (verticesInSubgraph.insert(w)) {
                    upBfsQueue.push_back(w);
                }
            }
        }
    }

    template<bool TrackDistances, bool TrackLevels>
    void findVerticesUsingTopoSearch(const std::vector<int> &targets, const std::vector<int> &offsets) {
        verticesInSubgraph.clear();

        // Initialize topological upward search rooted at all targets with given offsets.
        upSearchPriorityQueue.clear();

        if constexpr (TrackLevels) {
            upSearchLevels.init();
            for (const auto &t: targets) {
                upSearchLevels[t] = 0;
            }
        }

        if constexpr (TrackDistances) {
            upSearchDistances.init();
            for (int i = 0; i < targets.size(); ++i) {
                const auto t = targets[i];
                auto &dist = upSearchDistances[t];
                dist = std::min(dist, offsets[i]);
            }
        }

        for (const auto &t: targets) {
            if (!upSearchPriorityQueue.contains(t))
                upSearchPriorityQueue.insert(t, t);
        }

        // Run topological upward search, populating verticesInSubgraph with all vertices reached
        // while applying pruning criterion.
        while (!upSearchPriorityQueue.empty()) {
            int v, key;
            upSearchPriorityQueue.deleteMin(v, key);

            if constexpr (TrackDistances && WithPruning) {
                auto &distToV = upSearchDistances[v];
                // Check whether the search can be pruned at v.
                if (pruneUpSearch(v, distToV, upSearchDistances))
                    continue;
            }

            KASSERT(!contains(verticesInSubgraph.begin(), verticesInSubgraph.end(), v));
            verticesInSubgraph.insert(v);

            // Relax all edges out of v.
            FORALL_INCIDENT_EDGES(fullGraph, v, e) {
                const auto w = fullGraph.edgeHead(e);

                if (!upSearchPriorityQueue.contains(w))
                    upSearchPriorityQueue.insert(w, w);

                if constexpr (TrackLevels) {
                    const auto levelOfV = upSearchLevels.readDistanceWithoutStaleCheck(v);
                    auto &levelOfW = upSearchLevels[w];
                    levelOfW = std::max(levelOfW, levelOfV + 1);
                }

                if constexpr (TrackDistances) {
                    const auto &distToV = upSearchDistances.readDistanceWithoutStaleCheck(v);
                    auto &distToW = upSearchDistances[w];
                    const auto distViaV = distToV + fullGraph.template get<CH::Weight>(e);
                    distToW = std::min(distToW, distViaV);
                }
            }
        }
    }

    // Constructs a subgraph induced by the given set of vertices. The subgraph is ordered by decreasing vertex rank.
    // TODO: The UnpackingInfoAttribute is not adequately updated for the new edge IDs in the subgraph.
    //  When we want to retrieve paths, we need to implement an appropriate update.
    SearchGraph
    constructOrderedSubgraph(const LightweightSubset &sortedSubgraphVertices, std::vector<int> &origToNewIds) {
        const auto numVertices = sortedSubgraphVertices.size();
        // Invalid mapping is represented by number of vertices in subgraph (i.e. one past last valid vertex).
        origToNewIds = std::vector<int>(fullGraph.numVertices(), numVertices);

        // Assign new sequential IDs to the vertices in the subgraph. Vertex IDs in subgraph will be assigned according
        // to order of passed vertices.

        int nextId = 0;
        for (auto it = sortedSubgraphVertices.begin(); it != sortedSubgraphVertices.end(); ++it)
            origToNewIds[*it] = nextId++;

        // Count the edges in the subgraph.
        int numEdges = 0;
        for (const auto &v: sortedSubgraphVertices) {
            for (int e = fullGraph.firstEdge(v); e != fullGraph.lastEdge(v); ++e)
                numEdges += origToNewIds[fullGraph.edgeHead(e)] != numVertices;
        }

        AlignedVector<SearchGraph::OutEdgeRange> outEdges(nextId + 1);
        AlignedVector<int32_t> edgeHeads(numEdges);
        AlignedVector<typename CH::Weight::Type> weights(numEdges);
        AlignedVector<typename UnpackingInfoAttribute::Type> unpackingInfo(numEdges);

        int edgeCount = 0;
        for (int i = 0; i < numVertices; ++i) {
            const auto u = *(sortedSubgraphVertices.begin() + i);
            // Copy the current vertex belonging to the subgraph.
            outEdges[i].first() = edgeCount;

            // Copy the edges out of u going to vertices belonging to the subgraph.
            // Edges in full graph are ordered by increasing rank of head vertex. We order edges in subgraph
            // by decreasing rank of head vertex.
            for (int e = fullGraph.lastEdge(u) - 1; e >= fullGraph.firstEdge(u); --e) {
                const int v = origToNewIds[fullGraph.edgeHead(e)];
                if (v != numVertices) {
                    edgeHeads[edgeCount] = v;
                    weights[edgeCount] = fullGraph.template get<typename CH::Weight>(e);
                    unpackingInfo[edgeCount] = fullGraph.template get<UnpackingInfoAttribute>(e);
                    ++edgeCount;
                }
            }
        }
        KASSERT(edgeCount == numEdges);

        outEdges.back().last() = edgeCount;
        return SearchGraph(std::move(outEdges), std::move(edgeHeads), edgeCount, std::move(weights),
                           std::move(unpackingInfo));
    }

    const SearchGraph &fullGraph;
    const CH &ch;
    LightweightSubset verticesInSubgraph;

//    std::vector<int> verticesInSubgraph;

    std::vector<int> upBfsQueue;

    using PriorityQueue = AddressableQuadHeap;
    PriorityQueue upSearchPriorityQueue; // The priority queue of unsettled vertices in the upward selection search.
    StampedDistanceLabelContainer<int> upSearchLevels;

    StampedDistanceLabelContainer<int> upSearchDistances; // The distance labels of the vertices in the upward selection search. Used if pruning is active.
    PruningCriterionT pruneUpSearch;    // The criterion used to prune the upward selection search.
};

