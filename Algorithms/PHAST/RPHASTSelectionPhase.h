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
    RPHASTSelection(const RPHASTSelection&) = delete;
    RPHASTSelection& operator=(const RPHASTSelection&) = delete;

    RPHASTSelection& operator=(RPHASTSelection&& other) {
        if (this != &other) {
            subGraph = std::move(other.subGraph);
            fullToSubMapping = std::move(other.fullToSubMapping);
            subToFullMapping = std::move(other.subToFullMapping);
        }
        return *this;
    }

    RPHASTSelection(RPHASTSelection&& other)
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

template<typename PruningCriterionT = dij::NoCriterion>
class RPHASTSelectionPhase {

    static constexpr bool WithPruning = !std::is_same_v<PruningCriterionT, dij::NoCriterion>;

    struct RememberSearchSpace {

        explicit RememberSearchSpace(LightweightSubset& searchSpace)
                : searchSpace(searchSpace) {}

        template<typename DistLabelT, typename DistLabelContT>
        bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
            searchSpace.insert(v);
            return false;
        }

    private:
        LightweightSubset& searchSpace;

    };

    using SearchGraph = typename CH::SearchGraph;
    using SelectionSearch = DagShortestPaths<SearchGraph, typename CH::Weight, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, dij::CompoundCriterion<PruningCriterionT, RememberSearchSpace>>;

    bool validateVerticesInIncreasingRankOrder(const SearchGraph& graph) {
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

    explicit RPHASTSelectionPhase(const SearchGraph& fullGraph, PruningCriterionT prune = {}) : fullGraph(fullGraph),
                                                                             verticesInSubgraph(fullGraph.numVertices()), selectionQueue(),
                                                                             selectionSearch(fullGraph, {prune, RememberSearchSpace(verticesInSubgraph)}) {
        if constexpr (!WithPruning) {
            selectionQueue.reserve(fullGraph.numVertices());
        }
        KASSERT(validateVerticesInIncreasingRankOrder(fullGraph));
    }

    RPHASTSelection run(const std::vector<int> &targets, const std::vector<int>& offsets = {}) {
        findVerticesInSubgraph(targets, offsets);
        RPHASTSelection result;
        result.subGraph = constructOrderedSubgraph(verticesInSubgraph, result.fullToSubMapping);
        result.subToFullMapping.resize(verticesInSubgraph.size());
        for (const auto& v : verticesInSubgraph)
            result.subToFullMapping[result.fullToSubMapping[v]] = v;
        return result;
    }

    // Given a rank in the full graph, this returns the distance from v to the closest target in the last call to run()
    // or INFTY if v was not reached.
    int getDistanceToClosestTarget(const int v) const requires WithPruning {
        KASSERT(v >= 0);
        KASSERT(v < fullGraph.numVertices());
        return selectionSearch.getDistance(v);
    }

private:

    void findVerticesInSubgraph(const std::vector<int> &targets, const std::vector<int>&) requires (!WithPruning) {
        verticesInSubgraph.clear();
        selectionQueue.resize(0);
        for (const auto &t: targets) {
            if (verticesInSubgraph.insert(t)) {
                selectionQueue.push_back(t);
            }
        }
        LIGHT_KASSERT(selectionQueue.capacity() >= fullGraph.numVertices());
        for (auto it = selectionQueue.begin(); it != selectionQueue.end(); ++it) {
            const auto v = *it;
            FORALL_INCIDENT_EDGES(fullGraph, v, e) {
                const auto w = fullGraph.edgeHead(e);
                if (verticesInSubgraph.insert(w)) {
                    selectionQueue.push_back(w);
                }
            }
        }
    }

    void findVerticesInSubgraph(const std::vector<int> &targets, const std::vector<int>& offsets) requires WithPruning {
        verticesInSubgraph.clear();

        // Initialize topological upward search rooted at all targets with given offsets.
        selectionSearch.numVerticesSettled = 0;
        selectionSearch.numEdgeRelaxations = 0;
        selectionSearch.distanceLabels.init();
        selectionSearch.queue.clear();

        for (int i = 0; i < targets.size(); ++i) {
            const auto t = targets[i];
            selectionSearch.distanceLabels[t] = offsets[i];
        }

        for (const auto& t : targets) {
            if (!selectionSearch.queue.contains(t))
                selectionSearch.queue.insert(t, t);
        }

        // Run topological upward search, populating verticesInSubgraph with all vertices reached
        // while applying pruning criterion.
        while (!selectionSearch.queue.empty())
            selectionSearch.settleNextVertex();
    }

    // Constructs a subgraph induced by the given set of vertices. The subgraph is ordered by decreasing vertex rank.
    // TODO: The UnpackingInfoAttribute is not adequately updated for the new edge IDs in the subgraph.
    //  When we want to retrieve paths, we need to implement an appropriate update.
    SearchGraph
    constructOrderedSubgraph(LightweightSubset &vertices, std::vector<int> &origToNewIds) {
        const auto numVertices = vertices.size();
        // Invalid mapping is represented by number of vertices in subgraph (i.e. one past last valid vertex).
        origToNewIds = std::vector<int>(fullGraph.numVertices(), numVertices);

        // Assign new sequential IDs to the vertices in the subgraph. Make sure that IDs are assigned in
        // decreasing order of the original vertex ranks.
        std::sort(vertices.begin(), vertices.end(), std::greater<>());
        int nextId = 0;
        for (auto it = vertices.begin(); it != vertices.end(); ++it)
            origToNewIds[*it] = nextId++;

        // Count the edges in the subgraph.
        int numEdges = 0;
        for (const auto& v : vertices) {
            for (int e = fullGraph.firstEdge(v); e != fullGraph.lastEdge(v); ++e)
                numEdges += origToNewIds[fullGraph.edgeHead(e)] != numVertices;
        }

        AlignedVector<SearchGraph::OutEdgeRange> outEdges(nextId + 1);
        AlignedVector<int32_t> edgeHeads(numEdges);
        AlignedVector<typename CH::Weight::Type> weights(numEdges);
        AlignedVector<typename UnpackingInfoAttribute::Type> unpackingInfo(numEdges);

        int edgeCount = 0;
        for (int i = 0; i < numVertices; ++i) {
            const auto u = *(vertices.begin() + i);
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

    const SearchGraph& fullGraph;
    LightweightSubset verticesInSubgraph;

    std::vector<int> selectionQueue;
    SelectionSearch selectionSearch;

};

