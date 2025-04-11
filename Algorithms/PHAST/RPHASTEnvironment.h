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
#include "DataStructures/Containers/LightweightSubset.h"
#include "PHASTQuery.h"

class RPHASTEnvironment {

public:
    using SearchGraph = typename CH::SearchGraph;

    RPHASTEnvironment(const CH &ch) : upwardGraph(ch.upwardGraph()), downwardGraph(ch.downwardGraph()),
                                      selectionQueue(), verticesInSubgraph(ch.upwardGraph().numVertices()),
                                      sourcesSubgraph(), sourcesFullToSubMapping(),
                                      targetsSubgraph(), targetsFullToSubMapping() {
        KASSERT(upwardGraph.numVertices() == downwardGraph.numVertices());
        selectionQueue.reserve(ch.upwardGraph().numVertices());

        // Make sure that vertices in upward graph and downward graph are ordered by increasing rank.
        FORALL_VERTICES(upwardGraph, u) {
            int prevEdgeHead = -1;
            FORALL_INCIDENT_EDGES(upwardGraph, u, e) {
                const auto v = upwardGraph.edgeHead(e);
                LIGHT_KASSERT(u < v);
                LIGHT_KASSERT(v > prevEdgeHead);
                prevEdgeHead = v;
            }
        }
        FORALL_VERTICES(downwardGraph, u) {
            int prevEdgeHead = -1;
            FORALL_INCIDENT_EDGES(downwardGraph, u, e) {
                const auto v = downwardGraph.edgeHead(e);
                LIGHT_KASSERT(u < v);
                LIGHT_KASSERT(v > prevEdgeHead);
                prevEdgeHead = v;
            }
        }
    }


    // Runs RPHAST target selection phase for given sources in upward graph.
    // Given sources should be ranks in the CH.
    void runSourceSelection(const std::vector<int> &sources) {
        int numEdgesInSubgraph = 0;
        findVerticesInSubgraph(upwardGraph, sources, numEdgesInSubgraph);
        sourcesSubgraph = constructOrderedSubgraph(upwardGraph, verticesInSubgraph, numEdgesInSubgraph,
                                                   sourcesFullToSubMapping);
        sourcesSubToFullMapping.resize(verticesInSubgraph.size());
        for (const auto& v : verticesInSubgraph)
            sourcesSubToFullMapping[sourcesFullToSubMapping[v]] = v;
    }

    // Runs RPHAST target selection phase for given targets in downward graph.
    // Given targets should be ranks in the CH.
    void runTargetSelection(const std::vector<int> &targets) {
        int numEdgesInSubgraph = 0;
        findVerticesInSubgraph(downwardGraph, targets, numEdgesInSubgraph);
        targetsSubgraph = constructOrderedSubgraph(downwardGraph, verticesInSubgraph, numEdgesInSubgraph,
                                                   targetsFullToSubMapping);
        targetsSubToFullMapping.resize(verticesInSubgraph.size());
        for (const auto& v : verticesInSubgraph)
            targetsSubToFullMapping[targetsFullToSubMapping[v]] = v;
    }

    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    using Query = PHASTQuery<SearchGraph, SearchGraph, typename CH::Weight, LabelSetT>;

    // Returns a forward RPHAST query that uses the result of the target selection phase completed by the last
    // call to runTargetSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    Query<LabelSetT> getForwardRPHASTQuery() const {
        return Query<LabelSetT>(upwardGraph, targetsSubgraph, targetsFullToSubMapping, targetsSubToFullMapping);
    }

    // Returns a reverse RPHAST query that uses the result of the target selection phase completed by the last
    // call to runSourceSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    Query<LabelSetT> getReverseRPHASTQuery() const {
        return Query<LabelSetT>(downwardGraph, sourcesSubgraph, sourcesFullToSubMapping, sourcesSubToFullMapping);
    }

private:

    void findVerticesInSubgraph(const SearchGraph &graph, const std::vector<int> &sources, int &numEdgesInSubgraph) {
        verticesInSubgraph.clear();
        selectionQueue.resize(0);
        numEdgesInSubgraph = 0;
        for (const auto &s: sources) {
            if (verticesInSubgraph.insert(s)) {
                selectionQueue.push_back(s);
            }
        }
        LIGHT_KASSERT(selectionQueue.capacity() >= graph.numVertices());
        for (auto it = selectionQueue.begin(); it != selectionQueue.end(); ++it) {
            const auto v = *it;
            FORALL_INCIDENT_EDGES(graph, v, e) {
                ++numEdgesInSubgraph;
                const auto w = graph.edgeHead(e);
                if (verticesInSubgraph.insert(w)) {
                    selectionQueue.push_back(w);
                }
            }
        }
    }

    // Constructs a subgraph induced by the given set of vertices. The subgraph is ordered by decreasing vertex rank.
    // TODO: The UnpackingInfoAttribute is not adequately updated for the new edge IDs in the subgraph.
    //  When we want to retrieve paths, we need to implement an appropriate update.
    SearchGraph
    constructOrderedSubgraph(const SearchGraph &graph, LightweightSubset &vertices, const int &numEdgesInSubgraph,
                             std::vector<int> &origToNewIds) {
        // Invalid mapping is represented by number of vertices in subgraph (i.e. one past last valid vertex).
        origToNewIds = std::vector<int>(graph.numVertices(), vertices.size());

        // Assign new sequential IDs to the vertices in the subgraph. Make sure that IDs are assigned in
        // decreasing order of the original vertex ranks.
        std::sort(vertices.begin(), vertices.end(), std::greater<>());
        int nextId = 0;
        for (auto it = vertices.begin(); it != vertices.end(); ++it)
            origToNewIds[*it] = nextId++;

        AlignedVector<SearchGraph::OutEdgeRange> outEdges(nextId + 1);
        AlignedVector<int32_t> edgeHeads(numEdgesInSubgraph);
        AlignedVector<typename CH::Weight::Type> weights(numEdgesInSubgraph);
        AlignedVector<typename UnpackingInfoAttribute::Type> unpackingInfo(numEdgesInSubgraph);
//        subgraph.outEdges.resize(nextId + 1);
//        subgraph.edgeHeads.resize(numEdgesInSubgraph);
//        RUN_FORALL(subgraph.VertexAttributes::values.resize(nextId));
//        RUN_FORALL(subgraph.EdgeAttributes::values.resize(numEdgesInSubgraph));

//        int &edgeCount = subgraph.edgeCount;
        int edgeCount = 0;
        for (int i = 0; i < vertices.size(); ++i) {
            const auto u = *(vertices.begin() + i);
            // Copy the current vertex belonging to the subgraph.
//            subgraph.outEdges[i].first() = edgeCount;
//            RUN_FORALL(subgraph.VertexAttributes::values[i] = VertexAttributes::values[u]);
            outEdges[i].first() = edgeCount;

            // Copy the edges out of u going to vertices belonging to the subgraph.
            // Edges in full graph are ordered by increasing rank of head vertex. We order edges in subgraph
            // by decreasing rank of head vertex.
            for (int e = graph.lastEdge(u) - 1; e >= graph.firstEdge(u); --e) {
                const int v = origToNewIds[graph.edgeHead(e)];
                if (v != vertices.size()) {
//                    subgraph.edgeHeads[edgeCount] = v;
//                    RUN_FORALL(subgraph.EdgeAttributes::values[edgeCount] = EdgeAttributes::values[e]);
                    edgeHeads[edgeCount] = v;
                    weights[edgeCount] = graph.template get<typename CH::Weight>(e);
                    unpackingInfo[edgeCount] = graph.template get<UnpackingInfoAttribute>(e);
                    ++edgeCount;
                }
            }
        }
        KASSERT(edgeCount == numEdgesInSubgraph);

        outEdges.back().last() = edgeCount;
//        subgraph.outEdges.back().last() = edgeCount;
        return SearchGraph(std::move(outEdges), std::move(edgeHeads), edgeCount, std::move(weights),
                           std::move(unpackingInfo));
    }


    // Full CH search graphs with vertices ordered by increasing rank. Used for query side ("one" side of one-to-many) of RPHAST queries.
    SearchGraph upwardGraph;
    SearchGraph downwardGraph;

    std::vector<int> selectionQueue;
    LightweightSubset verticesInSubgraph;

    // Subgraphs of the upward and downward search graphs with vertices ordered by decreasing rank.
    // Used for the target side ("many" side of one-to-many) of RPHAST queries.
    // Full-to-sub mappings map each vertex ID in the full graph to the according ID in the subgraph if present.
    // If vertex v is not present in the subgraph, its mapping is subGraph.numVertices() to mark it as invalid.
    // Sub-to-full mappings map each vertex ID in the subgraph to the according ID in the full graph.
    // Subgraphs and mappings are constructed for a specific set of sources/targets passed to
    // runSourceSelection/runTargetSelection.
    SearchGraph sourcesSubgraph;
    std::vector<int> sourcesFullToSubMapping;
    std::vector<int> sourcesSubToFullMapping;
    SearchGraph targetsSubgraph;
    std::vector<int> targetsFullToSubMapping;
    std::vector<int> targetsSubToFullMapping;

};