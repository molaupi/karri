/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <vector>
#include <cstdint>
#include <random>
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "Algorithms/GraphTraversal/StronglyConnectedComponents.h"
#include "DataStructures/Utilities/IteratorRange.h"

namespace traffic_flow_subnetwork {

    template<typename VertexAttributes, typename EdgeAttributes, typename WeightT>
    class KeptEdgesConnector;

    // This class takes the edges for a traffic flow based subnetwork returned by the KeptEdgesFinder and connects them
    // to a strongly connected subnetwork of the original road network.
    template<typename ...VertexAttributes, typename ...EdgeAttributes, typename WeightT>
    class KeptEdgesConnector<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>, WeightT> {

        using InGraph = StaticGraph<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>>;
        using OutGraph = DynamicGraph<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>>;

        using Search = Dijkstra<InGraph, WeightT, BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>;

    public:

        KeptEdgesConnector(const InGraph &in) : in(in), scc(), search(in) {}

        OutGraph connectEdges(std::vector<int> &keptEdges) {

            // Get edge-induced subgraph
            std::vector<int> inToOutVertexIds(in.numVertices(), INVALID_VERTEX);
            auto out = getInitialOutGraph(keptEdges, inToOutVertexIds);
            KASSERT(out.validate());

            // Compute out-to-in vertex ID mapping.
            std::vector<int> outToInVertexIds(out.numVertices(), INVALID_VERTEX);
            for (int vIn = 0; vIn < in.numVertices(); ++vIn) {
                const auto &vOut = inToOutVertexIds[vIn];
                if (vOut != INVALID_VERTEX)
                    outToInVertexIds[vOut] = vIn;
            }


            // Compute SCCs of subgraph induced by kept edges.
            scc.run(out);
            std::vector<int> reprOfVertex = scc.getStronglyConnectedComponents();
            std::vector<int> uniqueReprs;
            std::unordered_map<int, int> reprToComp;
            for (const auto &r: reprOfVertex) {
                if (!contains(uniqueReprs.begin(), uniqueReprs.end(), r)) {
                    reprToComp[r] = static_cast<int>(uniqueReprs.size());
                    uniqueReprs.push_back(r);
                }
            }
            std::cout << "Subgraph induced by kept edges:" << std::endl;
            std::cout << "\t|V| = " << out.numVertices() << std::endl;
            std::cout << "\t|E| = " << out.numEdges() << std::endl;
            std::cout << "\tNumber of SCCs: " << uniqueReprs.size() << std::endl;
            std::cout << "\tSize of largest SCC: " << scc.getLargestSccAsBitmask().count() << std::endl;

            using Path = std::vector<int>;

            // Iteratively connect SCCs. In each iteration, for each remaining SCC C, a Dijkstra search rooted at all
            // vertices in C is run in the input graph. The Dijkstra search stops once a vertex that belongs to a
            // different SCC in the subgraph is settled. After running a search for each SCC, we find cycles in the
            // component graph that would be created by inserting the paths found by the Dijkstra searches.
            // Each cycle is collapsed by actually inserting the according paths found by the searches into the
            // subgraph and merging the SCCs. Note that there is always at least one new cycle so the iteration converges.
            int iteration = 0;
            while (uniqueReprs.size() > 1) {
                ++iteration;

                const int numComponents = uniqueReprs.size();
                // There are N = uniqueReprs.size() many SCCs. We identify each SCC either by its index c in [0,n).

                // touched[c] contains c' if the search for component c found the closest neighbor component c'
                std::vector<int> touched(numComponents);

                // inPathToTouched[c] contains the shortest path from a vertex of component c to a vertex of the closest
                // neighbor component touched[c].
                std::vector<Path> inPathToTouched(numComponents);

                for (int c = 0; c < numComponents; ++c) {
                    const auto &r = uniqueReprs[c];

                    // Root search at all vertices in SCC
                    search.distanceLabels.init();
                    search.queue.clear();
                    for (int v = 0; v < reprOfVertex.size(); ++v) {
                        if (reprOfVertex[v] == r) {
                            const auto &vIn = outToInVertexIds[v];
                            search.distanceLabels[vIn][0] = 0;
                            search.queue.insert(vIn, 0);
                            search.parent.setVertex(vIn, vIn, true);
                            search.parent.setEdge(vIn, INVALID_EDGE, true);
                        }
                    }

                    // Run search in input graph until a vertex that is in a different SCC of the subgraph is settled.
                    // Store touched other component and path to the vertex in the input graph.
                    while (true) {
                        LIGHT_KASSERT(!search.queue.empty());
                        const int vIn = search.queue.minId();
                        const int &vOut = inToOutVertexIds[vIn];
                        if (vOut != INVALID_VERTEX && reprOfVertex[vOut] != r) {
                            touched[c] = reprToComp.at(reprOfVertex[vOut]);
                            inPathToTouched[c] = search.getReverseEdgePath(vIn);
                            std::reverse(inPathToTouched[c].begin(), inPathToTouched[c].end());
                            break;
                        }
                        search.settleNextVertex();
                    }
                    LIGHT_KASSERT(touched[c] >= 0 && touched[c] < numComponents && touched[c] != c);
                    LIGHT_KASSERT(!inPathToTouched[c].empty());
                    LIGHT_KASSERT(reprOfVertex[inToOutVertexIds[in.edgeHead(inPathToTouched[c].back())]] == uniqueReprs[touched[c]]);
                    LIGHT_KASSERT(reprOfVertex[inToOutVertexIds[in.edgeTail(inPathToTouched[c].front())]] == r);
                }
                LIGHT_KASSERT(!contains(touched.begin(), touched.end(), INVALID_VERTEX));

                // The paths found by Dijkstra searches induce a pseudo-forest in the component graph of the subnetwork.
                // We find the cycle in each pseudo-tree and collapse it by inserting the according paths into the
                // subnetwork and updating the components.
                // All components of a pseudo-tree that are not on the cycle of that pseudo-tree cannot be collapsed
                // in this iteration.
                std::vector<int> curCycle;
                std::vector<int> reprsMarkedForRemoval;
                for (int initialC = 0; initialC < numComponents; ++initialC) {
                    LIGHT_KASSERT(curCycle.empty());

                    if (touched[initialC] == INVALID_VERTEX) {
                        // Cycle in pseudo-tree of initialC has already been found.
                        continue;
                    }

                    // Try to find a new cycle:
                    curCycle.push_back(initialC);
                    curCycle.push_back(touched[initialC]);
                    while (true) {
                        const int nextC = touched[curCycle.back()];

                        // If we hit a component that has invalid vertex, the cycle for this pseudo-tree has already
                        // been found.
                        if (nextC == INVALID_VERTEX) {
                            for (const auto &c2: curCycle)
                                touched[c2] = INVALID_VERTEX;
                            curCycle.clear();
                            break;
                        }

                        const auto startOfCycle = std::find(curCycle.begin(), curCycle.end(), nextC);

                        // If we do not close a cycle, append next component.
                        if (startOfCycle == curCycle.end()) {
                            curCycle.push_back(nextC);
                            continue;
                        }

                        // Close a cycle:

                        // Remove components from curCycle that are not part of the cycle. Mark them as finished.
                        for (auto it = curCycle.begin(); it != startOfCycle; ++it)
                            touched[*it] = INVALID_VERTEX;
                        curCycle.erase(curCycle.begin(), startOfCycle);
                        LIGHT_KASSERT(curCycle.front() == nextC);

                        // Insert the Dijkstra paths into the subnetwork:
                        insertPathsIntoSubnetwork(out, curCycle, inPathToTouched, inToOutVertexIds, outToInVertexIds);

                        // Store representative of new vertices (new vertices are added at end with incremental IDs):
                        const int newRepr = uniqueReprs[nextC];
                        reprOfVertex.resize(out.numVertices(), newRepr);

                        // Update the representatives of existing vertices:
                        for (int v = 0; v < out.numVertices(); ++v) {
                            if (contains(curCycle.begin(), curCycle.end(), reprToComp.at(reprOfVertex[v])))
                                reprOfVertex[v] = newRepr;
                        }

                        // Mark components on cycle for removal (except expanded first component):
                        for (auto i = 1; i < curCycle.size(); ++i)
                            reprsMarkedForRemoval.push_back(uniqueReprs[curCycle[i]]);

                        // Mark components on cycle as finished for this iteration and reset cycle:
                        for (const auto &c2: curCycle)
                            touched[c2] = INVALID_VERTEX;
                        curCycle.clear();
                        break;
                    }

                }

                // Remove representatives of components that were on collapsed cycles and did not become new component:
                for (const auto& r : reprsMarkedForRemoval) {
                    uniqueReprs.erase(std::find(uniqueReprs.begin(), uniqueReprs.end(), r));
                    reprToComp.erase(r);
                }
                for (int c = 0; c < uniqueReprs.size(); ++c) {
                    reprToComp[uniqueReprs[c]] = c;
                }

                std::cout << "After iteration " << iteration << ": " << uniqueReprs.size() << " SCCs" << std::endl;
            }


            LIGHT_KASSERT(hasOneScc(out));
            std::cout << "Connected subnetwork in " << iteration << " iterations." << std::endl;

            out.defrag();
            return out;
        }


    private:

        OutGraph getInitialOutGraph(const std::vector<int> &keptEdges,
                                    std::vector<int> &inToOutVertexIds) const {
            OutGraph out(in.numVertices(), in.numEdges());

            // Add vertices and memorize mapping from old to new vertex IDs.
            for (const auto &e: keptEdges) {
                const int u = in.edgeTail(e);
                LIGHT_KASSERT(u >= 0 && u < inToOutVertexIds.size());
                if (inToOutVertexIds[u] == INVALID_VERTEX) {
                    inToOutVertexIds[u] = out.appendVertex();
                    RUN_FORALL(out.template get<VertexAttributes>(
                            inToOutVertexIds[u]) = in.template get<VertexAttributes>(u));
                }

                const int v = in.edgeHead(e);
                LIGHT_KASSERT(v >= 0 && v < inToOutVertexIds.size());
                if (inToOutVertexIds[v] == INVALID_VERTEX) {
                    inToOutVertexIds[v] = out.appendVertex();
                    RUN_FORALL(out.template get<VertexAttributes>(
                            inToOutVertexIds[v]) = in.template get<VertexAttributes>(v));
                }
            }

            // Add edges. (Mapping from old to new edge IDs will be computed later since they change in eventual
            // defragmentation).
            for (const auto &e: keptEdges) {
                const auto u = inToOutVertexIds[in.edgeTail(e)];
                const auto v = inToOutVertexIds[in.edgeHead(e)];
                LIGHT_KASSERT(u != INVALID_VERTEX && v != INVALID_VERTEX);
                const int idx = out.insertEdge(u, v);
                RUN_FORALL(out.template get<EdgeAttributes>(idx) = in.template get<EdgeAttributes>(e));
            }

            return out;
        }

        void insertPathsIntoSubnetwork(OutGraph &out,
                                       const std::vector<int> &pathIndices,
                                       const std::vector<std::vector<int>> &paths,
                                       std::vector<int> &inToOutVertexIds,
                                       std::vector<int> &outToInVertexIds) {

            for (int idxInIndices = 0; idxInIndices < pathIndices.size(); ++idxInIndices) {
                const auto& idx = pathIndices[idxInIndices];
                LIGHT_KASSERT(idx >= 0 && idx < paths.size());
                // p is a vector of edges in the input network that we need to insert into the output subnetwork.
                const auto &p = paths[idx];
                LIGHT_KASSERT(inToOutVertexIds[in.edgeTail(p.front())] != INVALID_VERTEX);
                LIGHT_KASSERT(inToOutVertexIds[in.edgeHead(p.back())] != INVALID_VERTEX);

                // Add vertices on path:
                for (int i = 0; i < p.size() - 1; ++i) {
                    const int v = in.edgeHead(p[i]);
                    LIGHT_KASSERT(v >= 0 && v < inToOutVertexIds.size());
                    if (inToOutVertexIds[v] != INVALID_VERTEX)
                        continue; // vertex was introduced by previously inserted path in same call to this method
                    const int newOutV = out.appendVertex();
                    inToOutVertexIds[v] = newOutV;
                    if (outToInVertexIds.size() < newOutV + 1)
                        outToInVertexIds.resize(newOutV + 1, INVALID_VERTEX);
                    outToInVertexIds[newOutV] = v;
                    RUN_FORALL(out.template get<VertexAttributes>(newOutV) = in.template get<VertexAttributes>(v));
                }

                // Add edges on path:
                for (const auto &e: p) {
                    const auto u = inToOutVertexIds[in.edgeTail(e)];
                    const auto v = inToOutVertexIds[in.edgeHead(e)];
                    LIGHT_KASSERT(u != INVALID_VERTEX && v != INVALID_VERTEX);
                    const int outEdgeIdx = out.insertEdge(u, v);
                    RUN_FORALL(out.template get<EdgeAttributes>(outEdgeIdx) = in.template get<EdgeAttributes>(e));
                }
            }
        }

        bool hasOneScc(const OutGraph& out) {
            scc.run(out);
            return scc.getLargestSccAsBitmask().count() == out.numVertices();
        }

        const InGraph &in;
        StronglyConnectedComponents scc;

        Search search;
    };


}