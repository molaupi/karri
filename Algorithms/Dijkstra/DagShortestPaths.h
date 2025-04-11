/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <cassert>
#include <cstdint>
#include <vector>

#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/Containers/ParentLabelContainer.h"
#include "DataStructures/Labels/Containers/StampedDistanceLabelContainer.h"
#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/Constants.h"


// Forward declarations for friend
namespace karri {
    template<typename, typename, bool>
    class EllipticBucketsEnvironment;
}

// Implementation of a shortest-path search on a directed acyclic graph. The vertices must be
// numbered in topological order. The search works similarly to Dijkstra's algorithm, but processes
// vertices in topological order rather than in increasing order of distance, and thus needs no
// decrease-key operations on the priority queue. Depending on the used label set, it keeps parent
// vertices and/or edges. The search can be used with different distance label containers and
// priority queues. Moreover, the caller can provide an own pruning criterion.
template<
        typename GraphT, typename WeightT, typename LabelSetT,
        typename PruningCriterionT = dij::NoCriterion,
        template<typename> class DistanceLabelContainerT = StampedDistanceLabelContainer,
        typename QueueT = AddressableQuadHeap>
class DagShortestPaths {

    template<typename, typename, bool>
    friend
    class karri::EllipticBucketsEnvironment;

    using DistanceLabel = typename LabelSetT::DistanceLabel; // The distance label of a vertex.
    using ParentLabel = typename LabelSetT::ParentLabel;     // The parent label of a vertex.
    static constexpr int K = LabelSetT::K; // The number of simultaneous shortest-path computations.


public:
    // Constructs a shortest-path search instance for the specified directed acyclic graph.
    explicit DagShortestPaths(const GraphT &graph, PruningCriterionT pruneSearch = {})
            : graph(graph),
              distanceLabels(graph.numVertices()),
              parent(graph),
              queue(graph.numVertices()),
              pruneSearch(pruneSearch) {}

    // Runs a shortest-path search from s.
    void run(const int s) {
        runWithOffset(s, 0);
    }

    // Runs a shortest-path search from s to t.
    void run(const int s, const int t) {
        std::array<int, K> sources;
        sources.fill(s);
        init(sources);
        while (!queue.empty()) {
            if (queue.minId() == t)
                break;
            settleNextVertex();
        }
    }

    // Runs a shortest-path search from s, with the distance of s initialized to the given offset.
    void runWithOffset(const int s, const int offset) {
        std::array<int, K> sources;
        std::array<int, K> offsets;
        sources.fill(s);
        offsets.fill(offset);
        init(s, offset);
        while (!queue.empty())
            settleNextVertex();
    }

    // Runs a shortest-path search from multiple sources s, with the distances of the sources initialized to the given
    // offsets.
    void runWithOffset(const std::array<int, K> &sources, const std::array<int, K> &offsets) {
        init(sources, offsets);
        while (!queue.empty())
            settleNextVertex();
    }

    // Returns the shortest-path distance from the i-th source to t.
    int getDistance(const int t, const int i = 0) const {
        return distanceLabels.readDistance(t)[i];
    }

    // Returns the shortest-path distances from all K sources to t.
    DistanceLabel getDistances(const int t) const {
        return distanceLabels.readDistance(t);
    }

    // Returns the parent vertex of v on the shortest path to v.
    int getParentVertex(const int v) {
        assert(distanceLabels[v][0] != INFTY);
        return parent.getVertex(v);
    }

    // Returns the parent edge of v on the shortest path to v.
    int getParentEdge(const int v) {
        assert(distanceLabels[v][0] != INFTY);
        return parent.getEdge(v);
    }

    // Returns the vertices on the shortest path to t in reverse order.
    const std::vector<int32_t> &getReversePath(const int t) {
        assert(distanceLabels[t][0] != INFTY);
        return parent.getReversePath(t);
    }

    // Returns the edges on the shortest path to t in reverse order.
    const std::vector<int32_t> &getReverseEdgePath(const int t) {
        assert(distanceLabels[t][0] != INFTY);
        return parent.getReverseEdgePath(t);
    }

    // Used to update the pruning criterion for different runs of this search, e.g. to configure callbacks
    PruningCriterionT &getPruningCriterion() {
        return pruneSearch;
    }

    const int &getNumVerticesSettled() const {
        return numVerticesSettled;
    }

    const int &getNumEdgeRelaxations() const {
        return numEdgeRelaxations;
    }

private:
    // Resets the distance labels and inserts the source into the queue.
    void init(const std::array<int, K> &sources, const std::array<int, K> &offsets = {}) {
        numVerticesSettled = 0;
        numEdgeRelaxations = 0;
        distanceLabels.init();
        queue.clear();

        for (auto i = 0; i < K; ++i) {
            const auto s = sources[i];
            distanceLabels[s][i] = offsets[i];
            parent.setVertex(s, s, true);
            parent.setEdge(s, INVALID_EDGE, true);
        }

        for (auto i = 0; i < K; ++i) {
            const auto s = sources[i];
            if (!queue.contains(s))
                queue.insert(s, s);
        }
    }

    // Removes the next vertex from the queue, relaxes its outgoing edges, and returns its ID.
    int settleNextVertex() {
        int v, key;
        queue.deleteMin(v, key);
        auto &distToV = distanceLabels[v];

        // Check whether the search can be pruned at v.
        if (pruneSearch(v, distToV, distanceLabels))
            return v;

        ++numVerticesSettled;

        // Relax all edges out of v.
        FORALL_INCIDENT_EDGES(graph, v, e) {
            ++numEdgeRelaxations;
            const auto w = graph.edgeHead(e);
            auto &distToW = distanceLabels[w];
            const auto distViaV = distToV + graph.template get<WeightT>(e);
            const auto mask = distViaV < distToW;
            if (anySet(mask)) {
                distToW.min(distViaV);
                parent.setVertex(w, v, mask);
                parent.setEdge(w, e, mask);
                if (!queue.contains(w))
                    queue.insert(w, w);
            }
        }
        return v;
    }

    using DistanceLabelCont = DistanceLabelContainerT<typename LabelSetT::DistanceLabel>;
    using ParentLabelCont = ParentLabelContainer<GraphT, LabelSetT>;

    const GraphT &graph;              // The graph (DAG) on which we compute shortest paths.
    DistanceLabelCont distanceLabels; // The distance labels of the vertices.
    ParentLabelCont parent;           // The parent information for each vertex.
    QueueT queue;                     // The priority queue of unsettled vertices.
    PruningCriterionT pruneSearch;    // The criterion used to prune the search.

    int numVerticesSettled;
    int numEdgeRelaxations;
};
