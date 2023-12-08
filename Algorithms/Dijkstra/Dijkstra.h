/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include <cassert>
#include <cstdint>
#include <tuple>
#include <vector>

#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/Containers/ParentLabelContainer.h"
#include "DataStructures/Labels/Containers/StampedDistanceLabelContainer.h"
#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/Constants.h"

namespace dij {

// A dummy criterion for Dijkstra's algorithm that does nothing at all.
    struct NoCriterion {
        // Returns always false.
        template<typename... T>
        bool operator()(const T &...) const noexcept {
            return false;
        }
    };

// A criterion for Dijkstra's algorithm that is composed of multiple separate criteria. A compound
// criterion is met if any of the constituting criteria is met.
    template<typename... Criterions>
    struct CompoundCriterion {
        // Constructs a compound criterion from the specified separate criteria.
        CompoundCriterion(const Criterions... criterions) : criterions(criterions...) {}

        // Returns true if any of the constituting criteria is met.
        template<typename DistanceLabelT, typename DistanceLabelContainerT>
        bool operator()(const int v, DistanceLabelT &distToV, DistanceLabelContainerT &distanceLabels) {
            return std::apply([&](auto... criterion) {
                return (... || criterion(v, distToV, distanceLabels));
            }, criterions);
        }

        std::tuple<Criterions...> criterions; // The criteria that constitute the compound criterion.
    };

    struct NoEdgeRelaxationCallBack {

        // No-op callback for edge relaxations.
        template<typename... T>
        void operator()(const T & ...) const noexcept {}
    };
}



// Forward declarations for friend
namespace karri {
    template<typename, typename, bool>
    class EllipticBucketsEnvironment;

    template<typename, typename>
    class FindPDLocsInRadiusQuery;

    template<typename, typename, typename, typename, typename>
    class ClosestPDLocToLastStopBCHQuery;
}



// Implementation of Dijkstra's shortest-path algorithm. Depending on the used label set, it
// keeps parent vertices and/or edges, and computes multiple shortest paths simultaneously,
// optionally using SSE or AVX instructions. The algorithm can be used with different distance label
// containers and priority queues. Moreover, the caller can provide an own stopping and/or pruning
// criterion.
template<
        typename GraphT, typename WeightT, typename LabelSetT,
        typename StoppingCriterionT = dij::NoCriterion,
        typename PruningCriterionT = dij::NoCriterion,
        typename EdgeRelaxationCallBackT = dij::NoEdgeRelaxationCallBack,
        template<typename> class DistanceLabelContainerT = StampedDistanceLabelContainer,
        typename QueueT = AddressableQuadHeap>
class Dijkstra {
    // Some classes are allowed to execute a Dijkstra search step by step.
    template<typename>
    friend
    class DijkstraOpportunityChooser;

    template<typename>
    friend
    class FormulaDemandCalculator;

    template<typename, template<typename> class>
    friend
    class BiDijkstra;

    template<typename, typename>
    friend
    class ODPairGenerator;


    template<typename, typename, bool>
    friend
    class karri::EllipticBucketsEnvironment;

    template<typename, typename>
    friend
    class karri::FindPDLocsInRadiusQuery;

    template<typename, typename, typename, typename, typename>
    friend
    class karri::ClosestPDLocToLastStopBCHQuery;

private:
    using Graph = GraphT;                                    // The graph we work on.
    using DistanceLabel = typename LabelSetT::DistanceLabel; // The distance label of a vertex.
    using ParentLabel = typename LabelSetT::ParentLabel;     // The parent label of a vertex.
    using Queue = QueueT;                                    // The priority queue type.
    using PruningCriterion = PruningCriterionT;              // The criterion to prune search.

    static constexpr int K = LabelSetT::K; // The number of simultaneous shortest-path computations.

public:
    // Constructs a Dijkstra instance.
    explicit Dijkstra(
            const Graph &graph, StoppingCriterionT stopSearch = {}, PruningCriterionT pruneSearch = {}, EdgeRelaxationCallBackT edgeRelaxationCallBack = {})
            : graph(graph),
              distanceLabels(graph.numVertices()),
              parent(graph),
              queue(graph.numVertices()),
              stopSearch(stopSearch),
              pruneSearch(pruneSearch),
              edgeRelaxationCallBack(edgeRelaxationCallBack),
              numEdgeRelaxations(0),
              numVerticesSettled(0) {}

    // Runs a Dijkstra search from s.
    void run(const int s) {
        runWithOffset(s, 0);
    }

    // Runs a Dijkstra search from s to t.
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

    // Runs a Dijkstra search that computes multiple shortest paths simultaneously.
    void run(const std::array<int, K> &sources, const std::array<int, K> &targets) {
        init(sources);
        while (!queue.empty()) {
            // Stop the search as soon as Q.minKey >= d_i(t_i) for all i = 1, ..., k.
            auto stop = true;
            for (auto i = 0; i < K; ++i)
                stop &= queue.minKey() >= distanceLabels[targets[i]][i];
            if (stop)
                break;
            settleNextVertex();
        }
    }

    // Runs a Dijkstra search from multiple sources s, with the distances of the sources initialized to the given
    // offsets.
    void runWithOffset(const std::array<int, K> &sources, const std::array<int, K> &offsets) {
        init(sources, offsets);
        while (!queue.empty()) {
            if (stopSearch(queue.minId(), distanceLabels[queue.minId()], distanceLabels))
                break;
            settleNextVertex();
        }
    }

    // Runs a Dijkstra search from s, with the distance of s initialized to the given offset.
    void runWithOffset(const int s, const int offset) {
        std::array<int, K> sources;
        std::array<int, K> offsets;
        sources.fill(s);
        offsets.fill(offset);
        runWithOffset(sources, offsets);
    }

    // Runs multiple Dijkstra searches from different sources simultaneously until queue is exhausted (i.e. stopping
    // criterion is not used).
    void runWithOffsetUntilExhaustion(const std::array<int, K> &sources, const std::array<int, K> &offsets) {
        static_assert(std::is_same<StoppingCriterionT, dij::NoCriterion>::value,
                      "Dijkstra until exhaustion cannot utilize a stopping criterion. StoppingCriterionT must be dij::NoCriterion");
        init(sources, offsets);
        while (!queue.empty()) {
            settleNextVertex();
        }
    }

    void runWithOffsetUntilExhaustion(const int s, const int offset) {
        static_assert(std::is_same<StoppingCriterionT, dij::NoCriterion>::value,
                      "Dijkstra until exhaustion cannot utilize a stopping criterion. StoppingCriterionT must be dij::NoCriterion");
        std::array<int, K> sources;
        std::array<int, K> offsets;
        sources.fill(s);
        offsets.fill(offset);
        init(sources, offsets);
        while (!queue.empty()) {
            settleNextVertex();
        }
    }


    // Returns the shortest-path distance from the i-th source to t.
    int getDistance(const int t, const int i = 0) {
        return distanceLabels[t][i];
    }

    // Returns the parent vertex of v on the shortest path from the i-th source to v.
    int getParentVertex(const int v, const int i = 0) {
        assert(distanceLabels[v][i] != INFTY);
        return parent.getVertex(v, i);
    }

    // Returns the parent edge of v on the shortest path from the i-th source to v.
    int getParentEdge(const int v, const int i = 0) {
        assert(distanceLabels[v][i] != INFTY);
        return parent.getEdge(v, i);
    }

    // Returns the vertices on the shortest path from the i-th source to t in reverse order.
    const std::vector<int32_t> &getReversePath(const int t, const int i = 0) {
        assert(distanceLabels[t][i] != INFTY);
        return parent.getReversePath(t, i);
    }

    // Returns the edges on the shortest path from the i-th source to t in reverse order.
    const std::vector<int32_t> &getReverseEdgePath(const int t, const int i = 0) {
        assert(distanceLabels[t][i] != INFTY);
        return parent.getReverseEdgePath(t, i);
    }


    const int &getNumEdgeRelaxations() const {
        return numEdgeRelaxations;
    }

    const int &getNumVerticesSettled() const {
        return numVerticesSettled;
    }

private:
    // Resets the distance labels and inserts all k simultaneous sources into the queue.
    void init(const std::array<int, K> &sources, const std::array<int, K> &offsets = {}) {
        numEdgeRelaxations = 0;
        numVerticesSettled = 0;
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
                queue.insert(s, distanceLabels[s].getKey());
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
            edgeRelaxationCallBack(e, v, w, distanceLabels);
            auto &distToW = distanceLabels[w];
            const auto distViaV = distToV + graph.template get<WeightT>(e);
            const auto mask = distViaV < distToW;
            if (anySet(mask)) {
                distToW.min(distViaV);
                parent.setVertex(w, v, mask);
                parent.setEdge(w, e, mask);
                if (queue.contains(w))
                    queue.decreaseKey(w, distToW.getKey());
                else
                    queue.insert(w, distToW.getKey());
            }
        }
        return v;
    }

    // Used by friend classes to update the stopping criterion for different runs of this search
    StoppingCriterionT &getStoppingCriterion() {
        return stopSearch;
    }

    // Used by friend classes to update the pruning criterion for different runs of this search
    PruningCriterionT &getPruningCriterion() {
        return pruneSearch;
    }

    using DistanceLabelCont = DistanceLabelContainerT<typename LabelSetT::DistanceLabel>;
    using ParentLabelCont = ParentLabelContainer<Graph, LabelSetT>;

    const Graph &graph;               // The graph on which we compute shortest paths.
    DistanceLabelCont distanceLabels; // The distance labels of the vertices.
    ParentLabelCont parent;           // The parent information for each vertex.
    Queue queue;                      // The priority queue of unsettled vertices.
    StoppingCriterionT stopSearch;    // The criterion used to stop the search.
    PruningCriterionT pruneSearch;    // The criterion used to prune the search.
    EdgeRelaxationCallBackT edgeRelaxationCallBack; // Called whenever an edge is relaxed.

    int numEdgeRelaxations; // Number of edge relaxations in last run.
    int numVerticesSettled; // Number of vertices settled in last run.
};
