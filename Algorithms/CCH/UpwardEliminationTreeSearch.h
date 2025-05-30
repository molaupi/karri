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

#include <array>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

#include "Algorithms/CH/CH.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/Containers/ParentLabelContainer.h"
#include "DataStructures/Labels/Containers/SimpleDistanceLabelContainer.h"
#include "DataStructures/Queues/TournamentTree.h"
#include "Tools/Constants.h"

namespace elimintree {

// The pruning criterion for a standard upward elimination tree search that computes k shortest-path
// trees simultaneously. We can prune the search at v if v is contained in none of the k perfect
// search spaces, i.e., if d_i(v) = infty for all i = 1, ..., k.
    struct PruningCriterion {
        // Returns true if the search can be pruned at v.
        template<typename DistanceLabelT, typename DistanceLabelContainerT>
        bool operator()(const int, const DistanceLabelT &distToV, const DistanceLabelContainerT &) const {
            return !anySet(distToV < INFTY);
        }
    };

}

// Forward declarations for friend
namespace karri {
    template<typename, typename, bool>
    class EllipticBucketsEnvironment;
}

// Implementation of an upward elimination tree search. It enumerates all vertices on the path in
// the elimination tree from a source vertex to the root, and relaxes the edges out of each vertex.
// Depending on the used label set, it keeps parent vertices and/or edges, and computes multiple
// shortest-path trees simultaneously, optionally using SSE or AVX instructions. The caller can
// provide an own pruning criterion. Moreover, there is a member function that computes a single
// shortest-path tree from multiple sources (note that all sources must lie on the same path in the
// elimination tree).
template<typename LabelSetT, typename PruningCriterionT = elimintree::PruningCriterion>
class UpwardEliminationTreeSearch {
    // Some classes are allowed to execute an upward elimination tree search step by step.
    template<typename>
    friend
    class EliminationTreeQuery;

    template<typename, typename, bool>
    friend
    class karri::EllipticBucketsEnvironment;

private:
    using DistanceLabel = typename LabelSetT::DistanceLabel; // The distance label of a vertex.
    using ParentLabel = typename LabelSetT::ParentLabel;     // The parent label of a vertex.
    using PruningCriterion = PruningCriterionT;              // The criterion to prune search.

    static constexpr int K = LabelSetT::K; // The number of simultaneous shortest-path computations.

public:
    // Constructs an upward elimination tree search instance.
    UpwardEliminationTreeSearch(
            const CH::SearchGraph &searchGraph, const std::vector<int32_t> &eliminationTree,
            PruningCriterionT pruneSearch = {})
            : searchGraph(searchGraph),
              eliminationTree(eliminationTree),
              distanceLabels(searchGraph.numVertices()),
              parent(searchGraph),
              nextVertices({}),
              pruneSearch(pruneSearch) {
        assert(searchGraph.numVertices() == eliminationTree.size());
        lastSources.fill(INVALID_VERTEX);
    }

    // Runs an elimination tree search from s.
    void run(const int s) {
        runWithOffset(s, 0);
    }

    // Runs an elimination tree search from s, stopping when t is reached.
    void run(const int s, const int t) {
        std::array<int, K> sources;
        sources.fill(s);
        resetDistanceLabels();
        init(sources);
        while (nextVertices.minKey() != INVALID_VERTEX) {
            if (nextVertices.minKey() == t)
                break;
            settleNextVertex();
        }
    }

    // Runs an elimination tree search that computes a shortest-path tree from multiple sources.
    template<typename IteratorT>
    void run(IteratorT firstSource, IteratorT lastSource) {
        assert(std::is_sorted(firstSource, lastSource));
        resetDistanceLabels();
        init(firstSource, lastSource);
        while (nextVertices.minKey() != INVALID_VERTEX)
            settleNextVertex();
    }

    // Runs an elimination tree search from s, with the distance of s initialized to the given offset.
    void runWithOffset(const int s, const int offset) {
        std::array<int, K> sources;
        std::array<int, K> offsets;
        sources.fill(s);
        offsets.fill(offset);
        runWithOffset(sources, offsets);
    }

    // Runs a Dijkstra search from multiple sources s, with the distances of the sources initialized to the given
    // offsets.
    void runWithOffset(const std::array<int, K> &sources, const std::array<int, K> &offsets) {
        resetDistanceLabels();
        init(sources, offsets);
        while (nextVertices.minKey() != INVALID_VERTEX)
            settleNextVertex();
    }

//    // Runs an elimination tree search that computes a shortest-path tree from multiple sources with individual offsets.
//    template<typename SourceIteratorT, typename OffsetIteratorT>
//    void runWithOffset(SourceIteratorT firstSource, SourceIteratorT lastSource, OffsetIteratorT firstOffset,
//                       OffsetIteratorT lastOffset) {
//        assert(std::is_sorted(firstSource, lastSource));
//        resetDistanceLabels();
//        init(firstSource, lastSource, firstOffset, lastOffset);
//        while (nextVertices.minKey() != INVALID_VERTEX)
//            settleNextVertex();
//    }

    // Runs multiple searches from different sources simultaneously until queue is exhausted.
    void runWithOffsetUntilExhaustion(const std::array<int, K> &sources, const std::array<int, K> &offsets) {
        resetDistanceLabels();
        init(sources, offsets);
        while (nextVertices.minKey() != INVALID_VERTEX)
            settleNextVertex();
    }

    void runWithOffsetUntilExhaustion(const int s, const int offset) {
        runWithOffset(s, offset);
    }

    // Returns the shortest-path distance from the i-th source to t.
    int getDistance(const int t, const int i = 0) {
        return distanceLabels[t][i];
    }

    // Returns the parent vertex of v on the shortest path from the i-th source to v.
    int getParentVertex(const int v, const int i = 0) {
        return parent.getVertex(v, i);
    }

    // Returns the parent edge of v on the shortest path from the i-th source to v.
    int getParentEdge(const int v, const int i = 0) {
        return parent.getEdge(v, i);
    }

    // Returns the vertices on the shortest path from the i-th source to t in reverse order.
    const std::vector<int32_t> &getReversePath(const int t, const int i = 0) {
        return parent.getReversePath(t, i);
    }

    // Returns the edges on the shortest path from the i-th source to t in reverse order.
    const std::vector<int32_t> &getReverseEdgePath(const int t, const int i = 0) {
        return parent.getReverseEdgePath(t, i);
    }

    int getNumEdgeRelaxations() const {
        return numEdgeRelaxations;
    }

    int getNumVerticesSettled() const {
        return numVerticesSettled;
    }

private:
    // Initializes the labels for computing multiple shortest-path trees simultaneously.
    void init(const std::array<int, K> &sources, const std::array<int, K> &offsets = {}) {
        numEdgeRelaxations = 0;
        nextVertices.build(sources);
        lastSources = sources;
        for (auto i = 0; i < K; ++i) {
            const auto s = sources[i];
            distanceLabels[s][i] = offsets[i];
            parent.setVertex(s, s, true);
            parent.setEdge(s, INVALID_EDGE, true);
        }
    }

    // Initializes the labels for computing a single shortest-path tree from multiple sources.
    template<typename IteratorT>
    void init(IteratorT firstSource, IteratorT lastSource) {
        numEdgeRelaxations = 0;
        numVerticesSettled = 0;
        std::array<int, K> sources;
        sources.fill(*firstSource);
        nextVertices.build(sources);
        lastSources = sources;
        for (auto iter = firstSource; iter != lastSource; ++iter) {
            const auto s = *iter;
            distanceLabels[s] = 0;
            parent.setVertex(s, s, true);
            parent.setEdge(s, INVALID_EDGE, true);
        }
    }

//    // Initializes the labels for computing a single shortest-path tree from multiple sources.
//    template<typename SourceIteratorT, typename OffsetIteratorT>
//    void init(SourceIteratorT firstSource, SourceIteratorT lastSource, OffsetIteratorT firstOffset,
//              OffsetIteratorT lastOffset) {
//        std::array<int, K> sources;
//        sources.fill(*firstSource);
//        nextVertices.build(sources);
//        lastSources = sources;
//        auto sourceIter = firstSource, offsetIter = firstOffset;
//        for (; sourceIter != lastSource && offsetIter != lastOffset; ++sourceIter, ++offsetIter) {
//            const auto s = *sourceIter;
//            distanceLabels[s] = std::min(distanceLabels[s][0], *offsetIter);
//            parent.setVertex(s, s, true);
//            parent.setEdge(s, INVALID_EDGE, true);
//        }
//        assert(sourceIter == lastSource && offsetIter == lastOffset);
//    }

    // Resets all distance labels to infinity.
    void resetDistanceLabels() {
        nextVertices.build(lastSources);
        for (auto v = nextVertex(); v != INVALID_VERTEX; v = nextVertex())
            distanceLabels[v] = INFTY;
    }

    // Relaxes the edges out of the next vertex and returns its ID.
    int settleNextVertex() {
        const auto v = nextVertex();
        auto &distToV = distanceLabels[v];
        if (!pruneSearch(v, distToV, distanceLabels)) {
            ++numVerticesSettled;
            FORALL_INCIDENT_EDGES(searchGraph, v, e) {
                ++numEdgeRelaxations;
                const auto w = searchGraph.edgeHead(e);
                auto &distToW = distanceLabels[w];
                const auto distViaV = distToV + searchGraph.traversalCost(e);
                const auto mask = distViaV < distToW;
                if (anySet(mask)) {
                    distToW.min(distViaV);
                    parent.setVertex(w, v, mask);
                    parent.setEdge(w, e, mask);
                }
            }
        }
        return v;
    }

    // Removes the next vertex from the tournament tree and returns it.
    int nextVertex() {
        const auto v = nextVertices.minKey();
        nextVertices.deleteMin(eliminationTree[v]);
        // If two or more of the k searches merged at v, block all but one of them.
        while (nextVertices.minKey() == v)
            nextVertices.deleteMin(INFTY);
        return v;
    }

    // Used by friend classes to update the pruning criterion for different runs of this search
    PruningCriterionT &getPruningCriterion() {
        return pruneSearch;
    }

    using DistanceLabelCont = SimpleDistanceLabelContainer<typename LabelSetT::DistanceLabel>;
    using ParentLabelCont = ParentLabelContainer<CH::SearchGraph, LabelSetT>;

    const CH::SearchGraph &searchGraph;          // The upward or downward graph we work on.
    const std::vector<int32_t> &eliminationTree; // elimTreeParent[v] is the parent of v in the tree.

    DistanceLabelCont distanceLabels;             // The distance labels of the vertices.
    ParentLabelCont parent;                       // The parent information for each vertex.
    TournamentTree<LabelSetT::logK> nextVertices; // The vertices settled next by the k searches.
    std::array<int, K> lastSources;               // The source vertices of the last k searches.
    PruningCriterionT pruneSearch;                // The criterion used to prune the search.


    int numEdgeRelaxations; // Number of edge relaxations in last run.
    int numVerticesSettled; // Number of vertices settled in last run.
};
