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
#include <fstream>
#include <utility>
#include <vector>

#include <routingkit/contraction_hierarchy.h>

#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Attributes/UnpackingInfoAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Utilities/Permutation.h"
#include "Tools/Constants.h"

// A weighted contraction hierarchy. The contraction order is determined online and bottom-up.
class CH {
public:
    // The type of the upward and downward graph.
    using Weight = TraversalCostAttribute;
    using SearchGraph = StaticGraph<VertexAttrs<>, EdgeAttrs<Weight, UnpackingInfoAttribute, TravelTimeAttribute>>;

    // Constructs an empty CH.
    CH() = default;

    // Constructs a CH from the specified upward and downward graph.
    template<typename SearchGraphT, typename PermT>
    CH(SearchGraphT &&upGraph, SearchGraphT &&downGraph, PermT &&order, PermT &&ranks) noexcept
            : upGraph(std::forward<SearchGraphT>(upGraph)),
              downGraph(std::forward<SearchGraphT>(downGraph)),
              order(std::forward<PermT>(order)),
              ranks(std::forward<PermT>(ranks)) {
        assert(this->upGraph.numVertices() == this->downGraph.numVertices());
        assert(this->upGraph.numVertices() == this->order.size());
        assert(this->ranks == this->order.getInversePermutation());
    }

    // Constructs a CH from the specified binary file.
    explicit CH(std::ifstream &in) {
        readFrom(in);
    }

    // Builds a weighted CH for the specified graph.
    template<typename WeightT, typename InputGraphT>
    void preprocess(const InputGraphT &inputGraph) {
        const auto numVertices = inputGraph.numVertices();
        const auto numEdges = inputGraph.numEdges();
        std::vector<unsigned int> tails(numEdges);
        std::vector<unsigned int> heads(numEdges);
        std::vector<unsigned int> weights(numEdges);
        std::vector<int> travelTimes(numEdges);
        FORALL_VALID_EDGES(inputGraph, u, e) {
                tails[e] = u;
                heads[e] = inputGraph.edgeHead(e);
                weights[e] = inputGraph.template get<WeightT>(e);
                travelTimes[e] = inputGraph.template get<TravelTimeAttribute>(e);
            }

        const auto ch = RoutingKit::ContractionHierarchy::build(numVertices, tails, heads, weights);

        // TraversalCost and TravelTime may be the same but TravelTime is always needed to check for time constraints.
        const auto travelTimesOnShortCuts = RoutingKit::ContractionHierarchyExtraWeight<int>(
                ch, travelTimes, [](const int tt1, const int tt2) { return tt1 + tt2; });

        upGraph.clear();
        downGraph.clear();
        upGraph.reserve(numVertices, ch.forward.head.size());
        downGraph.reserve(numVertices, ch.backward.head.size());
        for (auto u = 0; u < numVertices; ++u) {
            upGraph.appendVertex();
            downGraph.appendVertex();
            for (auto e = ch.forward.first_out[u]; e < ch.forward.first_out[u + 1]; ++e) {
                upGraph.appendEdge(ch.forward.head[e]);
                upGraph.traversalCost(e) = ch.forward.weight[e];
                upGraph.travelTime(e) = travelTimesOnShortCuts.forward_weight[e];
                upGraph.unpackingInfo(e).first = ch.forward.shortcut_first_arc[e];
                upGraph.unpackingInfo(e).second = ch.forward.shortcut_second_arc[e];
                if (ch.forward.is_shortcut_an_original_arc.is_set(e))
                    upGraph.unpackingInfo(e).second = INVALID_EDGE;
            }
            for (auto e = ch.backward.first_out[u]; e < ch.backward.first_out[u + 1]; ++e) {
                downGraph.appendEdge(ch.backward.head[e]);
                downGraph.traversalCost(e) = ch.backward.weight[e];
                downGraph.travelTime(e) = travelTimesOnShortCuts.backward_weight[e];
                downGraph.unpackingInfo(e).first = ch.backward.shortcut_first_arc[e];
                downGraph.unpackingInfo(e).second = ch.backward.shortcut_second_arc[e];
                if (ch.backward.is_shortcut_an_original_arc.is_set(e))
                    downGraph.unpackingInfo(e).second = INVALID_EDGE;
            }
        }

        order.assign(ch.order.begin(), ch.order.end());
        ranks.assign(ch.rank.begin(), ch.rank.end());
    }

    // Returns the upward graph.
    const SearchGraph &upwardGraph() const noexcept {
        return upGraph;
    }

    // Returns the downward graph.
    const SearchGraph &downwardGraph() const noexcept {
        return downGraph;
    }

    // Returns the i-th vertex in the contraction order.
    int contractionOrder(const int i) const {
        return order[i];
    }

    // Returns the location of vertex v in the contraction order.
    int rank(const int v) const {
        return ranks[v];
    }

    // Reads the CH from the specified binary file.
    void readFrom(std::ifstream &in) {
        upGraph.readFrom(in);
        downGraph.readFrom(in);
        order.readFrom(in);
        ranks.readFrom(in);
    }

    // Writes the CH to the specified binary file.
    void writeTo(std::ofstream &out) const {
        upGraph.writeTo(out);
        downGraph.writeTo(out);
        order.writeTo(out);
        ranks.writeTo(out);
    }

private:
    SearchGraph upGraph;   // The upward graph.
    SearchGraph downGraph; // The downward graph.
    Permutation order;     // order[i] = v indicates that v was the i-th vertex contracted.
    Permutation ranks;     // ranks[v] = i indicates that v was the i-th vertex contracted.
};
