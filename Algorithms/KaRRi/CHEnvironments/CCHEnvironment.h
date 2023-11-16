/// ******************************************************************************
/// MIT License
///
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

#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHQuery.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/Dijkstra/DagShortestPaths.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/CCH/EliminationTreeQuery.h"
#include "Algorithms/CCH/CCHMetric.h"

#include <routingkit/nested_dissection.h>

namespace karri {

    // Maintains a CH for a fixed edge weight and input graph and offers factory methods for certain kinds of queries
    // on the CH.
    template<typename InputGraphT, typename WeightAttributeT>
    class CCHEnvironment {

        using DefaultLabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;

    public:

        template<typename LabelSetT = DefaultLabelSet>
        using FullCHQuery = EliminationTreeQuery<LabelSetT>;

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = elimintree::PruningCriterion, typename LabelSetT = DefaultLabelSet>
        using UpwardSearch = UpwardEliminationTreeSearch<LabelSetT, dij::CompoundCriterion<StoppingCriterionT, PruningCriterionT>>;

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename LabelSetT = DefaultLabelSet>
        using TopologicalUpwardSearch = UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>;


        // Constructs environment by building new CCH for given input inputGraph and customizing it
        explicit CCHEnvironment(const InputGraphT& graph)
                : inputGraph(graph), currentMetric(cch, &graph.template get<WeightAttributeT>(0)) {
            preprocess();
            customize();
        }

        CCHEnvironment(const InputGraphT& graph, const SeparatorDecomposition& decomp)
                : inputGraph(graph), currentMetric(cch, &graph.template get<WeightAttributeT>(0)) {
            cch.preprocess(graph, decomp);
            customize();
        }

        CCHEnvironment(CCHEnvironment&&) = delete;

        const CH& getCH() const noexcept {
            return minimumWeightedCH;
        }


        template<typename LabelSetT = DefaultLabelSet>
        FullCHQuery<LabelSetT> getFullCHQuery() const {
            return FullCHQuery<LabelSetT>(minimumWeightedCH, cch.getEliminationTree());
        }

        template<typename PruningCriterionT = dij::NoCriterion,
                typename StoppingCriterionT = elimintree::PruningCriterion,
                typename LabelSetT = DefaultLabelSet>
        UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>
        getForwardSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}) const {
            return UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    minimumWeightedCH.upwardGraph(), cch.getEliminationTree(), {stop, prune});
        }

        template<typename PruningCriterionT = dij::NoCriterion,
                typename StoppingCriterionT = elimintree::PruningCriterion,
                typename LabelSetT = DefaultLabelSet>
        UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>
        getReverseSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}) const {
            return UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    minimumWeightedCH.downwardGraph(), cch.getEliminationTree(), {stop, prune});
        }

        template<typename PruningCriterionT = dij::NoCriterion,
                typename StoppingCriterionT = dij::NoCriterion,
                typename LabelSetT = DefaultLabelSet>
        TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>
        getForwardTopologicalSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}) const {
            return getForwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(prune, stop);
        }

        template<typename PruningCriterionT = dij::NoCriterion,
                typename StoppingCriterionT = dij::NoCriterion,
                typename LabelSetT = DefaultLabelSet>
        TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>
        getReverseTopologicalSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}) const {
            return getReverseSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(prune, stop);
        }


        void preprocess() {
            // Convert the input graph to RoutingKit's graph representation.
            std::vector<float> lats(inputGraph.numVertices());
            std::vector<float> lngs(inputGraph.numVertices());
            std::vector<unsigned int> tails(inputGraph.numEdges());
            std::vector<unsigned int> heads(inputGraph.numEdges());
            FORALL_VERTICES(inputGraph, v) {
                lats[v] = inputGraph.latLng(v).latInDeg();
                lngs[v] = inputGraph.latLng(v).lngInDeg();
                FORALL_INCIDENT_EDGES(inputGraph, v, e) {
                    tails[e] = v;
                    heads[e] = inputGraph.edgeHead(e);
                }
            }

            // Compute a separator decomposition for the input graph.
            const auto graph = RoutingKit::make_graph_fragment(inputGraph.numVertices(), tails, heads);
            auto computeSep = [&](const RoutingKit::GraphFragment& fragment) {
                const auto cut = inertial_flow(fragment, 30, lats, lngs);
                return derive_separator_from_cut(fragment, cut.is_node_on_side);
            };
            const auto decomp = compute_separator_decomposition(graph, computeSep);

            // Convert the separator decomposition to our representation.
            SeparatorDecomposition sepDecomp;
            for (const auto& n : decomp.tree) {
                SeparatorDecomposition::Node node{};
                node.leftChild = n.left_child;
                node.rightSibling = n.right_sibling;
                node.firstSeparatorVertex = n.first_separator_vertex;
                node.lastSeparatorVertex = n.last_separator_vertex;
                sepDecomp.tree.push_back(node);
            }
            sepDecomp.order.assign(decomp.order.begin(), decomp.order.end());

            // Build the CCH.
            cch.preprocess(inputGraph, sepDecomp);
        }

        void customize() {
            minimumWeightedCH = currentMetric.buildMinimumWeightedCH();
        }

    private:
        const InputGraphT& inputGraph;
        CCH cch;
        CCHMetric currentMetric;
        CH minimumWeightedCH;

    };
}