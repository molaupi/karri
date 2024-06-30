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

namespace karri {

    // Maintains a CH for a fixed edge weight and input graph and offers factory methods for certain kinds of queries
    // on the CH.
    template<typename InputGraphT, typename WeightAttributeT>
    class CHEnvironment {

        using DefaultLabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;

        template<typename LabelSetT>
        using StallOnDemandCriterion = typename CHQuery<LabelSetT>::PruningCriterion;

    public:

        template<typename LabelSetT = DefaultLabelSet>
        using FullCHQuery = CHQuery<LabelSetT>;

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        using UpwardSearch = Dijkstra<typename CH::SearchGraph, CH::Weight, LabelSetT, StoppingCriterionT, dij::CompoundCriterion<StallOnDemandCriterion<LabelSetT>, PruningCriterionT>, RelaxationCallbackT>;

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        using TopologicalUpwardSearch = DagShortestPaths<typename CH::SearchGraph, CH::Weight, LabelSetT, dij::CompoundCriterion<StoppingCriterionT, PruningCriterionT>, RelaxationCallbackT>;


        // Constructs environment by building new CH for given input inputGraph
        explicit CHEnvironment(const InputGraphT &inputGraph) : ch() {
            ch.preprocess<WeightAttributeT, InputGraphT>(inputGraph);
        }

        // Constructs environment using given existing CH
        explicit CHEnvironment(CH &&ch) : ch(std::move(ch)) {}

        const CH &getCH() const {
            return ch;
        }

        template<typename LabelSetT = DefaultLabelSet>
        FullCHQuery<LabelSetT> getFullCHQuery() const {
            return FullCHQuery<LabelSetT>(ch);
        }

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        UpwardSearch<PruningCriterionT, StoppingCriterionT, RelaxationCallbackT, LabelSetT>
        getForwardSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}, RelaxationCallbackT relax = {}) const {
            return UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    ch.upwardGraph(), stop, {StallOnDemandCriterion<LabelSetT>(ch.downwardGraph()), prune}, relax);
        }

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        UpwardSearch<PruningCriterionT, StoppingCriterionT, RelaxationCallbackT, LabelSetT>
        getReverseSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}, RelaxationCallbackT relax = {}) const {
            return UpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    ch.downwardGraph(), stop, {StallOnDemandCriterion<LabelSetT>(ch.upwardGraph()), prune}, relax);
        }

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, RelaxationCallbackT, LabelSetT>
        getForwardTopologicalSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}, RelaxationCallbackT relax = {}) const {
            return TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    ch.upwardGraph(), {stop, prune}, relax);
        }

        template<typename PruningCriterionT = dij::NoCriterion, typename StoppingCriterionT = dij::NoCriterion, typename RelaxationCallbackT = dij::NoRelaxationCallback, typename LabelSetT = DefaultLabelSet>
        TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, RelaxationCallbackT, LabelSetT>
        getReverseTopologicalSearch(PruningCriterionT prune = {}, StoppingCriterionT stop = {}, RelaxationCallbackT relax = {}) const {
            return TopologicalUpwardSearch<PruningCriterionT, StoppingCriterionT, LabelSetT>(
                    ch.downwardGraph(), {stop, prune}, relax);
        }

    private:

        CH ch;

    };
}