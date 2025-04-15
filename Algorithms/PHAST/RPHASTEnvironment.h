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
#include "RPHASTSelectionPhase.h"

class RPHASTEnvironment {

public:
    using SearchGraph = typename CH::SearchGraph;

    explicit RPHASTEnvironment(const CH &ch) : upwardGraph(ch.upwardGraph()), downwardGraph(ch.downwardGraph()) {}

    template<typename PruningCriterionT = dij::NoCriterion>
    RPHASTSelectionPhase<PruningCriterionT> getTargetsSelectionPhase(PruningCriterionT pruningCriterion = {}) const {
        return RPHASTSelectionPhase<PruningCriterionT>(downwardGraph, pruningCriterion);
    }

    template<typename PruningCriterionT = dij::NoCriterion>
    RPHASTSelectionPhase<PruningCriterionT> getSourcesSelectionPhase(PruningCriterionT pruningCriterion = {}) const {
        return RPHASTSelectionPhase<PruningCriterionT>(upwardGraph, pruningCriterion);
    }


    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>,
            typename PruningCriterionT = dij::NoCriterion>
    using Query = PHASTQuery<SearchGraph, typename CH::Weight, LabelSetT, PruningCriterionT>;

    // Returns a forward RPHAST query that uses the result of the target selection phase completed by the last
    // call to runTargetSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>,
            typename PruningCriterionT = dij::NoCriterion>
    Query<LabelSetT, PruningCriterionT> getForwardRPHASTQuery(PruningCriterionT prune = {}) const {
        return Query<LabelSetT, PruningCriterionT>(upwardGraph, prune);
    }

    // Returns a reverse RPHAST query that uses the result of the target selection phase completed by the last
    // call to runSourceSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>,
            typename PruningCriterionT = dij::NoCriterion>
    Query<LabelSetT, PruningCriterionT> getReverseRPHASTQuery(PruningCriterionT prune = {}) const {
        return Query<LabelSetT, PruningCriterionT>(downwardGraph, prune);
    }


private:

    // Full CH search graphs with vertices ordered by increasing rank. Used for query side ("one" side of one-to-many) of RPHAST queries.
    const SearchGraph& upwardGraph;
    const SearchGraph& downwardGraph;

};