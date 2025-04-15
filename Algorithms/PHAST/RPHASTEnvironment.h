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

//
//    // Runs RPHAST target selection phase for given sources in upward graph.
//    // Given sources should be ranks in the CH.
//    // If using PruningCriterion, offsets to initialize the distance at each source may be given.
//    void runSourceSelection(const std::vector<int> &sources, const std::vector<int>& offsets = {}) {
//        findVerticesInSourcesSubgraph(sources, offsets);
//        sourcesSubgraph = constructOrderedSubgraph(upwardGraph, verticesInSubgraph, sourcesFullToSubMapping);
//        sourcesSubToFullMapping.resize(verticesInSubgraph.size());
//        for (const auto& v : verticesInSubgraph)
//            sourcesSubToFullMapping[sourcesFullToSubMapping[v]] = v;
//    }
//
//    int getDistanceFromClosestSource(const int v) const requires WithPruning {
//        KASSERT(v >= 0);
//        KASSERT(v < upwardGraph.numVertices());
//        return sourcesSelectionSearch.getDistance(v);
//    }
//
//    // Runs RPHAST target selection phase for given targets in downward graph.
//    // Given targets should be ranks in the CH.
//    // If using PruningCriterion, offsets to initialize the distance at each target may be given.
//    void runTargetSelection(const std::vector<int> &targets, const std::vector<int>& offsets = {}) {
//        findVerticesInTargetsSubgraph(targets, offsets);
//        targetsSubgraph = constructOrderedSubgraph(downwardGraph, verticesInSubgraph, targetsFullToSubMapping);
//        targetsSubToFullMapping.resize(verticesInSubgraph.size());
//        for (const auto& v : verticesInSubgraph)
//            targetsSubToFullMapping[targetsFullToSubMapping[v]] = v;
//    }
//
//    int getDistanceToClosestTarget(const int v) const requires WithPruning {
//        KASSERT(v >= 0);
//        KASSERT(v < upwardGraph.numVertices());
//        return targetsSelectionSearch.getDistance(v);
//    }

    template<typename PruningCriterionT = dij::NoCriterion>
    RPHASTSelectionPhase<PruningCriterionT> getTargetsSelectionPhase(PruningCriterionT pruningCriterion = {}) const {
        return RPHASTSelectionPhase<PruningCriterionT>(downwardGraph, pruningCriterion);
    }

    template<typename PruningCriterionT = dij::NoCriterion>
    RPHASTSelectionPhase<PruningCriterionT> getSourcesSelectionPhase(PruningCriterionT pruningCriterion = {}) const {
        return RPHASTSelectionPhase<PruningCriterionT>(upwardGraph, pruningCriterion);
    }


    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    using Query = PHASTQuery<SearchGraph, typename CH::Weight, LabelSetT>;

    // Returns a forward RPHAST query that uses the result of the target selection phase completed by the last
    // call to runTargetSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    Query<LabelSetT> getForwardRPHASTQuery() const {
        return Query<LabelSetT>(upwardGraph);
    }

    // Returns a reverse RPHAST query that uses the result of the target selection phase completed by the last
    // call to runSourceSelection().
    template<typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    Query<LabelSetT> getReverseRPHASTQuery() const {
        return Query<LabelSetT>(downwardGraph);
    }




private:

    // Full CH search graphs with vertices ordered by increasing rank. Used for query side ("one" side of one-to-many) of RPHAST queries.
    const SearchGraph& upwardGraph;
    const SearchGraph& downwardGraph;

//    // Subgraphs of the upward and downward search graphs with vertices ordered by decreasing rank.
//    // Used for the target side ("many" side of one-to-many) of RPHAST queries.
//    // Full-to-sub mappings map each vertex ID in the full graph to the according ID in the subgraph if present.
//    // If vertex v is not present in the subgraph, its mapping is subGraph.numVertices() to mark it as invalid.
//    // Sub-to-full mappings map each vertex ID in the subgraph to the according ID in the full graph.
//    // Subgraphs and mappings are constructed for a specific set of sources/targets passed to
//    // runSourceSelection/runTargetSelection.
//    SearchGraph sourcesSubgraph;
//    std::vector<int> sourcesFullToSubMapping;
//    std::vector<int> sourcesSubToFullMapping;
//    SearchGraph targetsSubgraph;
//    std::vector<int> targetsFullToSubMapping;
//    std::vector<int> targetsSubToFullMapping;

};