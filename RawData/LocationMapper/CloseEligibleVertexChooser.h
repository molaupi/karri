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

#include "DataStructures/Labels/ParentInfo.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"


// Given a vertex in a graph, this facility chooses another vertex that is different from the original one,
// eligible according to a given eligibility criterion and as close to the given vertex as possible.
template<typename GraphT,
        typename WeightT,
        typename IsEligibleT>
class CloseEligibleVertexChooser {

    struct StopWhenVertexWithEligibleIncEdgeFound {
        StopWhenVertexWithEligibleIncEdgeFound(CloseEligibleVertexChooser &chooser) : chooser(chooser) {}

        template<typename DistLabelT, typename DistLabelContainerT>
        bool operator()(const int v, DistLabelT &, const DistLabelContainerT &) {
            if (v != chooser.vertex && chooser.isEligible(v)) {
                chooser.vertex = v; // Sets vertexToRepair to the vertex found.
                return true;
            }
            return false;
        }

        CloseEligibleVertexChooser &chooser;
    };

public:

    CloseEligibleVertexChooser(const GraphT &graph, const IsEligibleT &isEligible)
            : graph(graph),
              isEligible(isEligible),
              vertex(INVALID_VERTEX),
              search(graph, {*this}) {}

    int findOtherVertex(const int v) {
        vertex = v;
        search.run(v);
        return vertex;
    }

private:

    const GraphT &graph;
    const IsEligibleT &isEligible;

    using Search = Dijkstra<GraphT, WeightT, karri::BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>,
            StopWhenVertexWithEligibleIncEdgeFound>;
    int vertex;
    Search search;


};