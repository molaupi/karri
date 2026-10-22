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

#include <vector>
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"

namespace karri {

    // Computes a shortest path between two edge-locations (represented as edge ids) using CH queries and unpacks
    // the resulting up/down edge path into a vector of edges. Returns the travel time distance from the source edge
    // to the target edge's head, i.e., CH-query distance plus travel time of the target edge (matching VehicleLocator
    // conventions).
    template<typename InputGraphT, typename CHEnvT>
    class PointToPointPathComputer {
    public:
        PointToPointPathComputer(const InputGraphT &inputGraph, const CHEnvT &chEnv)
                : inputGraph(inputGraph), ch(chEnv.getCH()), chQuery(chEnv.template getFullCHQuery<>()), unpacker(ch) {}

        // Computes path (sequence of edges) from fromEdge to toEdge (including toEdge but not fromEdge).
        // Returns total travel time distance.
        int computePathEdges(const int fromEdge, const int toEdge, std::vector<int> &outPath) {
            chQuery.run(ch.rank(inputGraph.edgeHead(fromEdge)), ch.rank(inputGraph.edgeTail(toEdge)));
            outPath.clear();
            unpacker.unpackUpDownPath(chQuery.getUpEdgePath(), chQuery.getDownEdgePath(), outPath);
            outPath.push_back(toEdge);
            return chQuery.getDistance() + inputGraph.travelTime(toEdge);
        }

    private:
        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CHPathUnpacker unpacker;
    };

}
