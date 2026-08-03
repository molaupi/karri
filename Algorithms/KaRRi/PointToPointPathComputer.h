#pragma once

#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "DataStructures/Graph/Graph.h"

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

        // Computes path (sequence of edges) from fromEdge to toEdge. Returns total travel time distance.
        int computePathEdges(const int fromEdge, const int toEdge, std::vector<int> &outPath) {
            chQuery.run(ch.rank(inputGraph.edgeHead(fromEdge)), ch.rank(inputGraph.edgeTail(toEdge)));
            outPath.clear();
            unpacker.unpackUpDownPath(chQuery.getUpEdgePath(), chQuery.getDownEdgePath(), outPath);
            return chQuery.getDistance() + inputGraph.travelTime(toEdge);
        }

    private:
        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CHPathUnpacker unpacker;
    };

}
