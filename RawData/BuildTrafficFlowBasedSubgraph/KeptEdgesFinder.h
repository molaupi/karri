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

#include <vector>
#include <cstdint>
#include <random>
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Tools/CommandLine/ProgressBar.h"

namespace traffic_flow_subnetwork {


// This class finds the edges that are kept in the vehicle network for a traffic flow based subnetwork.
    template<typename VehGraphT,
            typename PsgGraphT,
            typename WeightT = TravelTimeAttribute
    >
    class KeptEdgesFinder {

    private:

        using PickupSearch = Dijkstra<PsgGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>;
        using DropoffSearch = Dijkstra<PsgGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>;

    public:

        KeptEdgesFinder(const VehGraphT &vehGraph, const PsgGraphT &forwardPsgGraph, const PsgGraphT &reversePsgGraph,
                        const int walkRadius)
                : vehGraph(vehGraph), forwardPsgGraph(forwardPsgGraph), reversePsgGraph(reversePsgGraph),
                  walkRadius(walkRadius), edgeYetToBeCovered(forwardPsgGraph.numEdges()), numEdgesYetToBeCovered(),
                  pickupSearch(reversePsgGraph),
                  dropoffSearch(forwardPsgGraph) {}

        // Computes a set of edges that should be kept in the vehicle subnetwork to still cover all potential taxi
        // sharing requests with possible pickup and dropoff location.
        std::vector<int> findKeptEdges(const std::vector<int> &flows) {

            // Compute order of vehicle network edges by descending flow:
            std::vector<int> edgesInOrder(flows.size());
            std::iota(edgesInOrder.begin(), edgesInOrder.end(), 0);
            std::sort(edgesInOrder.begin(), edgesInOrder.end(),
                      [&flows](const auto &e1, const auto &e2) { return flows[e1] < flows[e2]; });

            // Compute maximum rank needed:
            const int maxRankForPickups = findMaxRankNeededForPickups(edgesInOrder);
            const int maxRankForDropoffs = findMaxRankNeededForDropoffs(edgesInOrder);
            const int maxRankNeeded = std::max(maxRankForPickups, maxRankForDropoffs);

            // Return all edges with rank smaller than (i.e. flow larger than) maximum rank:
            edgesInOrder.resize(maxRankNeeded + 1);
            return edgesInOrder;
        }


    private:

        int findMaxRankNeededForPickups(const std::vector<int> &descendingFlowOrder) {
            std::cout << "Finding covering pickup edges... " << std::flush;
            const int rank = findMaxNeededRank<karri::PICKUP>(descendingFlowOrder);
            std::cout << " done." << std::endl;
            return rank;
        }

        int findMaxRankNeededForDropoffs(const std::vector<int> &descendingFlowOrder) {
            std::cout << "Finding covering dropoff edges... " << std::flush;
            const int rank = findMaxNeededRank<karri::DROPOFF>(descendingFlowOrder);
            std::cout << " done." << std::endl;
            return rank;
        }

        // Given a descending (!) order of edges by flow, this returns a rank in that order s.t. all edges with larger
        // rank (i.e. smaller flow) do not need to be considered to cover the whole network with pickups/dropoffs.
        template<karri::PDLocType type>
        int findMaxNeededRank(const std::vector<int> &descendingFlowOrder) {

            markSharedEdgesToBeCovered();
            std::cout << "Num edges to cover: " << numEdgesYetToBeCovered << std::endl;
            progressBar.init(numEdgesYetToBeCovered);

            auto &search = type == karri::PICKUP ? pickupSearch : dropoffSearch;
            static constexpr SearchDirection dir =
                    type == karri::PICKUP ? SearchDirection::REVERSE : SearchDirection::FORWARD;

            search.distanceLabels.init();
            int rank = 0;
            for (; rank < descendingFlowOrder.size(); ++rank) {
                if (numEdgesYetToBeCovered == 0)
                    break;

                const auto &eInVeh = descendingFlowOrder[rank];
                const int eInForwPsgGraph = vehGraph.toPsgEdge(eInVeh);
                if (eInForwPsgGraph == CarEdgeToPsgEdgeAttribute::defaultValue())
                    continue;

                if (edgeYetToBeCovered[eInForwPsgGraph]) {
                    edgeYetToBeCovered[eInForwPsgGraph] = false;
                    --numEdgesYetToBeCovered;
                }

                const int s = type == karri::PICKUP ? forwardPsgGraph.edgeTail(eInForwPsgGraph)
                                                    : forwardPsgGraph.edgeHead(eInForwPsgGraph);
                const int offset = type == karri::PICKUP ? forwardPsgGraph.travelTime(eInForwPsgGraph) : 0;
                if (search.distanceLabels[s][0] <= offset)
                    continue;

                // Find all edges e s.t. if there is an origin/destination of a taxi sharing request at e,
                // eInForwPsgGraph can serve as a pickup/dropoff location for that request (i.e. eInForwPsgGraph is
                // within the walking radius from e/to e).
                // We do not clear the distance labels in between searches since every edge only needs to be covered by
                // one search. The search spaces become bounded voronoi cells with centers at every source used so far.
                search.queue.clear();
                search.distanceLabels[s][0] = offset;
                search.queue.insert(s, offset);
                int v, distToV;
                while (!search.queue.empty()) {
                    search.queue.min(v, distToV);
                    KASSERT(search.distanceLabels[v][0] == distToV);
                    if (distToV > walkRadius)
                        break;
                    markIncidentEdgesCovered<dir>(v, distToV);
                    search.settleNextVertex();
                }
            }
            if (numEdgesYetToBeCovered > 0)
                std::cout << " (Uncovered edges: " << numEdgesYetToBeCovered << ")";
            return rank - 1;
        }

        // Mark all edges that are accessible in both networks to be found since all of these are eligible origin
        // locations for taxi sharing requests.
        void markSharedEdgesToBeCovered() {
            KASSERT(numEdgesYetToBeCovered == edgeYetToBeCovered.cardinality());
            if (numEdgesYetToBeCovered > 0) {
                resetEdgeYetToBeCovered();
                numEdgesYetToBeCovered = 0;
            }

            FORALL_EDGES(vehGraph, e) {
                KASSERT(vehGraph.toPsgEdge(e) == CarEdgeToPsgEdgeAttribute::defaultValue() ||
                        forwardPsgGraph.toCarEdge(vehGraph.toPsgEdge(e)) == e);
                const auto eInForwPsgGraph = vehGraph.toPsgEdge(e);
                if (eInForwPsgGraph != CarEdgeToPsgEdgeAttribute::defaultValue()) {
                    edgeYetToBeCovered[eInForwPsgGraph] = true;
                    ++numEdgesYetToBeCovered;
                }
            }
        }

        template<SearchDirection dir>
        void markIncidentEdgesCovered(const int v, const int distToV) {
            KASSERT(numEdgesYetToBeCovered == edgeYetToBeCovered.cardinality());
            const auto &searchGraph = dir == SearchDirection::FORWARD ? forwardPsgGraph : reversePsgGraph;
            FORALL_INCIDENT_EDGES(searchGraph, v, e) {
                const int eInPsgForwGraph = searchGraph.edgeId(e);
                if (!edgeYetToBeCovered[eInPsgForwGraph])
                    continue;
                int distInForw = distToV;
                // Head of the edge in forward graph counts so in reverse search we do not need to consider
                // length of edge itself.
                if constexpr (dir == SearchDirection::FORWARD)
                    distInForw += searchGraph.template get<WeightT>(e);
                if (distInForw > walkRadius)
                    continue;
                edgeYetToBeCovered[eInPsgForwGraph] = false;
                --numEdgesYetToBeCovered;
                ++progressBar;
            }
        }

        // Sets all flags in edgeYetToBeCovered to false.
        void resetEdgeYetToBeCovered() {
            for (int i = edgeYetToBeCovered.firstSetBit(); i != -1; i = edgeYetToBeCovered.nextSetBit(i))
                edgeYetToBeCovered[i] = false;
            KASSERT(edgeYetToBeCovered.cardinality() == 0);
        }

        const VehGraphT &vehGraph;
        const PsgGraphT &forwardPsgGraph;
        const PsgGraphT &reversePsgGraph;
        const int walkRadius;

        BitVector edgeYetToBeCovered;
        int numEdgesYetToBeCovered;

        PickupSearch pickupSearch;
        DropoffSearch dropoffSearch;

        ProgressBar progressBar;

    };

} // end namespace