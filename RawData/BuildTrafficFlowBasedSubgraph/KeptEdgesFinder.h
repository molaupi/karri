/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
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
                      [&flows](const auto &e1, const auto &e2) { return flows[e1] > flows[e2]; });

            // Gather flow statistics
            const int minFlow = flows[edgesInOrder.back()];
            const int numEqMin = std::count(flows.begin(), flows.end(), minFlow);
            const int maxFlow = flows[edgesInOrder.front()];
            const int medianFlow = flows[edgesInOrder[edgesInOrder.size() / 2]];
            std::cout << "Flows: \n";
            std::cout << "\tMin: " << minFlow << " (Number of elements that = min: " << numEqMin << ")\n";
            std::cout << "\tMed: " << medianFlow << "\n";
            std::cout << "\tMax: " << maxFlow << "\n";

            // Compute maximum rank needed:
            int maxNeededRank = -1;
            findMaxRankNeededForPickups(edgesInOrder, flows, maxNeededRank);
            findMaxRankNeededForDropoffs(edgesInOrder, flows, maxNeededRank);

            // Return all edges with rank smaller than (i.e. flow larger than) maximum rank:
            edgesInOrder.resize(maxNeededRank + 1);
            return edgesInOrder;
        }


    private:

        void findMaxRankNeededForPickups(std::vector<int> &descendingFlowOrder, const std::vector<int> &flows,
                                         int &maxNeededRank) {
            std::cout << "Finding covering pickup edges... " << std::flush;
            findMaxNeededRank<karri::PICKUP>(descendingFlowOrder, flows, maxNeededRank);
            std::cout << " done." << std::endl;
        }

        void findMaxRankNeededForDropoffs(std::vector<int> &descendingFlowOrder, const std::vector<int> &flows,
                                          int &maxNeededRank) {
            std::cout << "Finding covering dropoff edges... " << std::flush;
            findMaxNeededRank<karri::DROPOFF>(descendingFlowOrder, flows, maxNeededRank);
            std::cout << " done." << std::endl;
        }

        // Given a descending (!) order of edges by flow, this returns a rank in that order s.t. all edges with larger
        // rank (i.e. smaller flow) do not need to be considered to cover the whole network with pickups/dropoffs.
        //
        // May change the flow order in the tail range of zero flow edges to pick a better cover with a smaller number
        // of edges.
        template<karri::PDLocType type>
        void
        findMaxNeededRank(std::vector<int> &descendingFlowOrder, const std::vector<int> &flows, int &maxNeededRank) {

            if (maxNeededRank == descendingFlowOrder.size())
                return;

            markSharedEdgesToBeCovered();
            std::cout << "Num edges to cover: " << numEdgesYetToBeCovered << std::endl;
            progressBar.init(numEdgesYetToBeCovered);

            auto &search = type == karri::PICKUP ? pickupSearch : dropoffSearch;
            search.distanceLabels.init();

            // For edges up to known lower bound maxNeededRank, we simply add the edges and all surrounding edges.
            // If this already covers all locations, maxNeededRank is not increased and we are done.
            for (int rank = 0; rank <= maxNeededRank; ++rank) {
                const auto &eInVeh = descendingFlowOrder[rank];
                coverEdgesSurroundingEdge<type>(eInVeh);
                if (numEdgesYetToBeCovered == 0)
                    return;
            }

            // If we have not yet covered all locations, we need to find more edges to cover the remaining locations,
            // increasing the maximum rank needed.
            ++maxNeededRank;
            bool reachedZeroFlow = false;
            for (; maxNeededRank < descendingFlowOrder.size(); ++maxNeededRank) {
                const auto &eInVeh = descendingFlowOrder[maxNeededRank];

                // If we have already exceeded the known minimum needed maximum rank, and we have reached edges with
                // flow 0, we can reorder the remaining zero flow edges to potentially find a better cover.
                if (flows[eInVeh] == 0 && !reachedZeroFlow) {
                    reachedZeroFlow = true;
                    reorderZeroFlowEdges<type>(descendingFlowOrder, maxNeededRank);
                }

                coverEdgesSurroundingEdge<type>(eInVeh);
                if (numEdgesYetToBeCovered == 0)
                    return;
            }
            std::cout << " (Uncovered edges: " << numEdgesYetToBeCovered << ")";
            LIGHT_KASSERT(false);
        }

        template<karri::PDLocType type>
        void coverEdgesSurroundingEdge(const int eInVeh) {

            const int eInForwPsgGraph = vehGraph.mapToEdgeInPsg(eInVeh);
            if (eInForwPsgGraph == MapToEdgeInPsgAttribute::defaultValue())
                return;

            if (edgeYetToBeCovered[eInForwPsgGraph]) {
                edgeYetToBeCovered[eInForwPsgGraph] = false;
                --numEdgesYetToBeCovered;
            }

            auto &search = type == karri::PICKUP ? pickupSearch : dropoffSearch;
            static constexpr SearchDirection dir =
                    type == karri::PICKUP ? SearchDirection::REVERSE : SearchDirection::FORWARD;

            const int s = type == karri::PICKUP ? forwardPsgGraph.edgeTail(eInForwPsgGraph)
                                                : forwardPsgGraph.edgeHead(eInForwPsgGraph);
            const int offset = type == karri::PICKUP ? forwardPsgGraph.travelTime(eInForwPsgGraph) : 0;
            if (search.distanceLabels[s][0] <= offset)
                return;

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

        // If the construction of a cover by picking edges in descending order of flows reaches edges with flow 0,
        // we use heuristically reorder all zero flow edges, making sure to favor those edges as sources who have
        // not been covered themselves.
        template<karri::PDLocType type>
        void reorderZeroFlowEdges(std::vector<int> &descendingFlowOrder, const int startRank) {

            // Remove zero flow edges that are not passenger-accessible as they cannot serve as sources for searches:
            const auto &isNotPsgAcc = [&](const int &e) -> bool {
                return vehGraph.mapToEdgeInPsg(e) == MapToEdgeInPsgAttribute::defaultValue();
            };
            descendingFlowOrder.erase(
                    std::remove_if(descendingFlowOrder.begin() + startRank, descendingFlowOrder.end(), isNotPsgAcc),
                    descendingFlowOrder.end());

            // Sort zero flow edges by how far away they are from previous sources in descending order.
            const auto cmpCurSearchDist = [&](const int &e1, const int &e2) {
                const int e1InForwPsg = vehGraph.mapToEdgeInPsg(e1);
                const int e2InForwPsg = vehGraph.mapToEdgeInPsg(e2);
                LIGHT_KASSERT(!isNotPsgAcc(e1) && !isNotPsgAcc(e2));

                if constexpr (type == karri::PICKUP) {
                    const int dist1 = pickupSearch.distanceLabels[forwardPsgGraph.edgeHead(e1InForwPsg)][0];
                    const int dist2 = pickupSearch.distanceLabels[forwardPsgGraph.edgeHead(e2InForwPsg)][0];
                    return dist1 > dist2;
                }
                // type == DROPOFF:
                const int dist1 = dropoffSearch.distanceLabels[forwardPsgGraph.edgeTail(e1InForwPsg)][0] +
                                  forwardPsgGraph.template get<WeightT>(e1InForwPsg);
                const int dist2 = dropoffSearch.distanceLabels[forwardPsgGraph.edgeTail(e2InForwPsg)][0] +
                                  forwardPsgGraph.template get<WeightT>(e2InForwPsg);
                return dist1 > dist2;
            };
            std::sort(descendingFlowOrder.begin() + startRank, descendingFlowOrder.end(), cmpCurSearchDist);
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
                KASSERT(vehGraph.mapToEdgeInPsg(e) == MapToEdgeInPsgAttribute::defaultValue() ||
                        forwardPsgGraph.mapToEdgeInFullVeh(vehGraph.mapToEdgeInPsg(e)) == e);
                const auto eInForwPsgGraph = vehGraph.mapToEdgeInPsg(e);
                if (eInForwPsgGraph != MapToEdgeInPsgAttribute::defaultValue()) {
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