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
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Tools/CommandLine/ProgressBar.h"


namespace traffic_flow_subnetwork {

// This class finds high ranked vertices in a vehicle network that cover all locations of an associated passenger
// network for pickups, dropoffs.
    template<typename VehGraphT,
            typename PsgGraphT,
            typename WeightT = TravelTimeAttribute
    >
    class KeptHighRankVerticesFinder {


    private:

        using PickupSearch = Dijkstra<PsgGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>;
        using DropoffSearch = Dijkstra<PsgGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>;

    public:

        KeptHighRankVerticesFinder(const VehGraphT &vehGraph, const PsgGraphT &forwardPsgGraph,
                                   const PsgGraphT &reversePsgGraph,
                                   const int walkRadius)
                : vehGraph(vehGraph), revVehGraph(vehGraph.getReverseGraph()),
                  forwardPsgGraph(forwardPsgGraph), reversePsgGraph(reversePsgGraph),
                  walkRadius(walkRadius), edgeYetToBeCovered(forwardPsgGraph.numEdges()), numEdgesYetToBeCovered(),
                  pickupSearch(reversePsgGraph),
                  dropoffSearch(forwardPsgGraph) {}

        // Computes a set of vertices that should be kept in the vehicle subnetwork to still cover all potential taxi
        // sharing requests with possible pickup and dropoff location.
        BitVector findKeptVertices() {

            // Compute vehicle CH:
            std::cout << "Building CH... " << std::flush;
            CH ch;
            ch.preprocess<WeightT, VehGraphT>(vehGraph);
            std::cout << "done.\n";

            // Compute maximum rank needed:
            int minNeededCHRank = vehGraph.numVertices() - 1;
            findMinCHRankNeededForPickups(ch, minNeededCHRank);
            findMinCHRankNeededForDropoffs(ch, minNeededCHRank);

            BitVector vertexNeeded(vehGraph.numVertices());
            for (int rank = 0; rank <= minNeededCHRank; ++rank)
                vertexNeeded[ch.contractionOrder(rank)] = true;

            return vertexNeeded;
        }


    private:

        void findMinCHRankNeededForPickups(const CH &ch, int &minNeededCHRank) {
            std::cout << "Finding covering pickup edges... " << std::flush;
            findMinNeededCHRank<karri::PICKUP>(ch, minNeededCHRank);
            std::cout << " done." << std::endl;
        }

        void findMinCHRankNeededForDropoffs(const CH &ch, int &minNeededCHRank) {
            std::cout << "Finding covering dropoff edges... " << std::flush;
            findMinNeededCHRank<karri::DROPOFF>(ch, minNeededCHRank);
            std::cout << " done." << std::endl;
        }

        template<karri::PDLocType type>
        void
        findMinNeededCHRank(const CH &ch, int &minNeededCHRank) {

            if (minNeededCHRank == 0)
                return;

            markSharedEdgesToBeCovered();
            std::cout << "Num edges to cover: " << numEdgesYetToBeCovered << std::endl;
            progressBar.init(numEdgesYetToBeCovered);

            auto &search = type == karri::PICKUP ? pickupSearch : dropoffSearch;
            search.distanceLabels.init();

            // For vertices up to known upper bound minNeededCHRank, we simply add all surrounding edges.
            // If this already covers all locations, minNeededCHRank is not decreasing and we are done.
            for (int rank = vehGraph.numVertices() - 1; rank >= minNeededCHRank; --rank) {
                const int v = ch.contractionOrder(rank);
                coverEdgesSurroundingEdge<type>(v);
                if (numEdgesYetToBeCovered == 0)
                    return;
            }

            // If we have not yet covered all locations, we need to find more edges to cover the remaining locations,
            // increasing the maximum rank needed.
            --minNeededCHRank;
            for (; minNeededCHRank >= 0; --minNeededCHRank) {
                const int v = ch.contractionOrder(minNeededCHRank);
                coverEdgesSurroundingEdge<type>(v);
                if (numEdgesYetToBeCovered == 0)
                    return;
            }
            std::cout << " (Uncovered edges: " << numEdgesYetToBeCovered << ")";
            LIGHT_KASSERT(false);
        }

        template<karri::PDLocType type>
        void coverEdgesSurroundingEdge(const int u) {

            // Use shortest passenger-accessible incident edge of u as potential pickup/dropoff center of search
            // (if any exist).
            int eInForwPsgGraph = INVALID_EDGE;
            int lengthOfEInForwPsgGraph = INFTY;
            if (type == karri::PICKUP) {
                FORALL_INCIDENT_EDGES(revVehGraph, u, e) {
                    const auto &eInPsg = revVehGraph.mapToEdgeInPsg(e);
                    if (eInPsg != MapToEdgeInPsgAttribute::defaultValue()) {
                        if (forwardPsgGraph.template get<WeightT>(eInPsg) < lengthOfEInForwPsgGraph) {
                            lengthOfEInForwPsgGraph = forwardPsgGraph.template get<WeightT>(eInPsg);
                            eInForwPsgGraph = eInPsg;
                        }
                    }
                }
            } else {
                FORALL_INCIDENT_EDGES(vehGraph, u, e) {
                    const auto &eInPsg = vehGraph.mapToEdgeInPsg(e);
                    if (eInPsg != MapToEdgeInPsgAttribute::defaultValue()) {
                        if (forwardPsgGraph.template get<WeightT>(eInPsg) < lengthOfEInForwPsgGraph) {
                            lengthOfEInForwPsgGraph = forwardPsgGraph.template get<WeightT>(eInPsg);
                            eInForwPsgGraph = eInPsg;
                        }
                    }
                }
            }

            if (eInForwPsgGraph == INVALID_EDGE)
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
            const int offset = type == karri::PICKUP ? lengthOfEInForwPsgGraph : 0;
            if (search.distanceLabels[s][0] <= offset)
                return;

            // Find all edges e s.t. if there is an origin/destination of a taxi sharing request at e,
            // s can serve as a pickup/dropoff location for that request (i.e. s is within the walking radius from e/to e).
            // We do not clear the distance labels in between searches since every edge only needs to be covered by
            // one search. The search spaces become bounded voronoi cells with centers at every source used so far.
            search.queue.clear();
            search.distanceLabels[s][0] = 0;
            search.queue.insert(s, 0);
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
        const VehGraphT revVehGraph;
        const PsgGraphT &forwardPsgGraph;
        const PsgGraphT &reversePsgGraph;
        const int walkRadius;

        BitVector edgeYetToBeCovered;
        int numEdgesYetToBeCovered;

        PickupSearch pickupSearch;
        DropoffSearch dropoffSearch;

        ProgressBar progressBar;

    };

} // traffic_flow_subnetwork

