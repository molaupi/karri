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
#include <cstdint>
#include <Algorithms/KaRRi/BaseObjects//Request.h>
#include <random>
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/InputConfig.h"

namespace karri {


// Given an origin / a destination location in a passenger network, this class finds the closest location in a
// vehicle network that is suited for a pickup / dropoff location by vehicles that are restricted to the vehicle network.
// The rider is expected to travel from the origin to the pickup / from the dropoff to the destination themselves
// in the passenger network using a mode of transport associated with this network (e.g. walking, cycling).
// The closest suitable pickup location in the vehicle network is the one which takes the least time to get to from the
// origin using the passenger mode of transport. The closest dropoff location is analogous.

    template<typename PassengerGraphT, typename WeightT = TravelTimeAttribute>
    class FindClosestPDLocInVehicleNetworkQuery {

    private:

        struct UpdateClosestPickup {

            UpdateClosestPickup(const PassengerGraphT& forwardGraph, PDLoc& closestPickup) : forwardGraph(forwardGraph), closestPickup(closestPickup) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {

                FORALL_INCIDENT_EDGES(forwardGraph, v, e) {

                    // If any of the incident edges are accessible in the (reduced) vehicle network, they are potential
                    // pickup locations.
                    const int eInReducedVehGraph = forwardGraph.mapToEdgeInReducedVeh(e);
                    if (eInReducedVehGraph == MapToEdgeInReducedVehAttribute::defaultValue())
                        continue;

                    // The distance to the location is the distance to v + the distance of the edge
                    // itself. If this distance is better than the best known pickup, we update it.
                    const auto distToE = distToV[0] + forwardGraph.travelTime(e);
                    if (distToE >= closestPickup.walkingDist)
                        continue;

                    const int eInFullVehGraph = forwardGraph.mapToEdgeInFullVeh(e);
                    KASSERT(eInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue());
                    closestPickup = {0, eInReducedVehGraph, eInFullVehGraph, e, distToV[0] + forwardGraph.travelTime(e),
                            INFTY, INFTY};
                }
                return false;
            }

        private:
            const PassengerGraphT& forwardGraph;
            PDLoc& closestPickup;
        };

        struct UpdateClosestDropoff {

            UpdateClosestDropoff(const PassengerGraphT& forwardGraph, const PassengerGraphT& reverseGraph, PDLoc& closestDropoff)
            : forwardGraph(forwardGraph), reverseGraph(reverseGraph), closestDropoff(closestDropoff) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {

                FORALL_INCIDENT_EDGES(reverseGraph, v, e) {
                    const auto eInForwGraph = reverseGraph.edgeId(e);
                    const int eInReducedVehGraph = forwardGraph.mapToEdgeInReducedVeh(eInForwGraph);
                    if (eInReducedVehGraph == MapToEdgeInReducedVehAttribute::defaultValue())
                        continue;

                    // The distance to the location is the distance to v. If this distance is better than the best
                    // known dropoff, we update it.
                    const auto distToE = distToV[0];
                    if (distToE >= closestDropoff.walkingDist)
                        continue;

                    const int eInFullVehGraph = forwardGraph.mapToEdgeInFullVeh(eInForwGraph);
                    KASSERT(eInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue());
                    closestDropoff = {0, eInReducedVehGraph, eInFullVehGraph, eInForwGraph, distToV[0], INFTY, INFTY};
                }
                return false;
            }

        private:
            const PassengerGraphT& forwardGraph;
            const PassengerGraphT& reverseGraph;
            PDLoc& closestDropoff;
        };

        struct StopIfDistExceedsClosestPdLoc {
            StopIfDistExceedsClosestPdLoc(const PDLoc& closestPdLoc) : closestPdLoc(closestPdLoc) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return distToV[0] >= closestPdLoc.walkingDist;
            }

        private:
            const PDLoc& closestPdLoc;
        };

        using PickupSearch = Dijkstra<PassengerGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopIfDistExceedsClosestPdLoc, UpdateClosestPickup>;
        using DropoffSearch = Dijkstra<PassengerGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopIfDistExceedsClosestPdLoc, UpdateClosestDropoff>;

    public:

        FindClosestPDLocInVehicleNetworkQuery(const PassengerGraphT &forwardPsgGraph,
                                              const PassengerGraphT &reversePsgGraph,
                                              std::vector<PDLoc> &pickups,
                                              std::vector<PDLoc> &dropoffs)
                : forwardGraph(forwardPsgGraph),
                  reverseGraph(reversePsgGraph),
                  pickups(pickups),
                  dropoffs(dropoffs),
                  closestPickup(),
                  closestDropoff(),
                  pickupSearch(forwardPsgGraph, {closestPickup}, {forwardGraph, closestPickup}),
                  dropoffSearch(reversePsgGraph, {closestDropoff}, {forwardGraph, reverseGraph, closestDropoff}) {}

        // Pickup will be written into the pickups vector and dropoff will be written into the dropoffs vector.
        void findPDLocs(const int origin, const int destination) {
            KASSERT(origin < forwardGraph.numEdges() && destination < forwardGraph.numEdges());
            pickups.clear();
            dropoffs.clear();
            closestPickup = PDLoc(INVALID_ID, INVALID_EDGE, INVALID_EDGE, INVALID_EDGE, INFTY, INFTY, INFTY);
            closestDropoff = PDLoc(INVALID_ID, INVALID_EDGE, INVALID_EDGE, INVALID_EDGE, INFTY, INFTY, INFTY);

            auto headOfOriginEdge = forwardGraph.edgeHead(origin);
            pickupSearch.run(headOfOriginEdge);

            auto tailOfDestEdge = forwardGraph.edgeTail(destination);
            auto destOffset = forwardGraph.travelTime(destination);
            dropoffSearch.runWithOffset(tailOfDestEdge, destOffset);

            KASSERT(closestPickup.psgLoc < forwardGraph.numEdges() && closestPickup.walkingDist < INFTY);
            KASSERT(closestDropoff.psgLoc < forwardGraph.numEdges() && closestDropoff.walkingDist < INFTY);

            pickups  = {closestPickup};
            dropoffs = {closestDropoff};
            KASSERT(sanityCheckPDLocs(pickups, forwardGraph));
            KASSERT(sanityCheckPDLocs(dropoffs, forwardGraph));
        }

    private:

        static bool sanityCheckPDLocs(const std::vector<PDLoc> &pdLocs, const PassengerGraphT& psgGraph) {
            for (int i = 0; i < pdLocs.size(); ++i) {
                if (pdLocs[i].id != i) return false;
                if (pdLocs[i].vehDistToCenter != INFTY) return false;
                if (pdLocs[i].vehDistFromCenter != INFTY) return false;
                if (psgGraph.mapToEdgeInReducedVeh(pdLocs[i].psgLoc) != pdLocs[i].loc) return false;
                if (psgGraph.mapToEdgeInFullVeh(pdLocs[i].psgLoc) != pdLocs[i].fullVehLoc) return false;
            }
            return true;
        }

        const PassengerGraphT &forwardGraph;
        const PassengerGraphT &reverseGraph;
        std::vector<PDLoc> &pickups;
        std::vector<PDLoc> &dropoffs;
        PDLoc closestPickup;
        PDLoc closestDropoff;
        PickupSearch pickupSearch;
        DropoffSearch dropoffSearch;
    };
}