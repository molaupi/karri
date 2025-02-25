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

#include <random>
#include "Tools/Timer.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {


// This class finds every possible location suited for passenger pickups or dropoffs in a given radius around the
// origin or destination of a request. When queried, a local Dijkstra search bounded by the given radius is executed
// around the center point.
    template<typename PassengerGraphT, typename WeightT = TravelTimeAttribute>
    class FindPDLocsInRadiusQuery {

    private:
        struct StopWhenRadiusExceeded {
            StopWhenRadiusExceeded(const int radius) : radius(radius) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return distToV[0] > radius;
            }

        private:
            const int radius;
        };

        struct RememberSearchSpace {

            RememberSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                searchSpace.push_back(v);
                return false;
            }

        private:
            std::vector<int> &searchSpace;

        };

        using PickupSearch = Dijkstra<PassengerGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopWhenRadiusExceeded, RememberSearchSpace>;
        using DropoffSearch = Dijkstra<PassengerGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopWhenRadiusExceeded, RememberSearchSpace>;

    public:

        FindPDLocsInRadiusQuery(const PassengerGraphT &forwardPsgGraph,
                                const PassengerGraphT &reversePsgGraph)
                : forwardGraph(forwardPsgGraph),
                  reverseGraph(reversePsgGraph),
                  pickupSearch(forwardPsgGraph, {InputConfig::getInstance().pickupRadius},
                               {searchSpace}),
                  dropoffSearch(reversePsgGraph, {InputConfig::getInstance().dropoffRadius},
                                {searchSpace}),
                  searchSpace(),
                  rand(seed) {}

        // Pickups will be collected into the given pickups vector and dropoffs will be collected into the given dropoffs vector
        void findPDLocs(const int origin, const int destination, std::vector<PDLoc>& pickups, std::vector<PDLoc>& dropoffs) {
            assert(origin < forwardGraph.numEdges() && destination < forwardGraph.numEdges());
            pickups.clear();
            dropoffs.clear();

            searchSpace.clear();
            auto headOfOriginEdge = forwardGraph.edgeHead(origin);
            pickupSearch.run(headOfOriginEdge);
            turnSearchSpaceIntoPickupLocations(pickups);

            searchSpace.clear();
            auto tailOfDestEdge = forwardGraph.edgeTail(destination);
            auto destOffset = forwardGraph.travelTime(destination);
            dropoffSearch.runWithOffset(tailOfDestEdge, destOffset);
            turnSearchSpaceIntoDropoffLocations(dropoffs);

            finalizePDLocs(origin, pickups, InputConfig::getInstance().maxNumPickups);
            finalizePDLocs(destination, dropoffs, InputConfig::getInstance().maxNumDropoffs);
        }

    private:

        void turnSearchSpaceIntoPickupLocations(std::vector<PDLoc>& pickups) {
            for (const auto &v: searchSpace) {
                const auto distToV = pickupSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().pickupRadius);
                FORALL_INCIDENT_EDGES(forwardGraph, v, e) {
                    const int eInVehGraph = forwardGraph.toCarEdge(e);
                    if (eInVehGraph == PsgEdgeToCarEdgeAttribute::defaultValue() ||
                        distToV + forwardGraph.travelTime(e) > InputConfig::getInstance().pickupRadius)
                        continue;

                    pickups.push_back({INVALID_ID, eInVehGraph, e, distToV + forwardGraph.travelTime(e), INFTY, INFTY});
                }
            }
        }

        void turnSearchSpaceIntoDropoffLocations(std::vector<PDLoc>& dropoffs) {
            for (const auto &v: searchSpace) {
                const auto distToV = dropoffSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().dropoffRadius);
                FORALL_INCIDENT_EDGES(reverseGraph, v, e) {
                    const auto eInForwGraph = reverseGraph.edgeId(e);
                    const int eInVehGraph = forwardGraph.toCarEdge(eInForwGraph);
                    if (eInVehGraph == PsgEdgeToCarEdgeAttribute::defaultValue())
                        continue;
                    dropoffs.push_back({INVALID_ID, eInVehGraph, eInForwGraph, distToV, INFTY, INFTY});
                }
            }
        }

        void finalizePDLocs(const int centerInPsgGraph, std::vector<PDLoc> &pdLocs, const int maxNumber) {
            assert(maxNumber > 0);
            // Add center to PD locs
            const int nextSeqId = pdLocs.size();
            const int centerInVehGraph = forwardGraph.toCarEdge(centerInPsgGraph);
            pdLocs.push_back({nextSeqId, centerInVehGraph, centerInPsgGraph, 0, INFTY, INFTY});

            // Remove duplicates
            removeDuplicates(pdLocs);

            // Make sure center is at beginning
            auto centerIt = std::find_if(pdLocs.begin(), pdLocs.end(),
                                         [centerInVehGraph](const auto &h) { return h.loc == centerInVehGraph; });
            assert(centerIt < pdLocs.end());
            const auto idx = centerIt - pdLocs.begin();
            std::swap(pdLocs[0], pdLocs[idx]);

            if (maxNumber > 1 && pdLocs.size() > maxNumber) {
                // If there are more PD-locs than the maximum number, then we permute the PD-locs randomly and
                // use only the first maxNumber ones. We make sure that the center is included and stays at the
                // beginning of the PD-locs.
                const auto perm = Permutation::getRandomPermutation(pdLocs.size(), rand);
                perm.applyTo(pdLocs);
                std::swap(pdLocs[perm[0]], pdLocs[0]);
            }

            const int desiredSize = std::min(static_cast<int>(pdLocs.size()), maxNumber);
            pdLocs.resize(desiredSize);

            // Assign sequential ids
            for (int i = 0; i < pdLocs.size(); ++i) {
                pdLocs[i].id = i;
            }

            assert(sanityCheckPDLocs(pdLocs, centerInVehGraph));
        }

        // Remove duplicate PDLocs.
        // Out of each set of PDLocs that have same location, we keep only the one with the shortest walking distance.
        static void removeDuplicates(std::vector<PDLoc> &pdLocs) {
            std::sort(pdLocs.begin(), pdLocs.end(), [](const auto &h1, const auto &h2) {
                return h1.loc < h2.loc || (h1.loc == h2.loc && h1.walkingDist < h2.walkingDist);
            });
            auto last = std::unique(pdLocs.begin(), pdLocs.end(), [](const auto &h1, const auto &h2) {
                return h1.loc == h2.loc;
            });
            pdLocs.erase(last, pdLocs.end());
        }

        static bool sanityCheckPDLocs(const std::vector<PDLoc> &pdLocs, const int centerInVehGraph) {
            if (pdLocs.empty()) return false;
            if (pdLocs[0].loc != centerInVehGraph) return false;
            if (pdLocs[0].walkingDist != 0) return false;
            for (int i = 0; i < pdLocs.size(); ++i) {
                if (pdLocs[i].id != i) return false;
                if (pdLocs[i].vehDistToCenter != INFTY) return false;
                if (pdLocs[i].vehDistFromCenter != INFTY) return false;
            }
            return true;
        }

        const PassengerGraphT &forwardGraph;
        const PassengerGraphT &reverseGraph;
        PickupSearch pickupSearch;
        DropoffSearch dropoffSearch;

        std::vector<int> searchSpace;

        static constexpr int seed = 42;
        std::minstd_rand rand;
    };



    // Initializes the pickup and dropoff locations for a new request.
    template<typename VehInputGraphT,
            typename PsgInputGraphT,
            typename VehicleToPDLocQueryT>
    class PDLocsFinder {

    public:
        PDLocsFinder(const VehInputGraphT &vehInputGraph, const PsgInputGraphT &psgInputGraph,
                                const PsgInputGraphT& revPsgGraph,
                                VehicleToPDLocQueryT &vehicleToPdLocQuery)
                : vehInputGraph(vehInputGraph), psgInputGraph(psgInputGraph),
                  findPdLocsInRadiusQuery(psgInputGraph, revPsgGraph),
                  vehicleToPdLocQuery(vehicleToPdLocQuery) {}


        PDLocs findPDLocs(const int origin, const int destination, stats::InitializationPerformanceStats& stats) {
            Timer timer;

            KASSERT(psgInputGraph.toCarEdge(vehInputGraph.toPsgEdge(origin)) == origin);
            const auto originInPsgGraph = vehInputGraph.toPsgEdge(origin);

            KASSERT(psgInputGraph.toCarEdge(vehInputGraph.toPsgEdge(destination)) == destination);
            const auto destInPsgGraph = vehInputGraph.toPsgEdge(destination);

            PDLocs pdLocs;
            findPdLocsInRadiusQuery.findPDLocs(originInPsgGraph, destInPsgGraph, pdLocs.pickups, pdLocs.dropoffs);


            const auto findHaltingSpotsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.findPDLocsInRadiusTime = findHaltingSpotsTime;
            timer.restart();

            // Precalculate the vehicle distances from pickups to origin and from destination to dropoffs for upper bounds on PD distances
            vehicleToPdLocQuery.runReverse(pdLocs.pickups);
            vehicleToPdLocQuery.runForward(pdLocs.dropoffs);

            const auto findVehicleToPdLocsDistancesTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.findVehicleToPdLocsDistancesTime = findVehicleToPdLocsDistancesTime;

            return pdLocs;
        }


    private:

        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;

        FindPDLocsInRadiusQuery<PsgInputGraphT> findPdLocsInRadiusQuery;
        VehicleToPDLocQueryT &vehicleToPdLocQuery;

    };
}