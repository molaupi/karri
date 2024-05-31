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
#include <Algorithms/KaRRi/BaseObjects//Request.h>
#include <random>
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"

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
                                const PassengerGraphT &reversePsgGraph,
                                std::vector<PDLoc> &pickups,
                                std::vector<PDLoc> &dropoffs)
                : forwardGraph(forwardPsgGraph),
                  reverseGraph(reversePsgGraph),
                  pickups(pickups),
                  dropoffs(dropoffs),
                  pickupSearch(forwardPsgGraph, {InputConfig::getInstance().pickupRadius},
                               {searchSpace}),
                  dropoffSearch(reversePsgGraph, {InputConfig::getInstance().dropoffRadius},
                                {searchSpace}),
                  searchSpace(),
                  rand(seed) {}

        // Pickups will be collected into the given pickups vector and dropoffs will be collected into the given dropoffs vector
        void findPDLocs(const int origin, const int destination) {
            KASSERT(origin < forwardGraph.numEdges() && destination < forwardGraph.numEdges());
            pickups.clear();
            dropoffs.clear();

            searchSpace.clear();
            auto headOfOriginEdge = forwardGraph.edgeHead(origin);
            pickupSearch.run(headOfOriginEdge);
            turnSearchSpaceIntoPickupLocations();

            searchSpace.clear();
            auto tailOfDestEdge = forwardGraph.edgeTail(destination);
            auto destOffset = forwardGraph.travelTime(destination);
            dropoffSearch.runWithOffset(tailOfDestEdge, destOffset);
            turnSearchSpaceIntoDropoffLocations();

            finalizePDLocs(origin, pickups, InputConfig::getInstance().maxNumPickups);
            finalizePDLocs(destination, dropoffs, InputConfig::getInstance().maxNumDropoffs);
        }

    private:

        void turnSearchSpaceIntoPickupLocations() {
            for (const auto &v: searchSpace) {
                const auto distToV = pickupSearch.getDistance(v);
                KASSERT(distToV <= InputConfig::getInstance().pickupRadius);
                FORALL_INCIDENT_EDGES(forwardGraph, v, e) {
                    const int eInReducedVehGraph = forwardGraph.mapToEdgeInReducedVeh(e);
                    if (eInReducedVehGraph == MapToEdgeInReducedVehAttribute::defaultValue() ||
                        distToV + forwardGraph.travelTime(e) > InputConfig::getInstance().pickupRadius)
                        continue;

                    const int eInFullVehGraph = forwardGraph.mapToEdgeInFullVeh(e);
                    KASSERT(eInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue());
                    pickups.push_back(
                            {INVALID_ID, eInReducedVehGraph, eInFullVehGraph, e, distToV + forwardGraph.travelTime(e),
                             INFTY, INFTY});
                }
            }
        }

        void turnSearchSpaceIntoDropoffLocations() {
            for (const auto &v: searchSpace) {
                const auto distToV = dropoffSearch.getDistance(v);
                KASSERT(distToV <= InputConfig::getInstance().dropoffRadius);
                FORALL_INCIDENT_EDGES(reverseGraph, v, e) {
                    const auto eInForwGraph = reverseGraph.edgeId(e);
                    const int eInReducedVehGraph = forwardGraph.mapToEdgeInReducedVeh(eInForwGraph);
                    if (eInReducedVehGraph == MapToEdgeInReducedVehAttribute::defaultValue())
                        continue;
                    const int eInFullVehGraph = forwardGraph.mapToEdgeInFullVeh(eInForwGraph);
                    KASSERT(eInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue());
                    dropoffs.push_back(
                            {INVALID_ID, eInReducedVehGraph, eInFullVehGraph, eInForwGraph, distToV, INFTY, INFTY});
                }
            }
        }

        void finalizePDLocs(const int centerInPsgGraph, std::vector<PDLoc> &pdLocs, const int maxNumber) {
            KASSERT(maxNumber > 0);
            // If center is accessible in reduced vehicle network, add center to PD locs
            const bool centerIsPdLoc = forwardGraph.mapToEdgeInReducedVeh(centerInPsgGraph) !=
                                       MapToEdgeInReducedVehAttribute::defaultValue();
            if (centerIsPdLoc) {
                const int nextSeqId = pdLocs.size();
                const int centerInRedVehGraph = forwardGraph.mapToEdgeInReducedVeh(centerInPsgGraph);
                const int centerInFullVehGraph = forwardGraph.mapToEdgeInFullVeh(centerInPsgGraph);
                KASSERT(centerInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue());
                pdLocs.push_back(
                        {nextSeqId, centerInRedVehGraph, centerInFullVehGraph, centerInPsgGraph, 0, INFTY, INFTY});
            }

            // Remove duplicates
            removeDuplicates(pdLocs);

            if (centerIsPdLoc) {
                // Make sure center is at beginning
                const int centerInRedVehGraph = forwardGraph.mapToEdgeInReducedVeh(centerInPsgGraph);
                auto centerIt = std::find_if(pdLocs.begin(), pdLocs.end(),
                                             [centerInRedVehGraph](const auto &h) { return h.loc == centerInRedVehGraph; });
                KASSERT(centerIt < pdLocs.end());
                const auto idx = centerIt - pdLocs.begin();
                std::swap(pdLocs[0], pdLocs[idx]);
            }

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

            KASSERT(sanityCheckPDLocs(pdLocs, forwardGraph));
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

        static bool sanityCheckPDLocs(const std::vector<PDLoc> &pdLocs, const PassengerGraphT& psgGraph) {
            if (pdLocs.empty()) return false;
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
        PickupSearch pickupSearch;
        DropoffSearch dropoffSearch;

        std::vector<int> searchSpace;

        static constexpr int seed = 42;
        std::minstd_rand rand;
    };
}