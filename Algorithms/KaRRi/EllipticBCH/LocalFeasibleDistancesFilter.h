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

#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"

namespace karri {

    // Removes all result entries from localDistances that are certain to not be relevant for the best assignment
    // based on existence of valid distances to and from the PDLocs as well as assignment cost bounds.
    template<PDLocType type, typename LocalFeasibleDistancesT>
    static void filterLocalEllipticDistances(const int firstIdInPdLocBatch, LocalFeasibleDistancesT &localDistances,
                                             const Fleet &fleet,
                                             const RequestState &requestState,
                                             const RouteState &routeState) {

        static constexpr int K = LocalFeasibleDistancesT::K;

        auto &indices = localDistances.indexInEntriesVector;
        auto &entries = localDistances.entries;

        int numEntriesRemoved = 0;
        for (int i = 0; i < entries.size(); ++i) {
            auto &e = entries[i];

            const auto &veh = fleet[routeState.vehicleIdOf(e.stopId)];
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            const auto &occupancies = routeState.occupanciesFor(veh.vehicleId);

            const auto &stopIdx = routeState.stopPositionOf(e.stopId);

            if ((stopIdx == numStops - 1 && type == PICKUP) || (occupancies[stopIdx] == veh.capacity && (type == PICKUP || stopIdx == 0))) {
                indices[e.stopId] = INVALID_INDEX;
                ++numEntriesRemoved;
                continue;
            }

            const auto &distTo = e.distFromStopToPDLoc;
            const auto &distFrom = e.distFromPDLocToNextStop;

            bool allFiltered = true;
            // todo SIMD-ify
            for (int idxInBatch = 0; idxInBatch < K; ++idxInBatch) {
                bool relevant = type == PICKUP ?
                                isPickupRelevant(veh, stopIdx, firstIdInPdLocBatch + idxInBatch, distTo[idxInBatch],
                                                 distFrom[idxInBatch], requestState, routeState) :
                                isDropoffRelevant(veh, stopIdx, firstIdInPdLocBatch + idxInBatch, distTo[idxInBatch],
                                                  distFrom[idxInBatch], requestState, routeState);
                if (relevant) {
                    allFiltered = false;
                    continue;
                }
                distTo[idxInBatch] = INFTY;
                distFrom[idxInBatch] = INFTY;
            }

            if (allFiltered) {
                indices[e.stopId] = INVALID_INDEX;
                ++numEntriesRemoved;
                continue;
            }
            assert(numEntriesRemoved <= i);
            entries[i - numEntriesRemoved] = entries[i];
            indices[e.stopId] = i - numEntriesRemoved;
        }
        entries.erase(entries.end() - numEntriesRemoved, entries.end());

    }

    namespace {
        static inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const unsigned int pickupId,
                                            const int distFromStopToPickup,
                                            const int distFromPickupToNextStop,
                                            const RequestState &requestState,
                                            const RouteState &routeState) {
            using namespace time_utils;

            CostCalculator calculator(routeState);

            const int &vehId = veh.vehicleId;

            assert(routeState.occupanciesFor(vehId)[stopIndex] < veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return false;

            assert(distFromStopToPickup + distFromPickupToNextStop >=
                   calcLengthOfLegStartingAt(stopIndex, vehId, routeState));

            const auto &p = requestState.pickups[pickupId];

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, p,
                                                                  requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     distFromPickupToNextStop, requestState,
                                                                     routeState);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;


            const int curKnownCost = calculator.calcMinKnownPickupSideCost(veh, stopIndex, initialPickupDetour,
                                                                           p.walkingDist, depTimeAtPickup,
                                                                           requestState);

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            if (curKnownCost > requestState.getBestCost())
                return false;

            return true;
        }

        static inline bool isDropoffRelevant(const Vehicle &veh, const int stopIndex, const unsigned int dropoffId,
                                             const int distFromStopToDropoff,
                                             const int distFromDropoffToNextStop,
                                             const RequestState &requestState,
                                             const RouteState &routeState) {
            using namespace time_utils;

            CostCalculator calculator(routeState);

            const int &vehId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &d = requestState.dropoffs[dropoffId];

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto &occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            assert(d.loc != stopLocations[stopIndex] || distFromStopToDropoff == 0);
            if (stopIndex == numStops - 1 || occupancy == veh.capacity)
                return d.loc == stopLocations[stopIndex];

            if (stopLocations[stopIndex + 1] == d.loc)
                return false;

            if (distFromStopToDropoff >= INFTY || distFromDropoffToNextStop >= INFTY)
                return false;

            const bool isDropoffAtExistingStop = d.loc == stopLocations[stopIndex];
            const int initialDropoffDetour = calcInitialDropoffDetour(vehId, stopIndex, distFromStopToDropoff,
                                                                      distFromDropoffToNextStop,
                                                                      isDropoffAtExistingStop,
                                                                      routeState);
            assert(initialDropoffDetour >= 0);
            if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopIndex, initialDropoffDetour,
                                                        routeState))
                return false;

            const int curMinCost = calculator.calcMinKnownDropoffSideCost(veh, stopIndex, initialDropoffDetour,
                                                                          d.walkingDist, requestState);

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            if (curMinCost > requestState.getBestCost())
                return false;

            return true;
        }
    }
}