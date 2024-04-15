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


    namespace {

        template<typename LabelSet>
        inline typename LabelSet::LabelMask
        isPickupRelevant(const Vehicle &veh, const int stopIndex,
                         const typename LabelSet::DistanceLabel &pickupLocs,
                         const typename LabelSet::DistanceLabel &walkingTimes,
                         const typename LabelSet::DistanceLabel &distFromStopToPickup,
                         const typename LabelSet::DistanceLabel &distFromPickupToNextStop,
                         const RequestState &requestState,
                         const RouteState &routeState) {
            using namespace time_utils;
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            CostCalculator calculator(routeState);

            LabelMask relevant(true);

            const int &vehId = veh.vehicleId;

            assert(routeState.occupanciesFor(vehId)[stopIndex] < veh.capacity);
            relevant &= (distFromStopToPickup < INFTY) & (distFromPickupToNextStop < INFTY);

            assert(allSet(distFromStopToPickup + distFromPickupToNextStop >=
                          DistanceLabel(calcLengthOfLegStartingAt(stopIndex, vehId, routeState))));

            const DistanceLabel passengerArrTimesAtPickups =
                    DistanceLabel(requestState.originalRequest.requestTime) + walkingTimes;
            const DistanceLabel depTimeAtPickup = getActualDepTimeAtPickup<LabelSet>(vehId, stopIndex, distFromStopToPickup,
                                                                           pickupLocs, passengerArrTimesAtPickups,
                                                                           requestState, routeState);
            const DistanceLabel initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX,
                                                                              depTimeAtPickup, distFromPickupToNextStop,
                                                                              requestState, routeState);

            relevant &= ~doesPickupDetourViolateHardConstraints<LabelSet>(veh, requestState, stopIndex, initialPickupDetour,
                                                                routeState);

            const DistanceLabel curKnownCost =
                    calculator.calcMinKnownPickupSideCost<LabelSet>(veh, stopIndex, initialPickupDetour, walkingTimes,
                                                          depTimeAtPickup, requestState);

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            relevant &= curKnownCost <= requestState.getBestCost();
            return relevant;
        }

        template<typename LabelSet>
        inline typename LabelSet::LabelMask
        isDropoffRelevant(const Vehicle &veh, const int stopIndex,
                          const typename LabelSet::DistanceLabel &dropoffLocs,
                          const typename LabelSet::DistanceLabel &walkingTimes,
                          const typename LabelSet::DistanceLabel &distFromStopToDropoff,
                          const typename LabelSet::DistanceLabel &distFromDropoffToNextStop,
                          const RequestState &requestState,
                          const RouteState &routeState) {
            using namespace time_utils;
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            CostCalculator calculator(routeState);


            const int &vehId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto &occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            assert(allSet((dropoffLocs != stopLocations[stopIndex]) | (distFromStopToDropoff == 0)));
            if (stopIndex == numStops - 1 || occupancy == veh.capacity)
                return dropoffLocs == stopLocations[stopIndex];

            LabelMask relevant(true);

            relevant &= dropoffLocs != stopLocations[stopIndex + 1];
            relevant &= (distFromStopToDropoff < INFTY) & (distFromDropoffToNextStop < INFTY);

            const LabelMask isDropoffAtExistingStop = dropoffLocs == stopLocations[stopIndex];
            const DistanceLabel initialDropoffDetour = calcInitialDropoffDetour<LabelSet>(vehId, stopIndex, distFromStopToDropoff,
                                                                                distFromDropoffToNextStop,
                                                                                isDropoffAtExistingStop,
                                                                                routeState);
            assert(allSet(initialDropoffDetour >= 0));
            relevant &= ~doesDropoffDetourViolateHardConstraints<LabelSet>(veh, requestState, stopIndex, initialDropoffDetour,
                                                                 routeState);

            const DistanceLabel curMinCost =
                    calculator.calcMinKnownDropoffSideCost<LabelSet>(veh, stopIndex, initialDropoffDetour,
                                                           walkingTimes, requestState);

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            relevant &= curMinCost <= requestState.getBestCost();
            return relevant;
        }

        // Default operator for stop eligibility. Considers every stop eligible.
        struct StopAlwaysEligible {
            bool operator()(const int) const {
                return true;
            }
        };
    }

    // Removes all result entries from localDistances that are certain to not be relevant for the best assignment
    // based on existence of valid distances to and from the PDLocs as well as assignment cost bounds.
    // May pass additional filter for eligibility of stops s.t. any entry of this stop is always considered irrelevant.
    template<PDLocType type, typename LabelSet, typename LocalFeasibleDistancesT,
            typename IsStopEligibleT = StopAlwaysEligible>
    static void filterLocalEllipticDistances(const int firstPdLocIdInBatch,
                                             LocalFeasibleDistancesT &localDistances,
                                             const Fleet &fleet,
                                             const RequestState &requestState,
                                             const RouteState &routeState,
                                             const IsStopEligibleT &isStopEligible = StopAlwaysEligible()) {

        static constexpr int K = LocalFeasibleDistancesT::K;

        typename LabelSet::DistanceLabel pdLocLocations = {};
        typename LabelSet::DistanceLabel pdLocWalkingTimes = {};
        const auto& pdLocs = type == PICKUP? requestState.pickups : requestState.dropoffs;
        for (int j = firstPdLocIdInBatch; j < firstPdLocIdInBatch + K; ++j) {
            pdLocLocations[j - firstPdLocIdInBatch] = j < pdLocs.size()? pdLocs[j].loc : INVALID_EDGE;
            pdLocWalkingTimes[j - firstPdLocIdInBatch] = j < pdLocs.size()? pdLocs[j].walkingDist : INFTY;
        }

        auto &indices = localDistances.indexInEntriesVector;
        auto &entries = localDistances.entries;

        int numEntriesRemoved = 0;
        for (int i = 0; i < entries.size(); ++i) {
            auto &e = entries[i];

            const auto &veh = fleet[routeState.vehicleIdOf(e.stopId)];
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            const auto &occupancies = routeState.occupanciesFor(veh.vehicleId);
            assert(numStops > 1);

            const auto &stopIdx = routeState.stopPositionOf(e.stopId);

            if (!isStopEligible(e.stopId) ||
                (stopIdx == numStops - 1 && type == PICKUP) ||
                (occupancies[stopIdx] == veh.capacity && (type == PICKUP || stopIdx == 0))) {
                indices[e.stopId] = INVALID_INDEX;
                ++numEntriesRemoved;
                continue;
            }

            auto &distTo = e.distFromStopToPdLoc;
            auto &distFrom = e.distFromPdLocToNextStop;

            const typename LabelSet::LabelMask notRelevant = type == PICKUP ?
                                                          ~isPickupRelevant<LabelSet>(veh, stopIdx, pdLocLocations,
                                                                           pdLocWalkingTimes, distTo, distFrom,
                                                                           requestState, routeState) :
                                                          ~isDropoffRelevant<LabelSet>(veh, stopIdx, pdLocLocations,
                                                                            pdLocWalkingTimes, distTo, distFrom,
                                                                            requestState, routeState);

            if (allSet(notRelevant)) {
                indices[e.stopId] = INVALID_INDEX;
                ++numEntriesRemoved;
                continue;
            }

            distTo.setIf(INFTY, notRelevant);
            distFrom.setIf(INFTY, notRelevant);

            assert(numEntriesRemoved <= i);
            entries[i - numEntriesRemoved] = entries[i];
            indices[e.stopId] = i - numEntriesRemoved;
        }
        entries.erase(entries.end() - numEntriesRemoved, entries.end());

    }
}