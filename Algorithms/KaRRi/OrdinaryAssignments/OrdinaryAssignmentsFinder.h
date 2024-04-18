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

#include "Tools/Timer.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"

namespace karri {

// Finds ordinary assignments, i.e. those assignments where pickup and dropoff are both inserted after the vehicle's
// next stop but before the vehicle's last stop. This includes ordinary paired assignments where the pickup and dropoff
// are inserted between the same pair of existing stops.
//
// Works based on filtered relevant PD locs.
    template<typename PDDistancesT>
    class OrdinaryAssignmentsFinder {

        struct ThreadLocalBestAsgnInfo {

            ThreadLocalBestAsgnInfo() {}

            Assignment bestAsgn{};
            int bestCost = INFTY;
            int numAssignmentsTried = 0;

            bool tryAssignment(const Assignment &asgn, const CostCalculator& pCalculator, const RequestState& pRequestState) {
                ++numAssignmentsTried;
                const auto cost = pCalculator.calc(asgn, pRequestState);

                if (cost >= INFTY || !(cost < bestCost || (cost == bestCost && breakCostTie(asgn, bestAsgn))))
                    return false;

                bestAsgn = asgn;
                bestCost = cost;
                return true;
            }

        private:

        };

    public:

        OrdinaryAssignmentsFinder(const RelevantPDLocs &relPickups, const RelevantPDLocs &relDropoffs,
                                  const PDDistancesT &pdDistances, const Fleet &fleet,
                                  const CostCalculator &calculator, const RouteState &routeState,
                                  RequestState &requestState)
                : relPickups(relPickups),
                  relDropoffs(relDropoffs),
                  pdDistances(pdDistances),
                  fleet(fleet),
                  calculator(calculator),
                  routeState(routeState),
                  requestState(requestState),
                  localBestAsgnInfo() {}

        void findAssignments() {
//            findOrdinaryAssignments();
//            findOrdinaryPairedAssignments();
            parFindOrdinaryAssignments();
        }

        void init() {
            // no op
        }

    private:

        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void parFindOrdinaryAssignments() {

            Timer timer;
            CAtomic<int> numCandidateVehicles{0};
            localBestAsgnInfo.clear();

            const auto &vehicles = relPickups.getVehiclesWithRelevantPDLocs();
            tbb::parallel_for(0, static_cast<int>(vehicles.size()), [&](const int i) {
                const auto &vehId = *(vehicles.begin() + i);

                if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
                    return;

                if (!relPickups.hasRelevantSpotsFor(vehId) ||
                    !relDropoffs.hasRelevantSpotsFor(vehId))
                    return;

                numCandidateVehicles.fetch_add(1, std::memory_order_relaxed);

                const auto numStops = routeState.numStopsOf(vehId);
                const auto &stopLocations = routeState.stopLocationsFor(vehId);
                const auto &relevantPickups = relPickups.relevantSpotsFor(vehId);
                const auto &relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);

                tbb::parallel_for(0, static_cast<int>(relevantPickups.size()), [&](const int j) {

                    auto &localBest = localBestAsgnInfo.local();
                    const auto &pickupEntry = *(relevantPickups.begin() + j);
                    const auto pickupStopIdx = routeState.stopPositionOf(pickupEntry.stopId);

                    Assignment asgn(&fleet[vehId]);
                    asgn.pickup = &requestState.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupStopIdx;
                    asgn.distToPickup = pickupEntry.distToPDLoc;

                    // Find first stop position after the pickup's stop position that has relevant dropoffs.
                    for (auto dropoffIt = relevantDropoffs.begin(); dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                        const auto &dropoffEntry = *dropoffIt;
                        const auto dropoffStopIdx = routeState.stopPositionOf(dropoffEntry.stopId);
                        if (dropoffStopIdx < pickupStopIdx)
                            continue; // Initially advance until dropoff is at same stop as or later stop than pickup

                        asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                        if (dropoffStopIdx + 1 < numStops && stopLocations[dropoffStopIdx + 1] == asgn.dropoff->loc) {
                            // If the dropoff is at the location of the following stop, do not try an assignment here as it would
                            // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                            // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                            continue;
                        }

                        asgn.dropoffStopIdx = dropoffStopIdx;
                        asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;

                        if (pickupStopIdx == dropoffStopIdx) {
                            asgn.distFromPickup = 0;
                            asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        } else {
                            if (asgn.dropoff->loc == asgn.pickup->loc) {
                                // In this case, this spot is the best spot at or after pickupIndex and the best spot at or after
                                // dropoffIndex. We ignore this case here since inserting them paired into the same leg will be better.
                                continue;
                            }
                            asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                            asgn.distToDropoff = dropoffEntry.distToPDLoc;
                        }

                        localBest.tryAssignment(asgn, calculator, requestState);
                    }

                });

            });

            int totalNumAssignmentsTried = 0;
            for (auto &localInfo: localBestAsgnInfo) {
                if (localInfo.bestCost < INFTY)
                    requestState.tryAssignmentWithKnownCost(localInfo.bestAsgn, localInfo.bestCost);
                totalNumAssignmentsTried += localInfo.numAssignmentsTried;
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.tryNonPairedAssignmentsTime += time;
            requestState.stats().ordAssignmentsStats.numCandidateVehicles += numCandidateVehicles.load(
                    std::memory_order_seq_cst);
            requestState.stats().ordAssignmentsStats.numAssignmentsTried += totalNumAssignmentsTried;
        }
//
//        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
//        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
//        // dropoff after a last stop.
//        void findOrdinaryAssignments() {
//
//            Timer timer;
//            int numCandidateVehicles = 0;
//            int numAssignmentsTried = 0;
//
//            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
//                if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
//                    continue;
//
//                if (!relPickups.hasRelevantSpotsFor(vehId) ||
//                    !relDropoffs.hasRelevantSpotsFor(vehId))
//                    continue;
//
//                ++numCandidateVehicles;
//                Assignment asgn(&fleet[vehId]);
//
//                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
//                auto curFirstDropoffIt = relevantDropoffs.begin();
//
//                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
//
//                    // Find first stop position after the pickup's stop position that has relevant dropoffs.
//                    const auto &stopIdx = routeState.stopPositionOf(pickupEntry.stopId);
//                    while (curFirstDropoffIt < relevantDropoffs.end() &&
//                           routeState.stopPositionOf(curFirstDropoffIt->stopId) <= stopIdx) {
//                        ++curFirstDropoffIt;
//                    }
//                    if (curFirstDropoffIt == relevantDropoffs.end())
//                        break; // No dropoffs later in route than current (or subsequent) pickup(s)
//
//                    asgn.pickup = &requestState.pickups[pickupEntry.pdId];
//                    asgn.pickupStopIdx = stopIdx;
//                    asgn.distToPickup = pickupEntry.distToPDLoc;
//                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
//
//                    numAssignmentsTried += tryDropoffLaterThanPickup(asgn, curFirstDropoffIt);
//                }
//            }
//
//            const auto time = timer.elapsed<std::chrono::nanoseconds>();
//            requestState.stats().ordAssignmentsStats.tryNonPairedAssignmentsTime += time;
//            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
//            requestState.stats().ordAssignmentsStats.numCandidateVehicles += numCandidateVehicles;
//
//        }
//
//        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
//        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
//        // completes the assignment with those dropoffs, and tries the resulting assignments.
//        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
//        int tryDropoffLaterThanPickup(Assignment &asgn,
//                                      const RelevantPDLocs::It &startItInRegularDropoffs) {
//            assert(asgn.vehicle && asgn.pickup);
//            const auto &vehId = asgn.vehicle->vehicleId;
//
//            const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
//            assert(startItInRegularDropoffs >= relevantDropoffs.begin() &&
//                   startItInRegularDropoffs <= relevantDropoffs.end());
//
//            if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
//                return 0;
//
//            auto numAssignmentsTriedWithOrdinaryDropoff = 0;
//
//            const auto &numStops = routeState.numStopsOf(vehId);
//            const auto &stopLocations = routeState.stopLocationsFor(vehId);
//
//            for (auto dropoffIt = startItInRegularDropoffs; dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
//                const auto &dropoffEntry = *dropoffIt;
//                asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];
//                const auto stopIdx = routeState.stopPositionOf(dropoffEntry.stopId);
//
//                if (stopIdx + 1 < numStops &&
//                    stopLocations[stopIdx + 1] == asgn.dropoff->loc) {
//                    // If the dropoff is at the location of the following stop, do not try an assignment here as it would
//                    // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
//                    // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
//                    continue;
//                }
//
//                if (asgn.dropoff->loc == asgn.pickup->loc) {
//                    // In this case, this spot is the best spot at or after pickupIndex and the best spot at or after
//                    // dropoffIndex. We ignore this case here since inserting them paired into the same leg will be better.
//                    continue;
//                }
//
//                asgn.dropoffStopIdx = stopIdx;
//                asgn.distToDropoff = dropoffEntry.distToPDLoc;
//                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
//                requestState.tryAssignment(asgn);
//                ++numAssignmentsTriedWithOrdinaryDropoff;
//            }
//
//            return numAssignmentsTriedWithOrdinaryDropoff;
//        }
//
//
//        void findOrdinaryPairedAssignments() {
//
//            Timer timer;
//            int numAssignmentsTried = 0;
//
//            // Try pairs with pickup at existing stop
//            Assignment asgn;
//            const auto &minDirectDistance = requestState.minDirectPDDist;
//
//            unsigned int minPickupId = INVALID_ID, minDropoffId = INVALID_ID;
//            int minDistToPickup, minDistFromDropoff;
//            RelevantPDLocs::It pickupIt, dropoffIt;
//
//            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
//                if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
//                    continue;
//
//                const auto &veh = fleet[vehId];
//                const auto &stopLocations = routeState.stopLocationsFor(vehId);
//
//                asgn.vehicle = &veh;
//
//                const auto relevantPickups = relPickups.relevantSpotsFor(vehId);
//                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
//
//                pickupIt = relevantPickups.begin();
//                dropoffIt = relevantDropoffs.begin();
//                while (pickupIt < relevantPickups.end() && dropoffIt < relevantDropoffs.end()) {
//
//                    // Alternating sweep over pickups and dropoffs which pause once they meet or pass the other sweep.
//                    while (pickupIt < relevantPickups.end() &&
//                           routeState.stopPositionOf(pickupIt->stopId) < routeState.stopPositionOf(dropoffIt->stopId))
//                        ++pickupIt;
//                    if (pickupIt == relevantPickups.end())
//                        break;
//                    while (dropoffIt < relevantDropoffs.end() &&
//                           routeState.stopPositionOf(dropoffIt->stopId) < routeState.stopPositionOf(pickupIt->stopId))
//                        ++dropoffIt;
//                    if (dropoffIt == relevantDropoffs.end())
//                        break;
//
//                    // If both sweeps paused at the same stopIndex, there are pickups and dropoffs at this stop.
//                    // We attempt a paired assignment.
//                    if (routeState.stopPositionOf(pickupIt->stopId) == routeState.stopPositionOf(dropoffIt->stopId)) {
//                        const auto stopPos = routeState.stopPositionOf(pickupIt->stopId);
//                        asgn.pickupStopIdx = stopPos;
//                        asgn.dropoffStopIdx = stopPos;
//
//                        if (routeState.occupanciesFor(vehId)[stopPos] + requestState.originalRequest.numRiders >
//                            veh.capacity) {
//                            continue;
//                        }
//
//                        const auto beginOfStopInPickups = pickupIt;
//                        const auto beginOfStopInDropoffs = dropoffIt;
//
//                        // Iterate over all pickups/dropoffs at this stop once to find a lower bound on the cost of any
//                        // paired assignment here
//                        minDistToPickup = INFTY;
//                        minDistFromDropoff = INFTY;
//
//                        while (pickupIt < relevantPickups.end() &&
//                               routeState.stopPositionOf(pickupIt->stopId) == stopPos) {
//                            const auto &entry = *pickupIt;
//                            if (entry.distToPDLoc < minDistToPickup) {
//                                minDistToPickup = entry.distToPDLoc;
//                                minPickupId = entry.pdId;
//                            }
//                            ++pickupIt;
//                        }
//
//                        while (dropoffIt < relevantDropoffs.end() &&
//                               routeState.stopPositionOf(dropoffIt->stopId) == stopPos) {
//                            const auto &entry = *dropoffIt;
//                            if (entry.distFromPDLocToNextStop < minDistFromDropoff) {
//                                minDistFromDropoff = entry.distFromPDLocToNextStop;
//                                minDropoffId = entry.pdId;
//                            }
//                            ++dropoffIt;
//                        }
//
//
//                        if (minDistToPickup == INFTY || minDistFromDropoff == INFTY) {
//                            continue;
//                        }
//
//                        const auto endOfStopInPickups = pickupIt;
//                        const auto endOfStopInDropoffs = dropoffIt;
//
////                        // With collected lower bounds, we check whether an assignment better than the best known is possible with this vehicle
//                        asgn.pickup = &requestState.pickups[minPickupId];
//                        asgn.dropoff = &requestState.dropoffs[minDropoffId];
//                        asgn.distToPickup = minDistToPickup;
//                        asgn.distToDropoff = minDirectDistance;
//                        asgn.distFromDropoff = minDistFromDropoff;
//                        const auto lowerBoundCost =
//                                calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState);
//
//                        if (lowerBoundCost > requestState.getBestCost())
//                            continue;
//
//
//                        // Try paired assignment for every combination of relevant pickup and dropoff
//                        for (auto dropoffIt2 = beginOfStopInDropoffs; dropoffIt2 < endOfStopInDropoffs; ++dropoffIt2) {
//                            const auto &dropoffEntry = *dropoffIt2;
//                            asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];
//
//                            if (stopLocations[stopPos + 1] == asgn.dropoff->loc)
//                                continue; // if dropoff coincides with the following stop, an ordinary non-paired assignment with dropoffIndex = pickupIndex + 1 will cover this case
//
//                            asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
//                            for (auto pickupIt2 = beginOfStopInPickups; pickupIt2 < endOfStopInPickups; ++pickupIt2) {
//                                const auto &pickupEntry = *pickupIt2;
//                                asgn.pickup = &requestState.pickups[pickupEntry.pdId];
//                                asgn.distToPickup = pickupEntry.distToPDLoc;
//
//                                assert(asgn.distToPickup < INFTY && asgn.distFromDropoff < INFTY);
//                                asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
//                                requestState.tryAssignment(asgn);
//                                ++numAssignmentsTried;
//                            }
//                        }
//                    }
//                }
//
//            }
//
//            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
//            requestState.stats().ordAssignmentsStats.tryPairedAssignmentsTime += pairedTime;
//            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
//        }

        const RelevantPDLocs &relPickups;
        const RelevantPDLocs &relDropoffs;
        const PDDistancesT &pdDistances;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;

        tbb::enumerable_thread_specific<ThreadLocalBestAsgnInfo> localBestAsgnInfo;
    };
}