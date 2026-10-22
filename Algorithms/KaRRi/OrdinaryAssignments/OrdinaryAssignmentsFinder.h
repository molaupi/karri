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
#include "Algorithms/KaRRi/EllipticBCH/FeasiblePDLocs.h"
#include "Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"

namespace karri {

// Finds ordinary assignments, i.e. those assignments where pickup and dropoff are both inserted after the vehicle's
// next stop but before the vehicle's last stop. This includes ordinary paired assignments where the pickup and dropoff
// are inserted between the same pair of existing stops.
//
// Non-paired assignments are based on filtered relevant PD locs. Paired assignments are based on the feasible PD locs
// since the Pareto-based filtering of relevant PD locs does not apply to paired assignments.
    template<typename PDDistancesT>
    class OrdinaryAssignmentsFinder {


    public:

        OrdinaryAssignmentsFinder(const Fleet &fleet, const RouteState &routeState)
                : fleet(fleet),
                  calculator(routeState),
                  routeState(routeState) {}

        void findAssignments(const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs,
                             const FeasiblePDLocs& feasiblePickups, const FeasiblePDLocs& feasibleDropoffs,
                             RequestState& requestState,
                             const PDDistancesT& pdDistances,
                             const PDLocs& pdLocs, stats::OrdAssignmentsPerformanceStats& stats) {
            findOrdinaryAssignments(relPickups, relDropoffs, requestState, pdLocs, stats);
            findOrdinaryPairedAssignments(feasiblePickups, feasibleDropoffs, pdDistances, requestState, pdLocs, stats);
        }

        void init(const RequestState&, const PDLocs&, stats::OrdAssignmentsPerformanceStats&) {
            // no op
        }

    private:

        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void findOrdinaryAssignments(const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs, RequestState& requestState, const PDLocs& pdLocs,
                                     stats::OrdAssignmentsPerformanceStats& stats) {

            Timer timer;
            int numCandidateVehicles = 0;
            int numAssignmentsTried = 0;

            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.hasRelevantSpotsFor(vehId))
                    continue;

                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
//                if (!relPickups.hasRelevantSpotsFor(vehId) ||
//                    !relDropoffs.hasRelevantSpotsFor(vehId))
//                    continue;

                ++numCandidateVehicles;
                Assignment asgn(&fleet[vehId]);

                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
                auto curFirstDropoffIt = relevantDropoffs.begin();

                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {

                    // Find first stop position after the pickup's stop position that has relevant dropoffs.
                    const auto &stopPos = pickupEntry.stopIndex;
                    while (curFirstDropoffIt < relevantDropoffs.end() &&
                           curFirstDropoffIt->stopIndex <= stopPos) {
                        ++curFirstDropoffIt;
                    }
                    if (curFirstDropoffIt == relevantDropoffs.end())
                        break; // No dropoffs later in route than current (or subsequent) pickup(s)

                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupEntry.stopIndex;
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;

                    numAssignmentsTried += tryDropoffLaterThanPickup(asgn, curFirstDropoffIt, relDropoffs, requestState, pdLocs);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryNonPairedAssignmentsTime += time;
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.numCandidateVehicles += numCandidateVehicles;

        }

        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
        // completes the assignment with those dropoffs, and tries the resulting assignments.
        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
        int tryDropoffLaterThanPickup(Assignment &asgn,
                                      const RelevantPDLocs::It &startItInRegularDropoffs,
                                      const RelevantPDLocs& relDropoffs,
                                      RequestState& requestState,
                                      const PDLocs& pdLocs) {
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto &vehId = asgn.vehicle->vehicleId;

            const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
            assert(startItInRegularDropoffs >= relevantDropoffs.begin() &&
                   startItInRegularDropoffs <= relevantDropoffs.end());

            if (!relDropoffs.hasRelevantSpotsFor(vehId))
                return 0;

            auto numAssignmentsTriedWithOrdinaryDropoff = 0;

            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            const auto &occupancies = routeState.occupanciesFor(vehId);

            // Compute smallest stop index after pickup at which capacity of vehicle would be broken (end of route
            // if never broken). Dropoff has to be made before or at this index.
            int capacityBrokenIndex = asgn.pickupStopIdx;
            while (capacityBrokenIndex < numStops &&
                   occupancies[capacityBrokenIndex] + requestState.originalRequest.numRiders <= asgn.vehicle->capacity)
                ++capacityBrokenIndex;

            for (auto dropoffIt = startItInRegularDropoffs; dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;

                // All remaining dropoffs would be after the stop where capacity is broken. (Dropoff may still be made
                // at an existing stop at capacityBrokenIndex.)
                if (dropoffEntry.stopIndex > capacityBrokenIndex)
                    break;

                asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                if (dropoffEntry.stopIndex + 1 < numStops &&
                    stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff.loc) {
                    // If the dropoff is at the location of the following stop, do not try an assignment here as it would
                    // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                    // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                    continue;
                }

                if (asgn.dropoff.loc == asgn.pickup.loc) {
                    // In this case, this spot is the best spot at or after pickupIndex and the best spot at or after
                    // dropoffIndex. We ignore this case here since inserting them paired into the same leg will be better.
                    continue;
                }

                asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                ++numAssignmentsTriedWithOrdinaryDropoff;
            }

            return numAssignmentsTriedWithOrdinaryDropoff;
        }


        void findOrdinaryPairedAssignments(const FeasiblePDLocs &feasiblePickups,
                                           const FeasiblePDLocs &feasibleDropoffs,
                                           const PDDistancesT &pdDistances,
                                           RequestState &requestState, const PDLocs &pdLocs,
                                           stats::OrdAssignmentsPerformanceStats &stats) {

            Timer timer;
            int numAssignmentsTried = 0;

            Assignment asgn;
            const auto &minDirectDistance = requestState.minDirectPDDist;

            for (const auto &stopId: feasiblePickups.getStopIdsWithFeasiblePDLocs()) {
                if (!feasibleDropoffs.hasFeasiblePDLocs(stopId))
                    continue;
                const int vehId = routeState.vehicleIdOf(stopId);
                const int stopPos = routeState.stopPositionOf(stopId);
                const auto &veh = fleet[vehId];

                if (stopPos == 0 || stopPos == routeState.numStopsOf(vehId) - 1)
                    continue;

                if (routeState.occupanciesFor(vehId)[stopPos] + requestState.originalRequest.numRiders > veh.capacity)
                    continue;

                const auto nextStopLoc = routeState.stopLocationsFor(vehId)[stopPos + 1];
                asgn.vehicle = &veh;
                asgn.pickupStopIdx = stopPos;
                asgn.dropoffStopIdx = stopPos;

                // With lower bounds on the distances, we check whether an assignment better than the best known is
                // possible at this stop.
                const auto &feasiblePickupsAtStop = feasiblePickups.feasiblePdLocsFor(stopId);
                const auto &feasibleDropoffsAtStop = feasibleDropoffs.feasiblePdLocsFor(stopId);
                asgn.pickup = pdLocs.pickups[feasiblePickupsAtStop.begin()->pdId];
                asgn.dropoff = pdLocs.dropoffs[feasibleDropoffsAtStop.begin()->pdId];
                asgn.distToPickup = feasiblePickups.minDistToPDLocFor(stopId);
                asgn.distToDropoff = minDirectDistance;
                const int minDistFromDropoff = feasibleDropoffs.minDistFromPDLocToNextStopOf(stopId);
                asgn.distFromDropoff = minDistFromDropoff;
                if (calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState) >
                    requestState.getBestCost())
                    continue;

                // Filter feasible pickups and dropoffs at this stop using cost lower bounds. (Pareto-based
                // filtering does not apply to paired assignments.)
                pickupsForPaired.clear();
                for (const auto &entry: feasiblePickupsAtStop) {
                    const auto &p = pdLocs.pickups[entry.pdId];
                    if (calculator.calcMinCostForPickupBetweenStops(veh, stopPos, p, entry.distToPDLoc,
                                                                    entry.distFromPDLocToNextStop, requestState) >
                        requestState.getBestCost())
                        continue;
                    // Lower bound for this pickup with any dropoff
                    asgn.pickup = p;
                    asgn.distToPickup = entry.distToPDLoc;
                    asgn.distToDropoff = pdDistances.getMinDirectDistanceForPickup(entry.pdId);
                    asgn.distFromDropoff = minDistFromDropoff;
                    if (calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState) >
                        requestState.getBestCost())
                        continue;
                    pickupsForPaired.push_back(entry);
                }
                if (pickupsForPaired.empty())
                    continue;

                dropoffsForPaired.clear();
                for (const auto &entry: feasibleDropoffsAtStop) {
                    const auto &d = pdLocs.dropoffs[entry.pdId];
                    // If dropoff coincides with the following stop, an ordinary non-paired assignment with
                    // dropoffIndex = pickupIndex + 1 will cover this case
                    if (nextStopLoc == d.loc)
                        continue;
                    const bool atStop = d.loc == routeState.stopLocationsFor(vehId)[stopPos];
                    if (calculator.calcMinCostForDropoffBetweenStops(veh, stopPos, d, entry.distToPDLoc,
                                                                     entry.distFromPDLocToNextStop, atStop,
                                                                     requestState) > requestState.getBestCost())
                        continue;
                    dropoffsForPaired.push_back(entry);
                }

                // Try paired assignment for every combination of relevant pickup and dropoff
                for (const auto &pickupEntry: pickupsForPaired) {
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.distToPickup = pickupEntry.distToPDLoc;

                    for (const auto &dropoffEntry: dropoffsForPaired) {
                        asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];
                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                        KASSERT(asgn.distToPickup < INFTY && asgn.distFromDropoff < INFTY);
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                        requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        ++numAssignmentsTried;
                    }
                }
            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryPairedAssignmentsTime += pairedTime;
            stats.numAssignmentsTried += numAssignmentsTried;
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;

        std::vector<FeasiblePDLocs::FeasiblePDLoc> pickupsForPaired;
        std::vector<FeasiblePDLocs::FeasiblePDLoc> dropoffsForPaired;
    };
}