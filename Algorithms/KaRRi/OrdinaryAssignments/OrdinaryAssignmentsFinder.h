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


    public:

        OrdinaryAssignmentsFinder(const Fleet &fleet, const CostCalculator &calculator, const RouteState &routeState,
                                  RequestState &requestState)
                : fleet(fleet),
                  calculator(calculator),
                  routeState(routeState),
                  requestState(requestState) {}

//        void findAssignments(const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs,
//                             const PDDistancesT& pdDistances) {
//            findOrdinaryAssignments(relPickups, relDropoffs);
//            findOrdinaryPairedAssignments(pdDistances, relPickups, relDropoffs);
//        }

        void init() {
            // no op
        }

        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void findOrdinaryAssignments(const RelevantPDLocs &relPickups, const RelevantPDLocs &relDropoffs) {

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

                    asgn.pickup = &requestState.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupEntry.stopIndex;
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;

                    numAssignmentsTried += tryDropoffLaterThanPickup(asgn, curFirstDropoffIt, relDropoffs);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.tryNonPairedAssignmentsTime += time;
            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().ordAssignmentsStats.numCandidateVehicles += numCandidateVehicles;

        }

        template<typename FeasibleEllipticDistancesT>
        void findOrdinaryPairedAssignments(const FeasibleEllipticDistancesT &feasiblePickups,
                                           const FeasibleEllipticDistancesT &feasibleDropoffs,
                                           const PDDistancesT &pdDistances) {

            Timer timer;
            int numAssignmentsTried = 0;

            // Try pairs with pickup at existing stop
            Assignment asgn;
            const auto &minDirectDistance = requestState.minDirectPDDist;

            // Find stops that can have a paired assignment by intersecting stops with feasible pickups and stops
            // with feasible dropoffs
            auto stopsWithFeasiblePickups = feasiblePickups.getStopIdsWithRelevantPDLocs();
            auto stopsWithFeasibleDropoffs = feasibleDropoffs.getStopIdsWithRelevantPDLocs();
            std::sort(stopsWithFeasiblePickups.begin(), stopsWithFeasiblePickups.end());
            std::sort(stopsWithFeasibleDropoffs.begin(), stopsWithFeasibleDropoffs.end());
            std::vector<int> stopsWithPossiblePaired;
            std::set_intersection(stopsWithFeasiblePickups.begin(), stopsWithFeasiblePickups.end(),
                                  stopsWithFeasibleDropoffs.begin(), stopsWithFeasibleDropoffs.end(),
                                  std::back_inserter(stopsWithPossiblePaired));

            struct RelAtStop {
                int pdLocId = INVALID_ID;
                int dist = INFTY;
            };
            std::vector<RelAtStop> relPickups, relDropoffs;
            relPickups.reserve(requestState.numPickups());
            relDropoffs.reserve(requestState.numDropoffs());

            for (const auto &stopId: stopsWithPossiblePaired) {
                KASSERT(feasiblePickups.hasPotentiallyRelevantPDLocs(stopId) &&
                        feasibleDropoffs.hasPotentiallyRelevantPDLocs(stopId));
                const auto stopPos = routeState.stopPositionOf(stopId);
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto &veh = fleet[vehId];
                if (stopPos == 0 || stopPos == routeState.numStopsOf(vehId) - 1)
                    continue; // handled by PBNSAssignmentFinder or ordinary assignments

                // If capacity does not allow a pickup and dropoff at this stop, skip
                if (routeState.occupanciesFor(vehId)[stopPos] + requestState.originalRequest.numRiders > veh.capacity)
                    continue;

                asgn.vehicle = &veh;
                asgn.pickupStopIdx = stopPos;
                asgn.dropoffStopIdx = stopPos;

                // Find lower bound for cost of paired insertion at this stop. If lower bound is worse than best
                // known cost, skip the stop.
                const auto minDistToAnyPickup = feasiblePickups.minDistToRelevantPDLocsFor(stopId);
                const auto minDistFromAnyDropoff = feasibleDropoffs.minDistFromPDLocToNextStopOf(stopId);
                asgn.distToPickup = minDistToAnyPickup;
                asgn.distToDropoff = minDirectDistance;
                asgn.distFromDropoff = minDistFromAnyDropoff;
                const auto lowerBoundCost =
                        calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState);
                if (lowerBoundCost > requestState.getBestCost())
                    continue;

                const auto distToPickups = feasiblePickups.distancesToRelevantPDLocsFor(stopId);
                const auto distFromDropoffs = feasibleDropoffs.distancesFromRelevantPDLocsToNextStopOf(stopId);
                using namespace time_utils;
                const auto vehDepTimeAtPrevStop = getVehDepTimeAtStopForRequest(vehId, stopPos, requestState, routeState);
                const auto legLength = calcLengthOfLegStartingAt(stopPos, vehId, routeState);

                // Filter all pickups and dropoffs that cannot be relevant even by themselves
                relPickups.clear();
                relDropoffs.clear();

                for (auto pickupId = 0; pickupId < requestState.numPickups(); ++pickupId) {
                    const auto dist = distToPickups[pickupId];
                    if (dist >= INFTY)
                        continue;
                    const auto& p = requestState.pickups[pickupId];
                    const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopPos, dist, p,
                                                                          requestState, routeState);
                    const auto minDetour = std::max(depTimeAtPickup - vehDepTimeAtPrevStop + pdDistances.getMinDirectDistanceForPickup(pickupId) + minDistFromAnyDropoff - legLength, 0);
                    if (doesPickupDetourViolateHardConstraints(veh, requestState, stopPos, minDetour, routeState))
                        continue;

                    const int curKnownCost = calculator.calcMinKnownPickupSideCost(veh, stopPos, minDetour,
                                                                                   p.walkingDist, depTimeAtPickup,
                                                                                   requestState.minDirectPDDist,
                                                                                   requestState);

                    // If cost for only pickup side is already worse than best known cost for a whole assignment, then
                    // this pickup is not relevant at this stop.
                    if (curKnownCost > requestState.getBestCost())
                        continue;

                    relPickups.push_back({pickupId, dist});
                }

                const auto stopLocOfFollowing = routeState.stopLocationsFor(vehId)[stopPos + 1];
                for (auto dropoffId = 0; dropoffId < requestState.numDropoffs(); ++dropoffId) {
                    const auto dist = distFromDropoffs[dropoffId];
                    if (dist >= INFTY)
                        continue;
                    const auto& d = requestState.dropoffs[dropoffId];

                    if (stopLocOfFollowing == d.loc)
                        continue; // if dropoff coincides with the following stop, an ordinary non-paired assignment with dropoffIndex = pickupIndex + 1 will cover this case


                    const auto minDetour = std::max(minDistToAnyPickup + pdDistances.getMinDirectDistanceForDropoff(dropoffId) + minDistFromAnyDropoff - legLength, 0);
                    if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopPos,
                                                                                  minDetour, routeState))
                        continue;

                    const int curMinCost = calculator.calcMinKnownDropoffSideCost(veh, stopPos, minDetour,
                                                                                  d.walkingDist, requestState.minDirectPDDist, requestState);

                    // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
                    // this dropoff is not relevant at this stop.
                    if (curMinCost > requestState.getBestCost())
                        continue;

                    relDropoffs.push_back({dropoffId, dist});
                }


                // Try paired assignment for every combination of relevant pickup and dropoff
                for (const auto& relDropoff : relDropoffs) {
                    asgn.dropoff = &requestState.dropoffs[relDropoff.pdLocId];
                    asgn.distFromDropoff = relDropoff.dist;
                    KASSERT(asgn.distFromDropoff < INFTY);

                    for (const auto& relPickup : relPickups) {
                        asgn.pickup = &requestState.pickups[relPickup.pdLocId];
                        asgn.distToPickup = relPickup.dist;
                        KASSERT(asgn.distToPickup < INFTY);

                        asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        requestState.tryAssignment(asgn);
                        ++numAssignmentsTried;
                    }
                }
            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.tryPairedAssignmentsTime += pairedTime;
            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
        }

    private:


        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
        // completes the assignment with those dropoffs, and tries the resulting assignments.
        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
        int tryDropoffLaterThanPickup(Assignment &asgn,
                                      const RelevantPDLocs::It &startItInRegularDropoffs,
                                      const RelevantPDLocs &relDropoffs) {
            assert(asgn.vehicle && asgn.pickup);
            const auto &vehId = asgn.vehicle->vehicleId;

            const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
            assert(startItInRegularDropoffs >= relevantDropoffs.begin() &&
                   startItInRegularDropoffs <= relevantDropoffs.end());

            if (!relDropoffs.hasRelevantSpotsFor(vehId))
                return 0;

            auto numAssignmentsTriedWithOrdinaryDropoff = 0;

            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            for (auto dropoffIt = startItInRegularDropoffs; dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;
                asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                if (dropoffEntry.stopIndex + 1 < numStops &&
                    stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff->loc) {
                    // If the dropoff is at the location of the following stop, do not try an assignment here as it would
                    // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                    // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                    continue;
                }

                if (asgn.dropoff->loc == asgn.pickup->loc) {
                    // In this case, this spot is the best spot at or after pickupIndex and the best spot at or after
                    // dropoffIndex. We ignore this case here since inserting them paired into the same leg will be better.
                    continue;
                }

                asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                requestState.tryAssignment(asgn);
                ++numAssignmentsTriedWithOrdinaryDropoff;
            }

            return numAssignmentsTriedWithOrdinaryDropoff;
        }

        const Fleet &fleet;
        const CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;
    };
}