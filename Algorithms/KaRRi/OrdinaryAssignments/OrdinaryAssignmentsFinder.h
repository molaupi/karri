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

        void findAssignments(const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs,
                             const PDDistancesT& pdDistances) {
            findOrdinaryAssignments(relPickups, relDropoffs);
            findOrdinaryPairedAssignments(pdDistances, relPickups, relDropoffs);
        }

        void init() {
            // no op
        }

    private:

        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void findOrdinaryAssignments(const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs) {

            Timer timer;
            int numCandidateVehicles = 0;
            int numAssignmentsTried = 0;

            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
                    continue;

                if (!relPickups.hasRelevantSpotsFor(vehId) ||
                    !relDropoffs.hasRelevantSpotsFor(vehId))
                    continue;

                ++numCandidateVehicles;
                Assignment asgn;
                asgn.legs.emplace_back();
                asgn.legs.back().vehicle = &fleet[vehId];

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
                    asgn.legs.back().pickupStopIdx = pickupEntry.stopIndex;
                    asgn.legs.back().travelTimeToPickup = pickupEntry.distToPDLoc;
                    asgn.legs.back().detourCostToPickup = pickupEntry.distToPDLoc;
                    asgn.legs.back().travelTimeFromPickup = pickupEntry.distFromPDLocToNextStop;
                    asgn.legs.back().detourCostFromPickup = pickupEntry.distFromPDLocToNextStop;

                    numAssignmentsTried += tryDropoffLaterThanPickup(asgn, curFirstDropoffIt, relDropoffs);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.tryNonPairedAssignmentsTime += time;
            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().ordAssignmentsStats.numCandidateVehicles += numCandidateVehicles;

        }

        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
        // completes the assignment with those dropoffs, and tries the resulting assignments.
        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
        int tryDropoffLaterThanPickup(Assignment &asgn,
                                      const RelevantPDLocs::It &startItInRegularDropoffs,
                                      const RelevantPDLocs& relDropoffs) {
            KASSERT(asgn.pickup && asgn.legs.size() == 1 && asgn.legs.back().vehicle);
            const auto &vehId = asgn.legs.back().vehicle->vehicleId;

            const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
            assert(startItInRegularDropoffs >= relevantDropoffs.begin() &&
                   startItInRegularDropoffs <= relevantDropoffs.end());

            if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
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

                asgn.legs.back().dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.legs.back().travelTimeToDropoff = dropoffEntry.distToPDLoc;
                asgn.legs.back().detourCostToDropoff = dropoffEntry.distToPDLoc;
                asgn.legs.back().travelTimeFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                asgn.legs.back().detourCostFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                requestState.tryAssignment(asgn);
                ++numAssignmentsTriedWithOrdinaryDropoff;
            }

            return numAssignmentsTriedWithOrdinaryDropoff;
        }


        void findOrdinaryPairedAssignments(const PDDistancesT& pdDistances, const RelevantPDLocs& relPickups, const RelevantPDLocs& relDropoffs) {

            Timer timer;
            int numAssignmentsTried = 0;

            // Try pairs with pickup at existing stop
            Assignment asgn;
            asgn.legs.emplace_back();
            const auto &minDirectDistance = requestState.minDirectPDDist;

            unsigned int minPickupId = INVALID_ID, minDropoffId = INVALID_ID;
            int minDistToPickup, minDistFromDropoff;
            RelevantPDLocs::It pickupIt, dropoffIt;
            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
                    continue;

                const auto &veh = fleet[vehId];
                const auto &stopLocations = routeState.stopLocationsFor(vehId);

                asgn.legs.back().vehicle = &veh;

                const auto relevantPickups = relPickups.relevantSpotsFor(vehId);
                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);

                pickupIt = relevantPickups.begin();
                dropoffIt = relevantDropoffs.begin();
                while (pickupIt < relevantPickups.end() && dropoffIt < relevantDropoffs.end()) {

                    // Alternating sweep over pickups and dropoffs which pause once they meet or pass the other sweep.
                    while (pickupIt < relevantPickups.end() && pickupIt->stopIndex < dropoffIt->stopIndex)
                        ++pickupIt;
                    if (pickupIt == relevantPickups.end())
                        break;
                    while (dropoffIt < relevantDropoffs.end() && dropoffIt->stopIndex < pickupIt->stopIndex)
                        ++dropoffIt;
                    if (dropoffIt == relevantDropoffs.end())
                        break;

                    // If both sweeps paused at the same stopIndex, there are pickups and dropoffs at this stop.
                    // We attempt a paired assignment.
                    if (pickupIt->stopIndex == dropoffIt->stopIndex) {
                        const auto stopPos = pickupIt->stopIndex;

                        if (routeState.occupanciesFor(vehId)[stopPos] + requestState.originalRequest.numRiders > veh.capacity) {
                            continue;
                        }

                        const auto beginOfStopInPickups = pickupIt;
                        const auto beginOfStopInDropoffs = dropoffIt;

                        // Iterate over all pickups/dropoffs at this stop once to find a lower bound on the cost of any
                        // paired assignment here
                        minDistToPickup = INFTY;
                        minDistFromDropoff = INFTY;

                        while (pickupIt < relevantPickups.end() && pickupIt->stopIndex == stopPos) {
                            const auto &entry = *pickupIt;
                            if (entry.distToPDLoc < minDistToPickup) {
                                minDistToPickup = entry.distToPDLoc;
                                minPickupId = entry.pdId;
                            }
                            ++pickupIt;
                        }

                        while (dropoffIt < relevantDropoffs.end() && dropoffIt->stopIndex == stopPos) {
                            const auto &entry = *dropoffIt;
                            if (entry.distFromPDLocToNextStop < minDistFromDropoff) {
                                minDistFromDropoff = entry.distFromPDLocToNextStop;
                                minDropoffId = entry.pdId;
                            }
                            ++dropoffIt;
                        }

                        if (minDistToPickup == INFTY || minDistFromDropoff == INFTY)
                            continue;

                        const auto endOfStopInPickups = pickupIt;
                        const auto endOfStopInDropoffs = dropoffIt;

                        // With collected lower bounds, we check whether an assignment better than the best known is possible with this vehicle
                        asgn.pickup = &requestState.pickups[minPickupId];
                        asgn.dropoff = &requestState.dropoffs[minDropoffId];
                        asgn.legs.back().pickupStopIdx = stopPos;
                        asgn.legs.back().dropoffStopIdx = stopPos;
                        asgn.legs.back().travelTimeToPickup = minDistToPickup;
                        asgn.legs.back().detourCostToPickup = minDistToPickup;
                        asgn.legs.back().travelTimeToDropoff = minDirectDistance;
                        asgn.legs.back().detourCostToDropoff = minDirectDistance;
                        asgn.legs.back().travelTimeFromDropoff = minDistFromDropoff;
                        asgn.legs.back().detourCostFromDropoff = minDistFromDropoff;
                        const auto lowerBoundCost =
                                calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState);
                        if (lowerBoundCost > requestState.getBestCost())
                            continue;


                        // Try paired assignment for every combination of relevant pickup and dropoff
                        for (auto dropoffIt2 = beginOfStopInDropoffs; dropoffIt2 < endOfStopInDropoffs; ++dropoffIt2) {
                            const auto &dropoffEntry = *dropoffIt2;
                            asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                            if (stopLocations[stopPos + 1] == asgn.dropoff->loc)
                                continue; // if dropoff coincides with the following stop, an ordinary non-paired assignment with dropoffIndex = pickupIndex + 1 will cover this case

                            asgn.legs.back().travelTimeFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                            asgn.legs.back().detourCostFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                            for (auto pickupIt2 = beginOfStopInPickups; pickupIt2 < endOfStopInPickups; ++pickupIt2) {
                                const auto &pickupEntry = *pickupIt2;
                                asgn.pickup = &requestState.pickups[pickupEntry.pdId];
                                asgn.legs.back().travelTimeToPickup = pickupEntry.distToPDLoc;
                                asgn.legs.back().detourCostToPickup = pickupEntry.distToPDLoc;

                                KASSERT(asgn.legs.back().travelTimeToPickup < INFTY && asgn.legs.back().travelTimeFromDropoff < INFTY);
                                asgn.legs.back().travelTimeToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                                asgn.legs.back().detourCostToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                                requestState.tryAssignment(asgn);
                                ++numAssignmentsTried;
                            }
                        }
                    }
                }

            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.tryPairedAssignmentsTime += pairedTime;
            requestState.stats().ordAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
        }

        const Fleet &fleet;
        const CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;
    };
}