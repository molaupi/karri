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
#include "Algorithms/KaRRi/PbnsAssignments/CurVehLocToPickupSearches.h"

namespace karri {


// Finds pickup before next stop assignments, i.e. assignments where the pickup is inserted before the vehicle's next
// stop, which means the vehicle is rerouted at its current location. Dropoff may be inserted before the next stop or
// at ordinary locations, i.e. after the next stop but before the last stop.
// (The case of pickup before next stop and dropoff after last stop is considered by the DALSAssignmentsFinder.)
//
// Works based on filtered relevant pickups and dropoffs before next stop as well as relevant ordinary dropoffs.
    template<typename PDDistancesT, typename CurVehLocToPickupSearchesT>
    class PBNSAssignmentsFinder {

        // Algorithm iterates through combinations of pickups and dropoffs and filters based on lower bound cost.
        // If exact distance from vehicle's current location to a pickup is ever needed, we delay the computation of that
        // distance, so it can be bundled with other such computations. In this case, a continuation marks the combination
        // of pickup and dropoff where the exact distance was first needed for this pickup, so the iteration of
        // combinations can continue after the bundled computation of exact distances.
        struct Continuation {
            int pickupId = INVALID_ID;
            int distFromPickup = INFTY;
            RelevantPDLocs::It continueIt;
        };

    public:

        PBNSAssignmentsFinder(CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                              const Fleet &fleet, const RouteState &routeState)
                : curVehLocToPickupSearches(curVehLocToPickupSearches),
                  fleet(fleet),
                  calculator(routeState),
                  routeState(routeState) {}

        void findAssignments(const RelevantPDLocs &relPickupsBns, const RelevantPDLocs &relOrdinaryDropoffs,
                             const RelevantPDLocs &relDropoffsBns,
                             RequestState& requestState,
                             const PDDistancesT& pdDistances,
                             const PDLocs& pdLocs, stats::PbnsAssignmentsPerformanceStats& stats) {
            numAssignmentsTriedWithPickupBeforeNextStop = 0;
            Timer timer;

            int numCandidateVehicles = 0;
            for (const auto &vehId: relPickupsBns.getVehiclesWithRelevantPDLocs()) {

                if (!relOrdinaryDropoffs.hasRelevantSpotsFor(vehId) && !relDropoffsBns.hasRelevantSpotsFor(vehId))
                    continue;
                ++numCandidateVehicles;


                ordinaryContinuations.clear();
                pairedContinuations.clear();

                assert(routeState.occupanciesFor(vehId)[0]  + requestState.originalRequest.numRiders <= fleet[vehId].capacity);

                determineNecessaryExactDistances(fleet[vehId], relPickupsBns, relOrdinaryDropoffs, relDropoffsBns, requestState, pdDistances, pdLocs);

                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);

                finishContinuations(fleet[vehId], relOrdinaryDropoffs, relDropoffsBns, requestState, pdDistances, pdLocs);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>() -
                              curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            stats.tryAssignmentsTime += time;
            stats.numCandidateVehicles += numCandidateVehicles;
            stats.numAssignmentsTried += numAssignmentsTriedWithPickupBeforeNextStop;

            stats.locatingVehiclesTime += curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            stats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            stats.directCHSearchTime += curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
        }

        // Initialize for new request.
        void init(const RequestState& requestState, const PDLocs& pdLocs, stats::PbnsAssignmentsPerformanceStats& stats) {
            Timer timer;
            curVehLocToPickupSearches.initialize(requestState.dispatchingTime, pdLocs);
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;
        }

    private:


        // Filters combinations of pickups and dropoffs using a cost lower bound.
        // If a combination is found for a pickup that cannot be filtered, we need the exact distance from the vehicle
        // location to the pickup.
        // These pickups are added to the queue of curVehLocToPickupSearches and continuations are stored to restart
        // the iteration of combinations for that pickup after the computation of exact distances.
        void determineNecessaryExactDistances(const Vehicle &veh, const RelevantPDLocs &relPickupsBns,
                                              const RelevantPDLocs &relOrdinaryDropoffs,
                                              const RelevantPDLocs &relDropoffsBns,
                                              RequestState& requestState,
                                              const PDDistancesT& pdDistances,
                                              const PDLocs& pdLocs) {

            const auto &relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBns.relevantSpotsFor(veh.vehicleId);

            Assignment asgn(&veh);

            for (const auto &entry: relPickupsBns.relevantSpotsFor(veh.vehicleId)) {
                asgn.pickup = pdLocs.pickups[entry.pdId];

                // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
                // vehicle's current location to the pickup => we get lower bound costs.
                asgn.distToPickup = entry.distToPDLoc;
                const int distFromPickup = entry.distFromPDLocToNextStop;

                // For paired assignments before next stop, first try a lower bound with the smallest direct PD distance
                const auto lowerBoundCostPairedAssignment = calculator.calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                        veh, asgn.pickup, asgn.distToPickup, requestState.minDirectPDDist,
                        distFromPickup, requestState);
                if (lowerBoundCostPairedAssignment < requestState.getBestCost()) {
                    const auto scannedUntil = tryLowerBoundsForPaired(asgn, relDropoffsBns, requestState, pdDistances, pdLocs);
                    if (scannedUntil < relDropoffsBeforeNextStopForVeh.end()) {
                        // In this case some paired assignment before the next stop needs the exact distance to pickup via
                        // the vehicle. Postpone computation of the yet unknown exact distance and the rest of the paired
                        // assignments as well as all assignments with later dropoffs. That way, the exact distances can be
                        // computed in a bundled fashion and the postponed assignments can use exact distances afterward.
                        curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                        pairedContinuations.push_back({asgn.pickup.id, 0, scannedUntil});
                        ++numAssignmentsTriedWithPickupBeforeNextStop; // Count first ordinary continuation
                        ordinaryContinuations.push_back(
                                {asgn.pickup.id, distFromPickup, relOrdinaryDropoffsForVeh.begin()});
                        continue; // Continue with next pickup, rest of assignments for this pickup later with exact distance
                    }
                }


                asgn.distFromPickup = distFromPickup;
                const auto scannedUntil = tryLowerBoundsForOrdinary(asgn, relOrdinaryDropoffs, requestState, pdLocs);

                if (scannedUntil < relOrdinaryDropoffsForVeh.end()) {
                    // In this case some assignment with the pickup before the next stop and an ordinary dropoff
                    // needs the exact distance to pickup via the vehicle. Postpone computation
                    // of the yet unknown exact distance and the rest of the assignments with later dropoffs. That way,
                    // the exact distances can be computed in a bundled fashion and the postponed assignments can use
                    // exact distances afterward.
                    curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                    ordinaryContinuations.push_back({asgn.pickup.id, distFromPickup, scannedUntil});
                }
            }
        }


        // Examines combinations of a given pickup and all dropoffs before the next stop of a given vehicle until a
        // paired assignment needs the exact distance to the pickup via the vehicle. Returns an iterator to the dropoff at
        // which the exact distance is first needed or one-past-end iterator if all combinations could be filtered.
        RelevantPDLocs::It tryLowerBoundsForPaired(Assignment &asgn, const RelevantPDLocs &relDropoffsBns, RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs) {
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto vehId = asgn.vehicle->vehicleId;


            const auto relevantDropoffs = relDropoffsBns.relevantSpotsFor(vehId);

            if (!relDropoffsBns.getVehiclesWithRelevantPDLocs().contains(vehId))
                return relevantDropoffs.end();

            const auto stopLocations = routeState.stopLocationsFor(vehId);

            asgn.distFromPickup = 0;
            asgn.dropoffStopIdx = 0;

            for (auto dropoffIt = relevantDropoffs.begin(); dropoffIt != relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;
                asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];
                if (stopLocations[1] == asgn.dropoff.loc)
                    continue;
                ++numAssignmentsTriedWithPickupBeforeNextStop;

                asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                const auto cost = calculator.calc(asgn, requestState);
                if (cost < requestState.getBestCost() || (cost == requestState.getBestCost() &&
                                                          breakCostTie(asgn, requestState.getBestAssignment()))) {
                    // Lower bound is better than best known cost => We need the exact distance to pickup.
                    // Return and postpone remaining combinations.
                    return dropoffIt;
                }
            }

            return relevantDropoffs.end();
        }

        // Examines combinations of a given pickup before the next stop and all relevant dropoffs after later stops of a given
        // vehicle until an assignment requires the exact distance to the pickup via the vehicle. Returns an iterator to the
        // dropoff at which the exact distance is first needed or one-past-end iterator if all combinations could be filtered.
        RelevantPDLocs::It tryLowerBoundsForOrdinary(Assignment &asgn, const RelevantPDLocs& relOrdinaryDropoffs, RequestState& requestState, const PDLocs& pdLocs) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto vehId = asgn.vehicle->vehicleId;

            const auto relevantDropoffs = relOrdinaryDropoffs.relevantSpotsFor(vehId);

            if (!relOrdinaryDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
                return relevantDropoffs.end();

            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);

            for (auto dropoffIt = relevantDropoffs.begin(); dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;
                asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                if (dropoffEntry.stopIndex + 1 < numStops &&
                    stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff.loc)
                    continue;
                if (asgn.dropoff.loc == asgn.pickup.loc)
                    continue;

                asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;

                ++numAssignmentsTriedWithPickupBeforeNextStop;

                const auto cost = calculator.calc(asgn, requestState);
                if (cost < requestState.getBestCost() || (cost == requestState.getBestCost() &&
                                                          breakCostTie(asgn, requestState.getBestAssignment()))) {
                    // Lower bound is better than best known cost => We need the exact distance to pickup.
                    // Return and postpone remaining combinations.
                    return dropoffIt;
                }
            }

            return relevantDropoffs.end();
        }

        void finishContinuations(const Vehicle &veh, const RelevantPDLocs& relOrdinaryDropoffs, const RelevantPDLocs& relDropoffsBns, RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs) {
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            Assignment asgn(&veh);

            // Finish all postponed assignments where dropoff is at stop >= 1.
            const auto relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
            for (const auto &continuation: ordinaryContinuations) {
                asgn.pickup = pdLocs.pickups[continuation.pickupId];

                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = continuation.distFromPickup;

                for (auto dropoffIt = continuation.continueIt;
                     dropoffIt < relOrdinaryDropoffsForVeh.end(); ++dropoffIt) {
                    const auto &dropoffEntry = *dropoffIt;
                    asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                    if (dropoffEntry.stopIndex + 1 < numStops &&
                        stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff.loc)
                        continue;
                    if (asgn.pickup.loc == asgn.dropoff.loc)
                        continue;

                    asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                    asgn.distToDropoff = dropoffEntry.distToPDLoc;
                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                    requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));

                    if (dropoffIt > continuation.continueIt) { // Do not count assignment at continuation twice
                        ++numAssignmentsTriedWithPickupBeforeNextStop;
                    }
                }
            }

            // Finish all paired assignments.
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBns.relevantSpotsFor(veh.vehicleId);
            for (const auto &continuation: pairedContinuations) {
                const auto pId = continuation.pickupId;
                asgn.pickup = pdLocs.pickups[pId];

                asgn.dropoffStopIdx = 0;
                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, pId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = 0;

                for (auto dropoffIt = continuation.continueIt;
                     dropoffIt < relDropoffsBeforeNextStopForVeh.end(); ++dropoffIt) {
                    const auto &dropoffEntry = *dropoffIt;
                    asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                    if (stopLocations[1] == asgn.dropoff.loc)
                        continue;

                    if (dropoffIt > continuation.continueIt) { // Do not count assignment at continuation twice
                        ++numAssignmentsTriedWithPickupBeforeNextStop;
                    }

                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                    if (asgn.distFromDropoff >= INFTY)
                        continue;

                    asgn.distToDropoff = pdDistances.getDirectDistance(pId, asgn.dropoff.id);
                    requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                }
            }
        }

        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;

        int numAssignmentsTriedWithPickupBeforeNextStop;
        std::vector<Continuation> ordinaryContinuations;
        std::vector<Continuation> pairedContinuations;

    };
}