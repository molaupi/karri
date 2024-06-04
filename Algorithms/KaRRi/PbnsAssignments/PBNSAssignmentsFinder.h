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

        PBNSAssignmentsFinder(const RelevantPDLocs &relPickupsBns, const RelevantPDLocs &relOrdinaryDropoffs,
                              const RelevantPDLocs &relDropoffsBns, const PDDistancesT &pdDistances,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                              const Fleet &fleet, const RequestState &requestState,
                              stats::PbnsAssignmentsPerformanceStats& stats)
                : relPickupsBNS(relPickupsBns),
                  relOrdinaryDropoffs(relOrdinaryDropoffs),
                  relDropoffsBNS(relDropoffsBns),
                  pdDistances(pdDistances),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  fleet(fleet),
                  requestState(requestState),
                  stats(stats) {}

        void findAssignments(BestAsgn& bestAsgn, const RouteStateData& routeState) {
            numAssignmentsTriedWithPickupBeforeNextStop = 0;
            Timer timer;

            int numCandidateVehicles = 0;
            for (const auto &vehId: relPickupsBNS.getVehiclesWithRelevantPDLocs()) {

                if (!relOrdinaryDropoffs.hasRelevantSpotsFor(vehId) && !relDropoffsBNS.hasRelevantSpotsFor(vehId))
                    continue;
                ++numCandidateVehicles;


                ordinaryContinuations.clear();
                pairedContinuations.clear();

                KASSERT(routeState.occupanciesFor(vehId)[0]  + requestState.originalRequest.numRiders <= fleet[vehId].capacity);

                determineNecessaryExactDistances(fleet[vehId], bestAsgn, routeState);

                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId]);

                finishContinuations(fleet[vehId], bestAsgn, routeState);
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
        void init(const RouteStateData&) {
            Timer timer;
            curVehLocToPickupSearches.initialize(requestState.originalRequest.requestTime);
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;
        }

    private:


        // Filters combinations of pickups and dropoffs using a cost lower bound.
        // If a combination is found for a pickup that cannot be filtered, we need the exact distance from the vehicle
        // location to the pickup.
        // These pickups are added to the queue of curVehLocToPickupSearches and continuations are stored to restart
        // the iteration of combinations for that pickup after the computation of exact distances.
        void determineNecessaryExactDistances(const Vehicle &veh, const BestAsgn& bestAsgn, const RouteStateData& routeState) {

            const auto &relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBNS.relevantSpotsFor(veh.vehicleId);

            Assignment asgn(&veh);

            for (const auto &entry: relPickupsBNS.relevantSpotsFor(veh.vehicleId)) {
                asgn.pickup = &requestState.pickups[entry.pdId];

                // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
                // vehicle's current location to the pickup => we get lower bound costs.
                asgn.distToPickup = entry.distToPDLoc;
                const int distFromPickup = entry.distFromPDLocToNextStop;

                // For paired assignments before next stop, first try a lower bound with the smallest direct PD distance
                const auto lowerBoundCostPairedAssignment = Calc::calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                        veh, *asgn.pickup, asgn.distToPickup, requestState.minDirectPDDist,
                        distFromPickup, requestState, routeState);
                if (lowerBoundCostPairedAssignment < bestAsgn.cost()) {
                    const auto scannedUntil = tryLowerBoundsForPaired(asgn, bestAsgn, routeState);
                    if (scannedUntil < relDropoffsBeforeNextStopForVeh.end()) {
                        // In this case some paired assignment before the next stop needs the exact distance to pickup via
                        // the vehicle. Postpone computation of the yet unknown exact distance and the rest of the paired
                        // assignments as well as all assignments with later dropoffs. That way, the exact distances can be
                        // computed in a bundled fashion and the postponed assignments can use exact distances afterward.
                        curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                        pairedContinuations.push_back({asgn.pickup->id, 0, scannedUntil});
                        ++numAssignmentsTriedWithPickupBeforeNextStop; // Count first ordinary continuation
                        ordinaryContinuations.push_back(
                                {asgn.pickup->id, distFromPickup, relOrdinaryDropoffsForVeh.begin()});
                        continue; // Continue with next pickup, rest of assignments for this pickup later with exact distance
                    }
                }


                asgn.distFromPickup = distFromPickup;
                const auto scannedUntil = tryLowerBoundsForOrdinary(asgn, bestAsgn, routeState);

                if (scannedUntil < relOrdinaryDropoffsForVeh.end()) {
                    // In this case some assignment with the pickup before the next stop and an ordinary dropoff
                    // needs the exact distance to pickup via the vehicle. Postpone computation
                    // of the yet unknown exact distance and the rest of the assignments with later dropoffs. That way,
                    // the exact distances can be computed in a bundled fashion and the postponed assignments can use
                    // exact distances afterward.
                    curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                    ordinaryContinuations.push_back({asgn.pickup->id, distFromPickup, scannedUntil});
                }
            }
        }


        // Examines combinations of a given pickup and all dropoffs before the next stop of a given vehicle until a
        // paired assignment needs the exact distance to the pickup via the vehicle. Returns an iterator to the dropoff at
        // which the exact distance is first needed or one-past-end iterator if all combinations could be filtered.
        RelevantPDLocs::It tryLowerBoundsForPaired(Assignment &asgn, const BestAsgn& bestAsgn, const RouteStateData& routeState) {
            assert(asgn.vehicle && asgn.pickup);
            const auto vehId = asgn.vehicle->vehicleId;


            const auto relevantDropoffs = relDropoffsBNS.relevantSpotsFor(vehId);

            if (!relDropoffsBNS.getVehiclesWithRelevantPDLocs().contains(vehId))
                return relevantDropoffs.end();

            const auto stopLocations = routeState.stopLocationsFor(vehId);

            asgn.distFromPickup = 0;
            asgn.dropoffStopIdx = 0;

            for (auto dropoffIt = relevantDropoffs.begin(); dropoffIt != relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;
                asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];
                if (stopLocations[1] == asgn.dropoff->loc)
                    continue;
                ++numAssignmentsTriedWithPickupBeforeNextStop;

                asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                const auto cost = Calc::calc(asgn, requestState, routeState);
                if (cost < bestAsgn.cost() || (cost == bestAsgn.cost() &&
                                               breakCostTie(asgn, bestAsgn.asgn()))) {
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
        RelevantPDLocs::It tryLowerBoundsForOrdinary(Assignment &asgn, const BestAsgn& bestAsgn, const RouteStateData& routeState) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup);
            const auto vehId = asgn.vehicle->vehicleId;

            const auto relevantDropoffs = relOrdinaryDropoffs.relevantSpotsFor(vehId);

            if (!relOrdinaryDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId))
                return relevantDropoffs.end();

            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);

            for (auto dropoffIt = relevantDropoffs.begin(); dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;
                asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                if (dropoffEntry.stopIndex + 1 < numStops &&
                    stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff->loc)
                    continue;
                if (asgn.dropoff->loc == asgn.pickup->loc)
                    continue;

                asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;

                ++numAssignmentsTriedWithPickupBeforeNextStop;

                const auto cost = Calc::calc(asgn, requestState, routeState);
                if (cost < bestAsgn.cost() || (cost == bestAsgn.cost() &&
                                               breakCostTie(asgn, bestAsgn.asgn()))) {
                    // Lower bound is better than best known cost => We need the exact distance to pickup.
                    // Return and postpone remaining combinations.
                    return dropoffIt;
                }
            }

            return relevantDropoffs.end();
        }

        void finishContinuations(const Vehicle &veh, BestAsgn& bestAsgn, const RouteStateData& routeState) {
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            Assignment asgn(&veh);

            // Finish all postponed assignments where dropoff is at stop >= 1.
            const auto relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
            for (const auto &continuation: ordinaryContinuations) {
                asgn.pickup = &requestState.pickups[continuation.pickupId];

                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = continuation.distFromPickup;

                for (auto dropoffIt = continuation.continueIt;
                     dropoffIt < relOrdinaryDropoffsForVeh.end(); ++dropoffIt) {
                    const auto &dropoffEntry = *dropoffIt;
                    asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                    if (dropoffEntry.stopIndex + 1 < numStops &&
                        stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff->loc)
                        continue;
                    if (asgn.pickup->loc == asgn.dropoff->loc)
                        continue;

                    asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                    asgn.distToDropoff = dropoffEntry.distToPDLoc;
                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                    bestAsgn.tryAssignment(asgn, routeState);

                    if (dropoffIt > continuation.continueIt) { // Do not count assignment at continuation twice
                        ++numAssignmentsTriedWithPickupBeforeNextStop;
                    }
                }
            }

            // Finish all paired assignments.
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBNS.relevantSpotsFor(veh.vehicleId);
            for (const auto &continuation: pairedContinuations) {
                const auto pId = continuation.pickupId;
                asgn.pickup = &requestState.pickups[pId];

                asgn.dropoffStopIdx = 0;
                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, pId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = 0;

                for (auto dropoffIt = continuation.continueIt;
                     dropoffIt < relDropoffsBeforeNextStopForVeh.end(); ++dropoffIt) {
                    const auto &dropoffEntry = *dropoffIt;
                    asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                    if (stopLocations[1] == asgn.dropoff->loc)
                        continue;

                    if (dropoffIt > continuation.continueIt) { // Do not count assignment at continuation twice
                        ++numAssignmentsTriedWithPickupBeforeNextStop;
                    }

                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                    if (asgn.distFromDropoff >= INFTY)
                        continue;

                    asgn.distToDropoff = pdDistances.getDirectDistance(pId, asgn.dropoff->id);
                    bestAsgn.tryAssignment(asgn, routeState);
                }
            }
        }


        const RelevantPDLocs &relPickupsBNS;
        const RelevantPDLocs &relOrdinaryDropoffs;
        const RelevantPDLocs &relDropoffsBNS;
        const PDDistancesT &pdDistances;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const Fleet &fleet;
        const RequestState &requestState;
        stats::PbnsAssignmentsPerformanceStats& stats;

        int numAssignmentsTriedWithPickupBeforeNextStop;
        std::vector<Continuation> ordinaryContinuations;
        std::vector<Continuation> pairedContinuations;

    };
}