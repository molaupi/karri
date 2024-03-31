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
#include <tbb/parallel_for.h>

namespace karri {


// Finds pickup before next stop assignments, i.e. assignments where the pickup is inserted before the vehicle's next
// stop, which means the vehicle is rerouted at its current location. Dropoff may be inserted before the next stop or
// at ordinary locations, i.e. after the next stop but before the last stop.
// (The case of pickup before next stop and dropoff after last stop is considered by the DALSAssignmentsFinder.)
//
// Works based on filtered relevant pickups and dropoffs before next stop as well as relevant ordinary dropoffs.
    template<typename PDDistancesT,
            typename CurVehLocToPickupSearchesT>
    class PBNSAssignmentsFinder {

        enum AssignmentSubType : int8_t {
            PAIRED,
            ORDINARY,
//            DALS
        };

        struct ContinuationJob {
            int vehId;
            int pickupId;
            int distFromPickup;
            int offsetInDropoffs; // At which of the relevant dropoffs do we continue enumerating assignments
            AssignmentSubType type;
        };

    public:

        PBNSAssignmentsFinder(const RelevantPDLocs &relPickupsBns, const RelevantPDLocs &relOrdinaryDropoffs,
                              const RelevantPDLocs &relDropoffsBns,
                              const PDDistancesT &pdDistances,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                              const Fleet &fleet, const CostCalculator &calculator,
                              const RouteState &routeState, RequestState &requestState)
                : relPickupsBNS(relPickupsBns),
                  relOrdinaryDropoffs(relOrdinaryDropoffs),
                  relDropoffsBNS(relDropoffsBns),
                  pdDistances(pdDistances),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  fleet(fleet),
                  calculator(calculator),
                  routeState(routeState),
                  requestState(requestState),
                  localBestCosts([&] { return requestState.getBestCost(); }),
                  localBestAssignments([&] { return requestState.getBestAssignment(); }),
                  vehiclesForExactDistanceSearches(fleet.size()),
                  pickupsForExactDistancesSearches(0) {}

        void findAssignments() {
            numAssignmentsTriedWithPickupBeforeNextStop.store(0, std::memory_order_relaxed);
            Timer timer;

            // Determine which distances between pickups and vehicles are needed
            determineVehiclesAndPickupsForSearches();

            // Compute exact distances between vehicles and pickups
            curVehLocToPickupSearches.buildBucketsForMarkedVehiclesSequential(vehiclesForExactDistanceSearches);
            curVehLocToPickupSearches.computeDistancesForMarkedPickupsParallel(pickupsForExactDistancesSearches);

            // Finish continuations (i.e. try assignments with now known exact distances)
            finishContinuations();


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.tryAssignmentsAndLocatingVehiclesTime += time;

            requestState.stats().pbnsAssignmentsStats.numAssignmentsTried += numAssignmentsTriedWithPickupBeforeNextStop.load(
                    std::memory_order_relaxed);

            requestState.stats().pbnsAssignmentsStats.locatingVehiclesTimeLocal += curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            requestState.stats().pbnsAssignmentsStats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            requestState.stats().pbnsAssignmentsStats.bchSearchTimeLocal += curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
        }

        // Initialize for new request.
        void init() {
            Timer timer;
            localBestCosts.clear();
            localBestAssignments.clear();
            curVehLocToPickupSearches.initialize();

            continuationJobs.clear();
            vehiclesForExactDistanceSearches.clear();
            pickupsForExactDistancesSearches.clear();
            pickupsForExactDistancesSearches.resizeUnderlyingSet(requestState.numPickups());

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.initializationTime += time;
        }

    private:

        void determineVehiclesAndPickupsForSearches() {

//            int numCandidateVehicles = 0;
//            std::vector<std::pair<RelevantPDLoc, int>> jobs;
//            for (const auto &vehId: relPickupsBNS.getVehiclesWithRelevantPDLocs()) {
//
//                if (!relOrdinaryDropoffs.hasRelevantSpotsFor(vehId) && !relDropoffsBNS.hasRelevantSpotsFor(vehId))
//                    continue;
//                ++numCandidateVehicles;
//
//                assert(routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders <=
//                       fleet[vehId].capacity);
//
//                for (const auto &entry: relPickupsBNS.relevantSpotsFor(vehId)) {
//                    jobs.emplace_back(entry, vehId);
//                }
//            }

            CAtomic<int> numCandidateVehicles(0);
            const auto& vehicles = relPickupsBNS.getVehiclesWithRelevantPDLocs();
            tbb::parallel_for(int(0), static_cast<int>(vehicles.size()), 1, [&](int vehIdx) {
                const auto& vehId = *(vehicles.begin() + vehIdx);
                if (!relOrdinaryDropoffs.hasRelevantSpotsFor(vehId) && !relDropoffsBNS.hasRelevantSpotsFor(vehId))
                    return;

                numCandidateVehicles.add_fetch(1, std::memory_order_relaxed);
                assert(routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders <=
                       fleet[vehId].capacity);

                const auto& entries = relPickupsBNS.relevantSpotsFor(vehId);
                tbb::parallel_for(int(0), static_cast<int>(entries.size()), 1, [&](int entryIdx) {
                    const auto& entry = *(entries.begin() + entryIdx);
                    determineNecessaryExactDistancesForPickup(entry, vehId);
                });
            });

            requestState.stats().pbnsAssignmentsStats.numCandidateVehicles += numCandidateVehicles.load(std::memory_order_seq_cst);
        }

        void determineNecessaryExactDistancesForPickup(const RelevantPDLoc &entry, const int vehId) {
            using namespace time_utils;
            const auto &relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(vehId);
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBNS.relevantSpotsFor(vehId);


            if (curVehLocToPickupSearches.knowsDistance(vehId, entry.pdId)) {
                continuationJobs.push_back({vehId, entry.pdId, 0, 0, PAIRED});
                continuationJobs.push_back({vehId, entry.pdId, entry.distFromPDLocToNextStop, 0, ORDINARY});
                // Count first continuations
                numAssignmentsTriedWithPickupBeforeNextStop.fetch_add(3, std::memory_order_relaxed);
                return;
            }

            Assignment asgn(&fleet[vehId]);
            assert(entry.pdId >= 0 && entry.pdId < requestState.numPickups());
            asgn.pickup = &requestState.pickups[entry.pdId];

            int numAssignmentsTriedLocal = 0;

            // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
            // vehicle's current location to the pickup => we get lower bound costs.
            asgn.distToPickup = entry.distToPDLoc;
            const int distFromPickup = entry.distFromPDLocToNextStop;

            // For paired assignments before next stop, first try a lower bound with the smallest direct PD distance
            const auto lowerBoundCostPairedAssignment = calculator.calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                    fleet[vehId], *asgn.pickup, asgn.distToPickup, requestState.minDirectPDDist,
                    distFromPickup, requestState);
            if (lowerBoundCostPairedAssignment < requestState.getBestCost()) {
                const auto pairedScannedUntil = tryLowerBoundsForPaired(asgn, numAssignmentsTriedLocal);
                if (pairedScannedUntil < relDropoffsBeforeNextStopForVeh.end()) {
                    // In this case some paired assignment before the next stop needs the exact distance to pickup via
                    // the vehicle. Postpone computation of the yet unknown exact distance and the rest of the paired
                    // assignments as well as all assignments with later dropoffs. That way, the exact distances can be
                    // computed in a bundled fashion and the postponed assignments can use exact distances afterward.
                    vehiclesForExactDistanceSearches.insert(vehId);
                    pickupsForExactDistancesSearches.insert(asgn.pickup->id);
                    const int offsetInDropoffs = pairedScannedUntil - relDropoffsBeforeNextStopForVeh.begin();

                    assert(asgn.pickup->id >= 0 && asgn.pickup->id < requestState.numPickups());
                    continuationJobs.push_back({vehId, asgn.pickup->id, 0, offsetInDropoffs, PAIRED});
                    continuationJobs.push_back({vehId, asgn.pickup->id, distFromPickup, 0, ORDINARY});
                    numAssignmentsTriedLocal += 2; // Count first ordinary and DALS continuations
                    numAssignmentsTriedWithPickupBeforeNextStop.fetch_add(numAssignmentsTriedLocal,
                                                                          std::memory_order_relaxed);
                    return; // Continue with next pickup, rest of assignments for this pickup later with exact distance
                }
            }

            asgn.distFromPickup = distFromPickup;
            const auto ordinaryScannedUntil = tryLowerBoundsForOrdinary(asgn, numAssignmentsTriedLocal);
            if (ordinaryScannedUntil < relOrdinaryDropoffsForVeh.end()) {
                // In this case some assignment with the pickup before the next stop and an ordinary dropoff
                // needs the exact distance to pickup via the vehicle. Postpone computation
                // of the yet unknown exact distance and the rest of the assignments with later dropoffs. That way,
                // the exact distances can be computed in a bundled fashion and the postponed assignments can use
                // exact distances afterward.
                vehiclesForExactDistanceSearches.insert(vehId);
                pickupsForExactDistancesSearches.insert(asgn.pickup->id);
                const int offsetInDropoffs = ordinaryScannedUntil - relOrdinaryDropoffsForVeh.begin();
                assert(asgn.pickup->id >= 0 && asgn.pickup->id < requestState.numPickups());
                continuationJobs.push_back({vehId, asgn.pickup->id, distFromPickup, offsetInDropoffs, ORDINARY});
                numAssignmentsTriedLocal += 1; // Count first DALS continuation
                numAssignmentsTriedWithPickupBeforeNextStop.fetch_add(numAssignmentsTriedLocal,
                                                                      std::memory_order_relaxed);
            }
        }

        // Examines combinations of a given pickup and all dropoffs before the next stop of a given vehicle until a
        // paired assignment needs the exact distance to the pickup via the vehicle. Returns an iterator to the dropoff at
        // which the exact distance is first needed or one-past-end iterator if all combinations could be filtered.
        RelevantPDLocs::It tryLowerBoundsForPaired(Assignment &asgn, int &numAssignmentsTried) {
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
                ++numAssignmentsTried;

                asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                const auto cost = calculator.calc(asgn, requestState);
                if (cost < requestState.getBestCost() ||
                    (cost == requestState.getBestCost() && breakCostTie(asgn, requestState.getBestAssignment()))) {
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
        RelevantPDLocs::It tryLowerBoundsForOrdinary(Assignment &asgn, int &numAssignmentsTried) {
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
                const auto stopIdx = routeState.stopPositionOf(dropoffEntry.stopId);

                if (stopIdx + 1 < numStops && stopLocations[stopIdx + 1] == asgn.dropoff->loc)
                    continue;
                if (asgn.dropoff->loc == asgn.pickup->loc)
                    continue;

                asgn.dropoffStopIdx = stopIdx;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                ++numAssignmentsTried;

                const auto cost = calculator.calc(asgn, requestState);
                if (cost < requestState.getBestCost() ||
                    (cost == requestState.getBestCost() && breakCostTie(asgn, requestState.getBestAssignment()))) {
                    // Lower bound is better than best known cost => We need the exact distance to pickup.
                    // Return and postpone remaining combinations.
                    return dropoffIt;
                }
            }

            return relevantDropoffs.end();
        }

        void finishContinuations() {

            tbb::parallel_for(0, static_cast<int>(continuationJobs.size()), [&](const int i) {

                int &localBestCost = localBestCosts.local();
                Assignment &localBestAssignment = localBestAssignments.local();
                const auto &cont = continuationJobs[i];
                const auto &veh = fleet[cont.vehId];
                const auto &pickup = requestState.pickups[cont.pickupId];

                const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
                const auto numStops = routeState.numStopsOf(veh.vehicleId);
                Assignment asgn(&veh);

                // Finish all postponed assignments where dropoff is at stop >= 1.
                asgn.pickup = &pickup;
                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, pickup.id);
                if (asgn.distToPickup >= INFTY)
                    return;
                asgn.distFromPickup = cont.distFromPickup;

                if (cont.type == PAIRED) {
                    // Continuation is for paired assignments
                    const auto relDropoffsBNSForVeh = relDropoffsBNS.relevantSpotsFor(veh.vehicleId);
                    asgn.dropoffStopIdx = 0;

                    for (auto dropoffIt = relDropoffsBNSForVeh.begin() + cont.offsetInDropoffs;
                         dropoffIt < relDropoffsBNSForVeh.end(); ++dropoffIt) {
                        const auto &dropoffEntry = *dropoffIt;
                        asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                        if (stopLocations[1] == asgn.dropoff->loc)
                            continue;

                        asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                        asgn.distToDropoff = pdDistances.getDirectDistance(pickup.id, asgn.dropoff->id);
                        tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                    }
                    // Do not count assignment at continuation twice
                    numAssignmentsTriedWithPickupBeforeNextStop.fetch_add(
                            relDropoffsBNSForVeh.size() - cont.offsetInDropoffs - 1, std::memory_order_relaxed);
                    return;
                }

                // Continuation is for ordinary dropoffs
                const auto relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
                for (auto dropoffIt = relOrdinaryDropoffsForVeh.begin() + cont.offsetInDropoffs;
                     dropoffIt < relOrdinaryDropoffsForVeh.end(); ++dropoffIt) {
                    const auto &dropoffEntry = *dropoffIt;
                    const auto stopIdx = routeState.stopPositionOf(dropoffEntry.stopId);
                    asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                    if (stopIdx + 1 < numStops && stopLocations[stopIdx + 1] == asgn.dropoff->loc)
                        continue;
                    if (asgn.pickup->loc == asgn.dropoff->loc)
                        continue;

                    asgn.dropoffStopIdx = stopIdx;
                    asgn.distToDropoff = dropoffEntry.distToPDLoc;
                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                    tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                }
                // Do not count assignment at continuation twice
                numAssignmentsTriedWithPickupBeforeNextStop.fetch_add(
                        relOrdinaryDropoffsForVeh.size() - cont.offsetInDropoffs - 1, std::memory_order_relaxed);
            });

            // Apply local best assignments to global result
            for (const auto &asgn: localBestAssignments) {
                if (asgn.vehicle && asgn.pickup && asgn.dropoff)
                    requestState.tryAssignment(asgn);
            }
        }

        void tryAssignmentLocal(const Assignment &asgn, int &localBestCost, Assignment &localBestAssignment) const {

            const auto cost = calculator.calc(asgn, requestState);
            if (cost < localBestCost || (cost == localBestCost && breakCostTie(asgn, localBestAssignment))) {
                localBestCost = cost;
                localBestAssignment = asgn;
            }
        }


        const RelevantPDLocs &relPickupsBNS;
        const RelevantPDLocs &relOrdinaryDropoffs;
        const RelevantPDLocs &relDropoffsBNS;
        const PDDistancesT &pdDistances;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;

        tbb::enumerable_thread_specific<int> localBestCosts;
        tbb::enumerable_thread_specific<Assignment> localBestAssignments;

        CAtomic<int> numAssignmentsTriedWithPickupBeforeNextStop;

        ThreadSafeSubset vehiclesForExactDistanceSearches;
        ThreadSafeSubset pickupsForExactDistancesSearches;

        tbb::concurrent_vector<ContinuationJob> continuationJobs;

    };
}