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
#include <tbb/parallel_for.h>

namespace karri {


// Finds pickup before next stop assignments, i.e. assignments where the pickup is inserted before the vehicle's next
// stop, which means the vehicle is rerouted at its current location. Dropoff may be inserted before the next stop or
// at ordinary locations, i.e. after the next stop but before the last stop.
// (The case of pickup before next stop and dropoff after last stop is considered by the DALSAssignmentsFinder.)
//
// Works based on filtered relevant pickups and dropoffs before next stop as well as relevant ordinary dropoffs.
    template<typename PDDistancesT, typename CurVehLocToPickupSearchesT>
    class PBNSAssignmentsFinder {

    public:

        PBNSAssignmentsFinder(const RelevantPDLocs &relPickupsBns, const RelevantPDLocs &relOrdinaryDropoffs,
                              const RelevantPDLocs &relDropoffsBns, const PDDistancesT &pdDistances,
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
                  requestState(requestState) {}

        void findAssignments() {
            numAssignmentsTriedWithPickupBeforeNextStop.store(0, std::memory_order_relaxed);
            Timer timer;

            int numCandidateVehicles = 0;

            std::vector<std::pair<RelevantPDLocs::RelevantPDLoc, int>> jobs;
            
            for (const auto &vehId: relPickupsBNS.getVehiclesWithRelevantPDLocs()) {

                if (!relOrdinaryDropoffs.hasRelevantSpotsFor(vehId) && !relDropoffsBNS.hasRelevantSpotsFor(vehId))
                    continue;
                ++numCandidateVehicles;

                assert(routeState.occupanciesFor(vehId)[0]  + requestState.originalRequest.numRiders <= fleet[vehId].capacity);

                for (const auto &entry: relPickupsBNS.relevantSpotsFor(vehId)) {
                    jobs.emplace_back(entry, vehId);
                }
            }

//            auto permutation = Permutation::getRandomPermutation(jobs.size(), std::minstd_rand(requestState.originalRequest.requestId));
//            permutation.applyTo(jobs);

            tbb::parallel_for(int(0), static_cast<int>(jobs.size()), 1, [&](int i) {
                calculateNecessaryExactDistancesForPickup(jobs[i].first, fleet[jobs[i].second]);
            });

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.tryAssignmentsAndLocatingVehiclesTime += time;
            requestState.stats().pbnsAssignmentsStats.numCandidateVehicles += numCandidateVehicles;
            requestState.stats().pbnsAssignmentsStats.numAssignmentsTried += numAssignmentsTriedWithPickupBeforeNextStop.load(std::memory_order_relaxed);

            requestState.stats().pbnsAssignmentsStats.locatingVehiclesTimeLocal += curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            requestState.stats().pbnsAssignmentsStats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            requestState.stats().pbnsAssignmentsStats.directCHSearchTime += curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
        }

        // Initialize for new request.
        void init() {
            Timer timer;
            bestAsgnBefore = requestState.getBestAssignment();
            bestCostBefore = requestState.getBestCost();
            curVehLocToPickupSearches.initialize(requestState.originalRequest.requestTime);
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.initializationTime += time;
        }

    private:

        void calculateNecessaryExactDistancesForPickup(const RelevantPDLocs::RelevantPDLoc &entry, const Vehicle &veh) {
            using namespace time_utils;
            const auto &relOrdinaryDropoffsForVeh = relOrdinaryDropoffs.relevantSpotsFor(veh.vehicleId);
            const auto &relDropoffsBeforeNextStopForVeh = relDropoffsBNS.relevantSpotsFor(veh.vehicleId);
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto vehId = veh.vehicleId;

            Assignment asgn(&veh);
            asgn.pickup = &requestState.pickups[entry.pdId];

            Assignment localBestAsgn = bestAsgnBefore;
            int localBestCost = bestCostBefore;

            int numAssignmentsTriedWithPickupBeforeNextStopLocal = 0;

            // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
            // vehicle's current location to the pickup => we get lower bound costs.
            asgn.distToPickup = entry.distToPDLoc;
            const int distFromPickup = entry.distFromPDLocToNextStop;

            // For paired assignments before next stop, first try a lower bound with the smallest direct PD distance
            const auto lowerBoundCostPairedAssignment = calculator.calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                    veh, *asgn.pickup, asgn.distToPickup, requestState.minDirectPDDist,
                    distFromPickup, requestState);
            if (lowerBoundCostPairedAssignment < localBestCost) {
                assert(asgn.vehicle && asgn.pickup);

                if (relDropoffsBNS.getVehiclesWithRelevantPDLocs().contains(vehId)) {
                    asgn.distFromPickup = 0;
                    asgn.dropoffStopIdx = 0;

                    for (const auto& dropoffEntry : relDropoffsBeforeNextStopForVeh) {
                        asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];
                        if (stopLocations[1] == asgn.dropoff->loc)
                            continue;

                        asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                        if (asgn.distFromDropoff >= INFTY)
                                continue;
                        const auto cost = calculator.calc(asgn, requestState);
                        if (cost < localBestCost || (cost == localBestCost &&
                                                                breakCostTie(asgn, localBestAsgn))) {
                            // Lower bound is better than best known cost => We need the exact distance to pickup.
                            // Return and postpone remaining combinations.
                            curVehLocToPickupSearches.computeExactDistancesVia(veh, asgn.pickup->id, asgn.distToPickup);
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, asgn.pickup->id);
                            if (asgn.distToPickup >= INFTY)
                                continue;

                            ++numAssignmentsTriedWithPickupBeforeNextStopLocal;

                            const auto curCost = calculator.calc(asgn, requestState);
                            if (curCost < INFTY && (curCost < localBestCost || (curCost == localBestCost &&
                                            breakCostTie(asgn, localBestAsgn)))) {
                                localBestCost = curCost;
                                localBestAsgn = asgn;
                            }
                        }
                    }
                }
            }

            asgn.distFromPickup = distFromPickup;
            assert(asgn.vehicle && asgn.pickup);

            if (relOrdinaryDropoffs.getVehiclesWithRelevantPDLocs().contains(vehId)) {
                for (const auto& dropoffEntry : relOrdinaryDropoffsForVeh) {
                    asgn.dropoff = &requestState.dropoffs[dropoffEntry.pdId];

                    if (dropoffEntry.stopIndex + 1 < numStops &&
                        stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff->loc)
                        continue;
                    if (asgn.dropoff->loc == asgn.pickup->loc)
                        continue;

                    asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                    asgn.distToDropoff = dropoffEntry.distToPDLoc;
                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;

                    const auto cost = calculator.calc(asgn, requestState);
                    if (cost < localBestCost || (cost == localBestCost &&
                                                            breakCostTie(asgn, localBestAsgn))) {
                        // Lower bound is better than best known cost => We need the exact distance to pickup.
                        // Return and postpone remaining combinations.
                        curVehLocToPickupSearches.computeExactDistancesVia(veh, asgn.pickup->id, asgn.distToPickup);
                        asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, asgn.pickup->id);
                        if (asgn.distToPickup >= INFTY)
                            continue;

                        const auto curCost = calculator.calc(asgn, requestState);
                        if (curCost < INFTY && (curCost < localBestCost || (curCost == localBestCost &&
                                        breakCostTie(asgn, localBestAsgn)))) {
                            localBestCost = curCost;
                            localBestAsgn = asgn;
                        }

                        ++numAssignmentsTriedWithPickupBeforeNextStopLocal;
                    }
                }
            }

            if (localBestAsgn.vehicle && localBestAsgn.pickup && localBestAsgn.dropoff)
                requestState.tryAssignment(localBestAsgn);

            numAssignmentsTriedWithPickupBeforeNextStop.add_fetch(numAssignmentsTriedWithPickupBeforeNextStopLocal,
                std::memory_order_relaxed);
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

        Assignment bestAsgnBefore;
        int bestCostBefore;

        CAtomic<int> numAssignmentsTriedWithPickupBeforeNextStop;

    };
}