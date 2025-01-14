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

#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Algorithms/KaRRi/LastStopSearches/TentativeLastStopDistances.h"

namespace karri::DropoffAfterLastStopStrategies {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename CurVehLocToPickupSearchesT,
            typename LabelSet>
    struct IndividualBCHStrategy {
    private:


        static constexpr int K = LabelSet::K;
        using LabelMask = typename LabelSet::LabelMask;
        using DistanceLabel = typename LabelSet::DistanceLabel;


        struct DropoffAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = false;

            DropoffAfterLastStopPruner(IndividualBCHStrategy &strat,
                                       const CostCalculator &calc)
                    : strat(strat), calc(calc) {}

            // Returns whether a given distance from a vehicle's last stop to the dropoff cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel &distancesToDropoffs,
                                                   const bool considerWalkingDists) const {

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than the
                    // current best.
                    return ~(distancesToDropoffs < INFTY);
                }

                const DistanceLabel walkingDists = considerWalkingDists ? strat.currentDropoffWalkingDists : 0;
                const DistanceLabel costLowerBound = calc.template calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff<LabelSet>(
                        walkingDists, distancesToDropoffs, 0, strat.requestState);

                return strat.upperBoundCost < costLowerBound;
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the dropoff cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToDropoffs needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtDropoffs,
                                                  const DistanceLabel &minDistancesToDropoffs) const {

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtDropoffs[i] >= INFTY or
                    // minDistancesToDropoffs[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtDropoffs < INFTY) & (minDistancesToDropoffs < INFTY));
                }

                const DistanceLabel costLowerBound = calc.template calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinArrTime<LabelSet>(
                        strat.currentDropoffWalkingDists, minDistancesToDropoffs, arrTimesAtDropoffs, strat.requestState);

                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                            const DistanceLabel& distancesToDropoffs) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToDropoffs < INFTY);
                }

                const DistanceLabel costLowerBound = calc.template calcKVehicleDependentCostLowerBoundsForDALSWithKnownDistToDropoff<LabelSet>(
                        vehId, strat.currentDropoffWalkingDists, distancesToDropoffs, 0, strat.requestState);
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int, const DistanceLabel &) {
                // issue: for an upper bound on the cost, we need an upper bound on what the cost for a pickup using this
                // vehicle is (i.e. an upper bound on the detour, trip time and added trip time for existing passengers
                // until the last stop). However, for pickups before the next stop, we can't derive an upper bound on the
                // detour since we only know lower bounds from the elliptic BCH queries
            }

            bool isVehicleEligible(const int vehId) const {
                return strat.routeState.numStopsOf(vehId) > 1 &&
                       (strat.curRelPickupsBns->hasRelevantSpotsFor(vehId) ||
                        strat.curRelOrdinaryPickups->hasRelevantSpotsFor(vehId));
            }

        private:
            IndividualBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using DropoffBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, DropoffAfterLastStopPruner, LabelSet>;

    public:

        IndividualBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculator &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearchesT,
                              const RouteState &routeState,
                              RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  curVehLocToPickupSearches(curVehLocToPickupSearchesT),
                  routeState(routeState),
                  requestState(requestState),
                  checkPBNSForVehicle(fleet.size()),
                  vehiclesSeenForDropoffs(fleet.size()),
                  search(lastStopBucketsEnv, lastStopDistances, chEnv, routeState,
                         vehiclesSeenForDropoffs,
                         DropoffAfterLastStopPruner(*this, calculator)),
                  lastStopDistances(fleet.size()) {}

        void tryDropoffAfterLastStop(const RelevantPDLocs &relevantOrdinaryPickups,
                                     const RelevantPDLocs &relevantPickupsBeforeNextStop) {
            curRelOrdinaryPickups = &relevantOrdinaryPickups;
            curRelPickupsBns = &relevantPickupsBeforeNextStop;

            runBchQueries();
            enumerateAssignments(relevantOrdinaryPickups, relevantPickupsBeforeNextStop);
        }

    private:

        // Run BCH queries that obtain distances from last stops to dropoffs
        void runBchQueries() {
            Timer timer;

            initDropoffSearches();
            for (unsigned int i = 0; i < requestState.numDropoffs(); i += K)
                runSearchesForDropoffBatch(i);

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().dalsAssignmentsStats.searchTime += searchTime;
            requestState.stats().dalsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            requestState.stats().dalsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            requestState.stats().dalsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            requestState.stats().dalsAssignmentsStats.numCandidateVehicles += vehiclesSeenForDropoffs.size();
        }

        // Enumerate DALS assignments
        void enumerateAssignments(const RelevantPDLocs &relevantOrdinaryPickups,
                                  const RelevantPDLocs &relevantPickupsBeforeNextStop) {
            int numAssignmentsTried = 0;
            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            Timer timer;

            enumerateAssignmentsWithOrdinaryPickup(numAssignmentsTried, relevantOrdinaryPickups);
            enumerateAssignmentsWithPBNS(numAssignmentsTried, relevantPickupsBeforeNextStop);

            // Time spent to locate vehicles and compute distances from current vehicle locations to pickups is counted
            // into PBNS time so subtract it here.
            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>() - pbnsTime;
            requestState.stats().dalsAssignmentsStats.tryAssignmentsTime += tryAssignmentsTime;
            requestState.stats().dalsAssignmentsStats.numAssignmentsTried += numAssignmentsTried;

            // Find total number of candidate dropoffs for statistics
            int totalNumberOfCandidateDropoffs = 0;
            for (const auto &vehId: vehiclesSeenForDropoffs)
                for (const auto &dropoff: requestState.dropoffs)
                    totalNumberOfCandidateDropoffs += (getDistanceToDropoff(vehId, dropoff.id) < INFTY);
            requestState.stats().dalsAssignmentsStats.numCandidateDropoffsAcrossAllVehicles += totalNumberOfCandidateDropoffs;
        }

        // Enumerate assignments where pickup is after next stop (ordinary pickup):
        void enumerateAssignmentsWithOrdinaryPickup(int &numAssignmentsTried,
                                                    const RelevantPDLocs &relevantOrdinaryPickups) {
            Assignment asgn;

            checkPBNSForVehicle.reset();
            for (const auto &vehId: vehiclesSeenForDropoffs) {
                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId);
                    continue;
                }

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;

                for (const auto &dropoff: requestState.dropoffs) {
                    asgn.dropoff = &dropoff;

                    asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                    if (asgn.distToDropoff >= INFTY)
                        continue; // no need to check pickup before next stop

                    assert(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                    int curPickupIndex = numStops - 1;
                    auto pickupIt = relevantPickupsInRevOrder.begin();
                    for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                        const auto &entry = *pickupIt;

                        if (entry.stopIndex < curPickupIndex) {
                            // New smaller pickup index reached: Check if seating capacity and cost lower bound admit
                            // any valid assignments at this or earlier indices.
                            if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders > asgn.vehicle->capacity)
                                break;

                            assert(entry.stopIndex < numStops - 1);
                            const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                               routeState.schedArrTimesFor(vehId)[entry.stopIndex + 1];

                            const auto minCostFromHere = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                                    asgn.dropoff->walkingDist, asgn.distToDropoff, minTripTimeToLastStop, requestState);
                            if (minCostFromHere > requestState.getBestCost())
                                break;

                            curPickupIndex = entry.stopIndex;
                        }

                        asgn.pickup = &requestState.pickups[entry.pdId];
                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;
                        ++numAssignmentsTried;
                        asgn.pickupStopIdx = entry.stopIndex;
                        asgn.distToPickup = entry.distToPDLoc;
                        asgn.distFromPickup = entry.distFromPDLocToNextStop;
                        requestState.tryAssignment(asgn);
                    }

                    if (pickupIt == relevantPickupsInRevOrder.end()) {
                        // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                        // need to consider the pickup before next stop case.
                        checkPBNSForVehicle.set(vehId);
                    }
                }
            }
        }

        // Enumerate assignments where the pickup is before the next stop (PBNS + DALS):
        void enumerateAssignmentsWithPBNS(int &numAssignmentsTried,
                                          const RelevantPDLocs &relevantPickupsBeforeNextStop) {
            Assignment asgn;
            asgn.pickupStopIdx = 0;

            for (const auto &vehId: relevantPickupsBeforeNextStop.getVehiclesWithRelevantPDLocs()) {

                if (!vehiclesSeenForDropoffs.contains(vehId))
                    continue;

                if (!checkPBNSForVehicle.isSet(vehId))
                    continue;

                if (routeState.numStopsOf(vehId) == 0 ||
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders > fleet[vehId].capacity)
                    continue;

                pbnsContinuations.clear();

                const auto numStops = routeState.numStopsOf(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                for (auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                    asgn.pickup = &requestState.pickups[entry.pdId];
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;
                    for (const auto &dropoff: requestState.dropoffs) {
                        asgn.dropoff = &dropoff;
                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;

                        asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup->id)) {
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup->id);
                            requestState.tryAssignment(asgn);
                            ++numAssignmentsTried;
                        } else {
                            asgn.distToPickup = entry.distToPDLoc;
                            const auto lowerBoundCost = calculator.calc(asgn, requestState);
                            if (lowerBoundCost < requestState.getBestCost() ||
                                (lowerBoundCost == requestState.getBestCost() &&
                                 breakCostTie(asgn, requestState.getBestAssignment()))) {
                                // In this case, we need the exact distance to the pickup via the current location of the
                                // vehicle. We postpone computation of that distance to be able to bundle it with the
                                // computation of distances to other pickups via the vehicle location. Then all remaining
                                // assignments with this pickup can be tried with the exact distance later.
                                curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                                pbnsContinuations.push_back({asgn.pickup->id, asgn.distFromPickup, dropoff.id});
                                break;
                            }
                        }
                    }
                }

                // Continue with assignments for pickups where exact distance via vehicle location is needed
                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId]);
                for (const auto &continuation: pbnsContinuations) {
                    assert(continuation.pickupID >= 0 && continuation.pickupID < requestState.numPickups());
                    assert(continuation.fromDropoffID >= 0 && continuation.fromDropoffID < requestState.numDropoffs());
                    asgn.pickup = &requestState.pickups[continuation.pickupID];

                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId,
                                                                              continuation.pickupID);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    asgn.distFromPickup = continuation.distFromPickup;
                    for (int dropoffID = continuation.fromDropoffID;
                         dropoffID < requestState.numDropoffs(); ++dropoffID) {
                        asgn.dropoff = &requestState.dropoffs[dropoffID];
                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;

                        asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        ++numAssignmentsTried;
                        asgn.dropoffStopIdx = numStops - 1;
                        requestState.tryAssignment(asgn);
                    }
                }
            }
        }

        inline int getDistanceToDropoff(const int vehId, const unsigned int dropoffId) {
            return lastStopDistances.getDistance(vehId, dropoffId);
        }

        void initDropoffSearches() {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            upperBoundCost = requestState.getBestCost();
            vehiclesSeenForDropoffs.clear();

            // Construct more space for dropoff labels if needed.
            const int numDropoffBatches =
                    requestState.numDropoffs() / K + (requestState.numDropoffs() % K != 0);
            lastStopDistances.init(numDropoffBatches);

        }

        void runSearchesForDropoffBatch(const unsigned int firstDropoffId) {
            assert(firstDropoffId % K == 0 && firstDropoffId < requestState.numDropoffs());
            const int batchIdx = firstDropoffId / K;

            std::array<int, K> dropoffTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &dropoff =
                        firstDropoffId + i < requestState.numDropoffs() ? requestState.dropoffs[firstDropoffId + i]
                                                                        : requestState.dropoffs[firstDropoffId];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                travelTimes[i] = inputGraph.travelTime(dropoff.loc);
                currentDropoffWalkingDists[i] = dropoff.walkingDist;
            }

            lastStopDistances.setCurBatchIdx(batchIdx);
            search.run(dropoffTails, travelTimes);

            totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            totalNumVerticesSettled += search.getNumVerticesSettled();
            totalNumEntriesScanned += search.getNumEntriesScanned();
        }


        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const RouteState &routeState;
        RequestState &requestState;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        // Records for postponing and bundling computations of distances from current locations of vehicles to
        // pickups needed in the PBNS+DALS case.
        struct PickupBeforeNextStopContinuation {
            int pickupID;
            int distFromPickup;
            int fromDropoffID;
        };
        std::vector<PickupBeforeNextStopContinuation> pbnsContinuations;

        int upperBoundCost;

        // Vehicles seen by any last stop search
        Subset vehiclesSeenForDropoffs;
        DropoffBCHQuery search;
        DistanceLabel currentDropoffWalkingDists;
        TentativeLastStopDistances<LabelSet> lastStopDistances;

        // Pointers to relevant PD locs so Dijkstra search callback has access to them
        RelevantPDLocs const *curRelOrdinaryPickups;
        RelevantPDLocs const *curRelPickupsBns;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

    };

}