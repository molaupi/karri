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

namespace karri::DropoffAfterLastStopStrategies {

    template<typename InputGraphT,
            typename LastStopsAtVerticesT,
            typename CurVehLocToPickupSearchesT,
            typename DijLabelSet>
    struct DijkstraStrategy {

    private:

        static constexpr int K = DijLabelSet::K;
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;


        struct TryToInsertDropoffAfterLastStop {
            TryToInsertDropoffAfterLastStop(DijkstraStrategy &strategy,
                                            CostCalculator calculator)
                    : strategy(strategy),
                      calculator(calculator) {}

            template<typename DistLabelContainerT>
            bool operator()(const int v, DistanceLabel &distFromV, const DistLabelContainerT & /*distLabels*/) {

                const DistanceLabel minCosts =
                        calculator.calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff<DijLabelSet>(
                                strategy.curWalkingDists, distFromV, 0, *strategy.curReqState);
                const LabelMask bestCostExceeded = strategy.curBestCostAsLabel < minCosts;
                if (allSet(bestCostExceeded))
                    return true;

                strategy.enumerateAssignmentsWithLastStopsAt(v, distFromV, minCosts, *strategy.curReqState);
                return false;
            }

        private:
            DijkstraStrategy &strategy;
            CostCalculator calculator;
        };

    public:

        DijkstraStrategy(const InputGraphT &inputGraph,
                         const InputGraphT &reverseGraph,
                         const Fleet &fleet,
                         CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                         const RouteState &routeState,
                         const LastStopsAtVerticesT &lastStopsAtVertices)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  routeState(routeState),
                  lastStopsAtVertices(lastStopsAtVertices),
                  calculator(routeState),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  pickupsToTryBeforeNextStop(),
                  dijSearchToDropoff(reverseGraph, {*this, CostCalculator(routeState)}),
                  vehiclesSeen(fleet.size()) {}

        void tryDropoffAfterLastStop(const RelevantPDLocs &relevantOrdinaryPickups,
                                     const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                     RequestState &requestState,
                                     const PDLocs &pdLocs, stats::DalsAssignmentsPerformanceStats &stats) {

            vehiclesSeen.clear();
            tryAssignmentsTime = 0;
            timeSpentLocatingVehicles = 0;
            numAssignmentsTried = 0;
            numLastStopsVisited = 0;
            curBestCostAsLabel = DistanceLabel(requestState.getBestCost());
            int numEdgeRelaxations = 0;
            int numVerticesSettled = 0;

            // Set pointer to relevant ordinary pickups and pickups before next stop so Dijkstra callback can access them
            curReqState = &requestState;
            curPdLocs = &pdLocs;
            curRelOrdinaryPickups = &relevantOrdinaryPickups;
            curRelPickupsBns = &relevantPickupsBeforeNextStop;


            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            Timer timer;

            // Runs Dijkstra searches. Possible assignments are enumerated during the searches whenever a last stop is
            // scanned.
            for (unsigned int i = 0; i < pdLocs.numDropoffs(); i += K) {
                runSearchesForDropoffBatch(i, pdLocs);

                numEdgeRelaxations += dijSearchToDropoff.getNumEdgeRelaxations();
                numVerticesSettled += dijSearchToDropoff.getNumVerticesSettled();
            }

            int64_t totalTime = timer.elapsed<std::chrono::nanoseconds>();

            // Time spent to locate vehicles and compute distances from current vehicle locations to pickups is counted
            // into PBNS time so subtract it here.
            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;
            tryAssignmentsTime -= pbnsTime;
            totalTime -= pbnsTime;

            const int64_t searchTime = totalTime - tryAssignmentsTime;
            stats.numEdgeRelaxationsInSearchGraph += numEdgeRelaxations;
            stats.numVerticesOrLabelsSettled += numVerticesSettled;
            stats.numEntriesOrLastStopsScanned += numLastStopsVisited;
            stats.searchTime += searchTime;
            stats.numCandidateDropoffsAcrossAllVehicles += vehiclesSeen.size() * pdLocs.numDropoffs();
            stats.numCandidateVehicles += vehiclesSeen.size();
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryAssignmentsTime += tryAssignmentsTime - timeSpentLocatingVehicles;
        }

    private:

        void runSearchesForDropoffBatch(const int firstDropoffId, const PDLocs &pdLocs) {
            assert(firstDropoffId % K == 0 && firstDropoffId < pdLocs.numDropoffs());

            std::array<int, K> dropoffTails;
            std::array<int, K> offsets;
            for (int i = 0; i < K; ++i) {
                curDropoffIds[i] = firstDropoffId + i;
                if (curDropoffIds[i] >= pdLocs.numDropoffs())
                    curDropoffIds[i] = firstDropoffId; // fill last batch with copies of first dropoff in batch
                const auto &dropoff = pdLocs.dropoffs[curDropoffIds[i]];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                offsets[i] = inputGraph.travelTime(dropoff.loc);
                curWalkingDists[i] = dropoff.walkingDist;
            }

            // Mark index from where last batch contains only copies.
            if (firstDropoffId + K > pdLocs.numDropoffs()) {
                endOfCurBatch = pdLocs.numDropoffs() % K;
            } else {
                endOfCurBatch = K;
            }

            // Search accesses the batch through curDropoffIds
            dijSearchToDropoff.runWithOffset(dropoffTails, offsets);
        }


        void
        enumerateAssignmentsWithLastStopsAt(const int v, const DistanceLabel &distFromV,
                                            const DistanceLabel &minCosts,
                                            RequestState &requestState) {
            using namespace time_utils;

            const auto &pdLocs = *curPdLocs;
            const auto &vehiclesWithLastStopAtV = lastStopsAtVertices.vehiclesWithLastStopAt(v);

            if (vehiclesWithLastStopAtV.empty())
                return;

            Timer timer;

            Assignment asgn;
            auto &dropoffIndex = asgn.dropoffStopIdx;

            for (const auto &vehId: vehiclesWithLastStopAtV) {
                ++numLastStopsVisited;

                const auto occupancies = routeState.occupanciesFor(vehId);
                asgn.vehicle = &fleet[vehId];
                dropoffIndex = routeState.numStopsOf(vehId) - 1;

                if (curRelOrdinaryPickups->hasRelevantSpotsFor(vehId)) {
                    vehiclesSeen.insert(vehId);
                    const auto relevantPickupsInRevOrder = curRelOrdinaryPickups->relevantSpotsForInReverseOrder(vehId);

                    for (int searchIdx = 0; searchIdx < endOfCurBatch; ++searchIdx) {
                        if (minCosts[searchIdx] > requestState.getBestCost())
                            continue;

                        asgn.dropoff = pdLocs.dropoffs[curDropoffIds[searchIdx]];
                        asgn.distToDropoff = distFromV[searchIdx];

                        for (auto pickupIt = relevantPickupsInRevOrder.begin();
                             pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                            const auto &entry = *pickupIt;

                            if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders >
                                asgn.vehicle->capacity)
                                break;

                            asgn.pickup = pdLocs.pickups[entry.pdId];
                            if (asgn.pickup.loc == asgn.dropoff.loc)
                                continue;

                            asgn.pickupStopIdx = entry.stopIndex;
                            asgn.distToPickup = entry.distToPDLoc;
                            asgn.distFromPickup = entry.distFromPDLocToNextStop;

                            requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            ++numAssignmentsTried;
                        }
                    }
                }

                if (curRelPickupsBns->hasRelevantSpotsFor(vehId) &&
                    occupancies[0] != asgn.vehicle->capacity) {
                    vehiclesSeen.insert(vehId);
                    asgn.pickupStopIdx = 0;

                    for (int searchIdx = 0; searchIdx < endOfCurBatch; ++searchIdx) {
                        if (minCosts[searchIdx] > requestState.getBestCost())
                            continue;

                        asgn.dropoff = pdLocs.dropoffs[curDropoffIds[searchIdx]];
                        asgn.distToDropoff = distFromV[searchIdx];

                        pickupsToTryBeforeNextStop.clear();


                        for (const auto &entry: curRelPickupsBns->relevantSpotsFor(vehId)) {
                            asgn.pickup = pdLocs.pickups[entry.pdId];
                            if (asgn.pickup.loc == asgn.dropoff.loc)
                                continue;

                            asgn.distFromPickup = entry.distFromPDLocToNextStop;

                            if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup.id)) {
                                // If we know the exact distance to the pickup via the vehicles current location, we try
                                // the precise assignment.
                                asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup.id);
                                if (asgn.distToPickup >= INFTY)
                                    continue;

                                requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                                ++numAssignmentsTried;
                            } else {
                                // If we don't know that distance, we set dist to pickup to a lower bound not taking into
                                // account the path via the current vehicle location first.
                                asgn.distToPickup = entry.distToPDLoc;

                                const auto cost = calculator.calc(asgn, requestState);
                                if (cost < requestState.getBestCost() || (cost == requestState.getBestCost() &&
                                                                          breakCostTie(asgn,
                                                                                       requestState.getBestAssignment()))) {
                                    // In this case, we need the exact distance to the pickup via the current location of the
                                    // vehicle. We postpone computation of that distance to be able to bundle it with the
                                    // computation of distances to other pickups via the vehicle location. Then all remaining
                                    // assignments with this pickup can be tried with the exact distance later.
                                    pickupsToTryBeforeNextStop.push_back({asgn.pickup.id, asgn.distFromPickup});
                                    curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                                }
                            }
                        }


                        curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);
                        for (const auto &pair: pickupsToTryBeforeNextStop) {
                            asgn.pickup = pdLocs.pickups[pair.first];
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup.id);
                            if (asgn.distToPickup >= INFTY)
                                continue;

                            asgn.distFromPickup = pair.second;
                            requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            ++numAssignmentsTried;
                        }
                    }
                }
            }

            curBestCostAsLabel = DistanceLabel(requestState.getBestCost());


            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            tryAssignmentsTime += time;
        }


        const InputGraphT &inputGraph;

        const Fleet &fleet;
        const RouteState &routeState;
        const LastStopsAtVerticesT &lastStopsAtVertices;
        CostCalculator calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;

        std::vector <std::pair<unsigned int, int>> pickupsToTryBeforeNextStop;

        int numLastStopsVisited;
        int numAssignmentsTried;
        int64_t timeSpentLocatingVehicles;
        int64_t tryAssignmentsTime;

        RequestState *curReqState;
        PDLocs const *curPdLocs;
        RelevantPDLocs const *curRelOrdinaryPickups;
        RelevantPDLocs const *curRelPickupsBns;

        std::array<unsigned int, K> curDropoffIds;
        DistanceLabel curWalkingDists;
        DistanceLabel curBestCostAsLabel;
        int endOfCurBatch;

        Dijkstra <InputGraphT, TravelTimeAttribute, DijLabelSet, TryToInsertDropoffAfterLastStop> dijSearchToDropoff;
        LightweightSubset vehiclesSeen;


    };

}