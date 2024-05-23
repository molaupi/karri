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
            typename CurVehLocToPickupSearchesT,
            typename DijLabelSet>
    struct DijkstraStrategy {

    private:

        static constexpr int K = DijLabelSet::K;
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;


        struct TryToInsertDropoffAfterLastStop {
            TryToInsertDropoffAfterLastStop(DijkstraStrategy &strategy,
                                            const CostCalculator &calculator,
                                            const RequestState &requestState)
                    : strategy(strategy),
                      calculator(calculator),
                      requestState(requestState) {}

            template<typename DistLabelContainerT>
            bool operator()(const int v, DistanceLabel &distFromV, const DistLabelContainerT & /*distLabels*/) {

                const DistanceLabel minCosts =
                        calculator.calcCostLowerBoundForKDropoffsAfterLastStop<DijLabelSet>(distFromV, 0, requestState);
                const LabelMask bestCostExceeded = strategy.curBestCostAsLabel < minCosts;
                if (allSet(bestCostExceeded))
                    return true;

                strategy.enumerateAssignmentsWithLastStopsAt(v, distFromV, minCosts);
                return false;
            }

        private:
            DijkstraStrategy &strategy;
            const CostCalculator &calculator;
            const RequestState &requestState;
        };

    public:

        DijkstraStrategy(const InputGraphT &inputGraph,
                         const InputGraphT &reverseGraph,
                         const Fleet &fleet,
                         const CostCalculator &calculator,
                         CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                         const RouteStateData &routeState,
                         const LastStopsAtVertices &lastStopsAtVertices,
                         RequestState &requestState,
                         const RelevantPDLocs &relevantOrdinaryPickups,
                         const RelevantPDLocs &relevantPickupsBeforeNextStop)
                : inputGraph(inputGraph),
                  requestState(requestState),
                  fleet(fleet),
                  routeState(routeState),
                  lastStopsAtVertices(lastStopsAtVertices),
                  calculator(calculator),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  relevantOrdinaryPickups(relevantOrdinaryPickups),
                  relevantPickupsBeforeNextStop(relevantPickupsBeforeNextStop),
                  pickupsToTryBeforeNextStop(),
                  dijSearchToDropoff(reverseGraph, {*this, calculator, requestState}),
                  vehiclesSeen(fleet.size()) {}

        void tryDropoffAfterLastStop() {

            vehiclesSeen.clear();
            tryAssignmentsTime = 0;
            timeSpentLocatingVehicles = 0;
            numAssignmentsTried = 0;
            numLastStopsVisited = 0;
            curBestCostAsLabel = DistanceLabel(requestState.getBestCost());
            int numEdgeRelaxations = 0;
            int numVerticesSettled = 0;


            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            Timer timer;

            // Runs Dijkstra searches. Possible assignments are enumerated during the searches whenever a last stop is
            // scanned.
            for (unsigned int i = 0; i < requestState.numDropoffs(); i += K) {
                runSearchesForDropoffBatch(i);

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
            requestState.stats().dalsAssignmentsStats.numEdgeRelaxationsInSearchGraph += numEdgeRelaxations;
            requestState.stats().dalsAssignmentsStats.numVerticesOrLabelsSettled += numVerticesSettled;
            requestState.stats().dalsAssignmentsStats.numEntriesOrLastStopsScanned += numLastStopsVisited;
            requestState.stats().dalsAssignmentsStats.searchTime += searchTime;
            requestState.stats().dalsAssignmentsStats.numCandidateDropoffsAcrossAllVehicles +=
                    vehiclesSeen.size() * requestState.numDropoffs();
            requestState.stats().dalsAssignmentsStats.numCandidateVehicles += vehiclesSeen.size();
            requestState.stats().dalsAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().dalsAssignmentsStats.tryAssignmentsTime += tryAssignmentsTime - timeSpentLocatingVehicles;
        }

    private:

        void runSearchesForDropoffBatch(const int firstDropoffId) {
            assert(firstDropoffId % K == 0 && firstDropoffId < requestState.numDropoffs());

            std::array<int, K> dropoffTails;
            std::array<int, K> offsets;
            for (int i = 0; i < K; ++i) {
                curDropoffIds[i] = firstDropoffId + i;
                if (curDropoffIds[i] >= requestState.numDropoffs())
                    curDropoffIds[i] = firstDropoffId; // fill last batch with copies of first dropoff in batch
                const auto &dropoff = requestState.dropoffs[curDropoffIds[i]];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                offsets[i] = inputGraph.travelTime(dropoff.loc);
            }

            // Mark index from where last batch contains only copies.
            if (firstDropoffId + K > requestState.numDropoffs()) {
                endOfCurBatch = requestState.numDropoffs() % K;
            } else {
                endOfCurBatch = K;
            }

            // Search accesses the batch through curDropoffIds
            dijSearchToDropoff.runWithOffset(dropoffTails, offsets);
        }


        void
        enumerateAssignmentsWithLastStopsAt(const int v, const DistanceLabel &distFromV, const DistanceLabel &minCosts) {
            using namespace time_utils;


            if (!lastStopsAtVertices.isAnyLastStopAtVertex(v))
                return;

            Timer timer;

            Assignment asgn;
            auto &dropoffIndex = asgn.dropoffStopIdx;

            for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(v)) {
                ++numLastStopsVisited;

                const auto occupancies = routeState.occupanciesFor(vehId);
                asgn.vehicle = &fleet[vehId];
                dropoffIndex = routeState.numStopsOf(vehId) - 1;

                if (relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    vehiclesSeen.insert(vehId);
                    const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(
                            vehId);

                    for (int searchIdx = 0; searchIdx < endOfCurBatch; ++searchIdx) {
                        if (minCosts[searchIdx] > requestState.getBestCost())
                            continue;

                        asgn.dropoff = &requestState.dropoffs[curDropoffIds[searchIdx]];
                        asgn.distToDropoff = distFromV[searchIdx];

                        for (auto pickupIt = relevantPickupsInRevOrder.begin();
                             pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                            const auto &entry = *pickupIt;

                            if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders > asgn.vehicle->capacity)
                                break;

                            asgn.pickup = &requestState.pickups[entry.pdId];
                            if (asgn.pickup->loc == asgn.dropoff->loc)
                                continue;

                            asgn.pickupStopIdx = entry.stopIndex;
                            asgn.distToPickup = entry.distToPDLoc;
                            asgn.distFromPickup = entry.distFromPDLocToNextStop;

                            requestState.tryAssignment(asgn);
                            ++numAssignmentsTried;
                        }
                    }
                }

                if (relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId) &&
                    occupancies[0] != asgn.vehicle->capacity) {
                    vehiclesSeen.insert(vehId);
                    asgn.pickupStopIdx = 0;

                    for (int searchIdx = 0; searchIdx < endOfCurBatch; ++searchIdx) {
                        if (minCosts[searchIdx] > requestState.getBestCost())
                            continue;

                        asgn.dropoff = &requestState.dropoffs[curDropoffIds[searchIdx]];
                        asgn.distToDropoff = distFromV[searchIdx];

                        pickupsToTryBeforeNextStop.clear();


                        for (const auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                            asgn.pickup = &requestState.pickups[entry.pdId];
                            if (asgn.pickup->loc == asgn.dropoff->loc)
                                continue;

                            asgn.distFromPickup = entry.distFromPDLocToNextStop;

                            if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup->id)) {
                                // If we know the exact distance to the pickup via the vehicles current location, we try
                                // the precise assignment.
                                asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup->id);
                                if (asgn.distToPickup >= INFTY)
                                    continue;

                                requestState.tryAssignment(asgn);
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
                                    pickupsToTryBeforeNextStop.push_back({asgn.pickup->id, asgn.distFromPickup});
                                    curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup->id,
                                                                                     asgn.distToPickup);
                                }
                            }
                        }


                        curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId]);
                        for (const auto &pair: pickupsToTryBeforeNextStop) {
                            asgn.pickup = &requestState.pickups[pair.first];
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup->id);
                            if (asgn.distToPickup >= INFTY)
                                continue;

                            asgn.distFromPickup = pair.second;
                            requestState.tryAssignment(asgn);
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
        RequestState &requestState;

        const Fleet &fleet;
        const RouteStateData &routeState;
        const LastStopsAtVertices &lastStopsAtVertices;
        const CostCalculator &calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const RelevantPDLocs &relevantOrdinaryPickups;
        const RelevantPDLocs &relevantPickupsBeforeNextStop;

        std::vector<std::pair<unsigned int, int>> pickupsToTryBeforeNextStop;

        int numLastStopsVisited;
        int numAssignmentsTried;
        int64_t timeSpentLocatingVehicles;
        int64_t tryAssignmentsTime;

        std::array<unsigned int, K> curDropoffIds;
        DistanceLabel curBestCostAsLabel;
        int endOfCurBatch;

        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TryToInsertDropoffAfterLastStop> dijSearchToDropoff;
        Subset vehiclesSeen;


    };

}