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

#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Tools/Timer.h"

namespace karri::PickupAfterLastStopStrategies {

    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsEnvT, typename PDDistancesT, typename LabelSetT, typename CostCalculatorT>
    class IndividualBCHStrategy {

        static constexpr int K = LabelSetT::K;
        using LabelMask = typename LabelSetT::LabelMask;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        struct PickupAfterLastStopPruner {

            PickupAfterLastStopPruner(IndividualBCHStrategy &strat, const CostCalculatorT &calc)
                    : strat(strat), calc(calc) {}

            LabelMask isWorseThanUpperBoundCost(const DistanceLabel &distancesToPickups,
                                      const bool considerPickupWalkingDists = false) const {
                assert(strat.requestState.minDirectPDDist < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? strat.currentPickupWalkingDists : 0;

                const DistanceLabel directDists = strat.requestState.minDirectPDDist;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        distancesToPickups, directDists, walkingDists, strat.requestState);

                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int vehId, const DistanceLabel &distancesToPickups) {
                const DistanceLabel cost = calc.template calcUpperBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        strat.fleet[vehId], distancesToPickups, strat.curPassengerArrTimesAtPickups,
                        strat.curDistancesToDest,
                        strat.currentPickupWalkingDists, strat.requestState);

                strat.upperBoundCost = std::min(strat.upperBoundCost, cost.horizontalMin());
            }

            bool isVehicleEligible(const int &) const {
                // All vehicles can perform PALS assignments.
                return true;
            }

        private:
            IndividualBCHStrategy &strat;
            const CostCalculatorT &calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:

        IndividualBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculatorT &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const PDDistancesT &pdDistances,
                              RequestState<CostCalculatorT> &requestState,
                              const int& bestCostBeforeQuery,
                              const InputConfig &inputConfig)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  pdDistances(pdDistances),
                  requestState(requestState),
                  bestCostBeforeQuery(bestCostBeforeQuery),
                  inputConfig(inputConfig),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  distances(fleet.size()),
                  search(distances, chEnv, vehiclesSeenForPickups, PickupAfterLastStopPruner(*this, calculator)),
                  vehiclesSeenForPickups(fleet.size()) {}

        void tryPickupAfterLastStop(const RouteStateData &routeStateData) {
            search.exchangeBuckets(lastStopBucketsEnv.getBuckets(routeStateData.getTypeOfData()));
            runBchSearches();
            enumerateAssignments(routeStateData);
        }

    private:

        // Run BCH searches that find distances from last stops to pickups
        void runBchSearches() {
            Timer timer;

            initPickupSearches();
            for (int i = 0; i < requestState.numPickups(); i += K)
                runSearchesForPickupBatch(i);

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().palsAssignmentsStats.searchTime += searchTime;
            requestState.stats().palsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            requestState.stats().palsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            requestState.stats().palsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            requestState.stats().palsAssignmentsStats.numCandidateVehicles += vehiclesSeenForPickups.size();
        }

        // Enumerate assignments with pickup after last stop
        void enumerateAssignments(const RouteStateData &routeStateData) {
            using namespace time_utils;


            int numAssignmentsTried = 0;
            Timer timer;

            Assignment asgn;
            for (const auto &vehId: vehiclesSeenForPickups) {

                const int numStops = routeStateData.numStopsOf(vehId);
                if (numStops == 0)
                    continue;

                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = numStops - 1;
                asgn.dropoffStopIdx = numStops - 1;

                for (auto &p: requestState.pickups) {
                    asgn.pickup = &p;
                    asgn.distToPickup = getDistanceToPickup(vehId, asgn.pickup->id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    // Compute cost lower bound for this pickup specifically
                    const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState,
                                                                              routeStateData, inputConfig);
                    const auto vehTimeTillDepAtThisPickup = depTimeAtThisPickup -
                                                            getVehDepTimeAtStopForRequest(vehId, numStops - 1,
                                                                                          requestState, routeStateData);
                    const auto psgTimeTillDepAtThisPickup =
                            depTimeAtThisPickup - requestState.originalRequest.requestTime;
                    const auto minDirectDistForThisPickup = pdDistances.getMinDirectDistanceForPickup(asgn.pickup->id);
                    const auto minCost = calculator.calcCostForPairedAssignmentAfterLastStop(vehTimeTillDepAtThisPickup,
                                                                                             psgTimeTillDepAtThisPickup,
                                                                                             minDirectDistForThisPickup,
                                                                                             asgn.pickup->walkingDist, 0,
                                                                                             requestState);
                    if (minCost > requestState.getBestCost())
                        continue;

                    for (auto &d: requestState.dropoffs) {
                        asgn.dropoff = &d;

                        // Try inserting pair with pickup after last stop:
                        ++numAssignmentsTried;
                        asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        requestState.tryAssignment(asgn);
                    }
                }
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().palsAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().palsAssignmentsStats.tryAssignmentsTime += tryAssignmentsTime;
        }


        inline int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getDistance(vehId, pickupId);
        }

        void initPickupSearches() {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            upperBoundCost = bestCostBeforeQuery;
            vehiclesSeenForPickups.clear();
            const int numPickupBatches = requestState.numPickups() / K + (requestState.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId) {
            assert(firstPickupId % K == 0 && firstPickupId < requestState.numPickups());


            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                curDistancesToDest[i] = pdDistances.getDirectDistance(pickup.id, 0);
            }

            distances.setCurBatchIdx(firstPickupId / K);
            search.run(pickupTails, travelTimes);

            totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            totalNumVerticesSettled += search.getNumVerticesSettled();
            totalNumEntriesScanned += search.getNumEntriesScanned();
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculatorT &calculator;
        const PDDistancesT &pdDistances;
        RequestState<CostCalculatorT> &requestState;
        const int& bestCostBeforeQuery;
        const InputConfig &inputConfig;

        const LastStopBucketsEnvT &lastStopBucketsEnv;

        int upperBoundCost;

        TentativeLastStopDistances<LabelSetT> distances;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        Subset vehiclesSeenForPickups;
        DistanceLabel currentPickupWalkingDists;
        DistanceLabel curPassengerArrTimesAtPickups;
        DistanceLabel curDistancesToDest;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

    };

}
