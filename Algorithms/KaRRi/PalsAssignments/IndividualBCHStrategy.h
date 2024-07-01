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

    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsEnvT, typename PDDistancesT, typename LabelSetT>
    class IndividualBCHStrategy {

        static constexpr int K = LabelSetT::K;
        using LabelMask = typename LabelSetT::LabelMask;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        struct PickupAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = true;

            PickupAfterLastStopPruner(IndividualBCHStrategy &strat, const CostCalculator &calc)
                    : strat(strat), calc(calc) {}

            // Returns whether a given cost from a vehicle's last stop to the pickup cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop cost greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesNotAdmitBestAsgn(const DistanceLabel &costsToPickups,
                                                   const DistanceLabel &travelTimesToPickups,
                                                   const bool considerPickupWalkingDists = false) const {
                KASSERT(strat.requestState.minDirectPDCost < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with costsToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(costsToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? strat.currentPickupWalkingDists : 0;

                const DistanceLabel directTravelTime = strat.requestState.minDirectPDTravelTime;
                auto tripTimeTillDepAtPickup =
                        travelTimesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                tripTimeTillDepAtPickup.max(walkingDists);
                using F = CostCalculator::CostFunction;
                const auto tripVioCost = F::calcKTripTimeViolationCosts(tripTimeTillDepAtPickup + directTravelTime,
                                                                        strat.requestState);
                const auto vehCost = F::calcKVehicleCosts(costsToPickups + strat.requestState.minDirectPDCost);
                auto costLowerBound = vehCost + tripVioCost;
                costLowerBound.setIf(DistanceLabel(INFTY), ~(costsToPickups < INFTY));

                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                           const DistanceLabel &costsToPickups,
                                                           const DistanceLabel &travelTimesToPickups) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with costsToPickups[i] >= INFTY are worse than
                    // the current best.
                    return ~(costsToPickups < INFTY);
                }

                const auto &stopIdx = strat.routeState.numStopsOf(vehId) - 1;
                const int vehDepTimeAtLastStop = time_utils::getVehDepTimeAtStopForRequest(vehId, stopIdx,
                                                                                           strat.requestState,
                                                                                           strat.routeState);
                auto depTimeAtPickups =
                        vehDepTimeAtLastStop + travelTimesToPickups + InputConfig::getInstance().stopTime;
                depTimeAtPickups.max(strat.curPassengerArrTimesAtPickups);
                const auto tripTimeTillDepAtPickup = depTimeAtPickups - strat.requestState.originalRequest.requestTime;
                using F = CostCalculator::CostFunction;
                const auto waitVioCost = F::calcKWaitViolationCosts(depTimeAtPickups, strat.requestState);
                const auto tripVioCost = F::calcKTripTimeViolationCosts(
                        tripTimeTillDepAtPickup + strat.requestState.minDirectPDTravelTime, strat.requestState);
                const auto vehCost = F::calcKVehicleCosts(costsToPickups + strat.requestState.minDirectPDCost);

                auto costLowerBound = vehCost + waitVioCost + tripVioCost;
                costLowerBound.setIf(INFTY, ~(costsToPickups < INFTY));
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int vehId,
                                      const DistanceLabel &costsToPickups,
                                      const DistanceLabel &travelTimesToPickups) {
                KASSERT(allSet(costsToPickups >= 0));
                const DistanceLabel cost = calc.template calcUpperBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        strat.fleet[vehId],
                        costsToPickups,
                        travelTimesToPickups,
                        strat.curPassengerArrTimesAtPickups,
                        strat.currentPickupWalkingDists,
                        strat.curCostsToDropoffForUpperBound,
                        strat.curTravelTimesToDropoffForUpperBound,
                        strat.dropoffForUpperBoundWalkingDist,
                        strat.requestState);

                strat.upperBoundCost = std::min(strat.upperBoundCost, cost.horizontalMin());
            }

            bool isVehicleEligible(const int &) const {
                // All vehicles can perform PALS assignments.
                return true;
            }

        private:
            IndividualBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:

        IndividualBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculator &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const PDDistancesT &pdDistances,
                              const RouteState &routeState,
                              RequestState &requestState,
                              const int &bestCostBeforeQuery)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  pdDistances(pdDistances),
                  routeState(routeState),
                  requestState(requestState),
                  bestCostBeforeQuery(bestCostBeforeQuery),
                  distances(fleet.size()),
                  search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                         PickupAfterLastStopPruner(*this, calculator)),
                  vehiclesSeenForPickups(fleet.size()) {}

        void tryPickupAfterLastStop() {
            runBchSearches();
            enumerateAssignments();
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
        void enumerateAssignments() {
            using namespace time_utils;


            int numAssignmentsTried = 0;
            Timer timer;

            Assignment asgn;
            for (const auto &vehId: vehiclesSeenForPickups) {

                const int numStops = routeState.numStopsOf(vehId);
                if (numStops == 0)
                    continue;

                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = numStops - 1;
                asgn.dropoffStopIdx = numStops - 1;

                for (auto &p: requestState.pickups) {
                    asgn.pickup = &p;
                    asgn.costToPickup = getCostToPickup(vehId, p.id);
                    asgn.travelTimeToPickup = getTravelTimeToPickup(vehId, p.id);
                    if (asgn.costToPickup >= INFTY)
                        continue;

                    // Compute cost lower bound for this pickup specifically
                    const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
                    const auto waitTime = depTimeAtThisPickup - requestState.originalRequest.requestTime;
                    const auto minTripTime = waitTime + pdDistances.getMinTravelTimeForPickup(p.id);
                    const auto minPdCostForThisPickup = pdDistances.getMinCostForPickup(p.id);
                    const auto minCost = calculator.calcCostForPairedAssignmentAfterLastStop(
                            asgn.costToPickup, minPdCostForThisPickup, minTripTime, waitTime, requestState);
                    if (minCost > requestState.getBestCost())
                        continue;

                    for (auto &d: requestState.dropoffs) {
                        asgn.dropoff = &d;

                        // Try inserting pair with pickup after last stop:
                        ++numAssignmentsTried;
                        asgn.costToDropoff = pdDistances.getCost(p, d);
                        asgn.travelTimeToDropoff = pdDistances.getTravelTime(p, d);
                        requestState.tryAssignment(asgn);
                    }
                }
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().palsAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().palsAssignmentsStats.tryAssignmentsTime += tryAssignmentsTime;
        }


        inline int getCostToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getCost(vehId, pickupId);
        }

        inline int getTravelTimeToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getTravelTime(vehId, pickupId);
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
            KASSERT(firstPickupId % K == 0 && firstPickupId < requestState.numPickups());


            std::array<int, K> pickupTails;
            std::array<int, K> costOffsets;
            std::array<int, K> travelTimeOffsets;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                costOffsets[i] = inputGraph.traversalCost(pickup.loc);
                travelTimeOffsets[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                // To compute upper bounds on costs during the search, we arbitrarily combine each pickup with dropoff 0
                // (any dropoff would be possible).
                curCostsToDropoffForUpperBound[i] = pdDistances.getCost(pickup.id, 0);
                curTravelTimesToDropoffForUpperBound[i] = pdDistances.getTravelTime(pickup.id, 0);
                dropoffForUpperBoundWalkingDist[i] = requestState.dropoffs[0].walkingDist;
            }

            distances.setCurBatchIdx(firstPickupId / K);
            search.run(pickupTails, costOffsets, travelTimeOffsets);

            totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            totalNumVerticesSettled += search.getNumVerticesSettled();
            totalNumEntriesScanned += search.getNumEntriesScanned();
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const PDDistancesT &pdDistances;
        const RouteState &routeState;
        RequestState &requestState;
        const int &bestCostBeforeQuery;

        int upperBoundCost;

        TentativeLastStopDistances <LabelSetT> distances;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        Subset vehiclesSeenForPickups;
        DistanceLabel currentPickupWalkingDists;
        DistanceLabel curPassengerArrTimesAtPickups;

        DistanceLabel curCostsToDropoffForUpperBound;
        DistanceLabel curTravelTimesToDropoffForUpperBound;
        DistanceLabel dropoffForUpperBoundWalkingDist;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

    };

}
