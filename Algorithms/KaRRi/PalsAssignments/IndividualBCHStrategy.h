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

            // Returns whether a given distance from a vehicle's last stop to the pickup cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel &distancesToPickups,
                                                   const bool considerPickupWalkingDists = false) const {
                assert(strat.curReqState->minDirectPDDist < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? strat.currentPickupWalkingDists : 0;

                const DistanceLabel directDist = strat.curReqState->minDirectPDDist;
                const auto detourTillDepAtPickup = distancesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                auto tripTimeTillDepAtPickup = detourTillDepAtPickup;
                tripTimeTillDepAtPickup.max(walkingDists);
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, walkingDists, *strat.curReqState);

                costLowerBound.setIf(DistanceLabel(INFTY), ~(distancesToPickups < INFTY));

                return strat.upperBoundCost < costLowerBound;
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the pickup cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToPickups needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtPickups,
                                                  const DistanceLabel &minDistancesToPickups) const {
                assert(strat.curReqState->minDirectPDDist < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtPickups[i] >= INFTY or
                    // minDistancesToPickups[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                }

                const DistanceLabel directDist = strat.curReqState->minDirectPDDist;
                const auto detourTillDepAtPickup = minDistancesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                auto depTimeAtPickup = arrTimesAtPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                const auto reqTime = DistanceLabel(strat.curReqState->originalRequest.requestTime);
                depTimeAtPickup.max(reqTime + strat.currentPickupWalkingDists);
                const auto tripTimeTillDepAtPickup = depTimeAtPickup - reqTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, strat.currentPickupWalkingDists, *strat.curReqState);

                costLowerBound.setIf(DistanceLabel(INFTY),
                                     ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY)));
                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                            const DistanceLabel &distancesToPickups) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const DistanceLabel directDist = strat.curReqState->minDirectPDDist;
                const auto detourTillDepAtPickup = distancesToPickups + InputConfig::getInstance().stopTime;
                const auto &stopIdx = strat.routeState.numStopsOf(vehId) - 1;
                const int vehDepTimeAtLastStop = time_utils::getVehDepTimeAtStopForRequest(vehId, stopIdx,
                                                                                           *strat.curReqState,
                                                                                           strat.routeState);
                auto depTimeAtPickups = vehDepTimeAtLastStop + distancesToPickups + InputConfig::getInstance().stopTime;
                depTimeAtPickups.max(strat.curPassengerArrTimesAtPickups);
                const auto tripTimeTillDepAtPickup = depTimeAtPickups - strat.curReqState->originalRequest.requestTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, strat.currentPickupWalkingDists,
                        *strat.curReqState);

                costLowerBound.setIf(INFTY, ~(distancesToPickups < INFTY));
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int vehId, const DistanceLabel &distancesToPickups) {
                assert(allSet(distancesToPickups >= 0));
                const DistanceLabel cost = calc.template calcUpperBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        strat.fleet[vehId], distancesToPickups, strat.curPassengerArrTimesAtPickups,
                        strat.curDistancesToDest,
                        strat.currentPickupWalkingDists, *strat.curReqState);

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
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const RouteState &routeState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(routeState),
                  routeState(routeState),
                  externalUpperBoundCost(INFTY),
                  distances(fleet.size()),
                  search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                         PickupAfterLastStopPruner(*this, calculator)),
                  vehiclesSeenForPickups(fleet.size()) {}


        void tryPickupAfterLastStop(RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs, stats::PalsAssignmentsPerformanceStats& stats) {
            runBchSearches(requestState, pdDistances, pdLocs, stats);
            enumerateAssignments(requestState, pdDistances, pdLocs, stats);
        }

        // Sets a known upper bound on the cost of a PALS insertion. Useful if IndividualBCHStrategy is used as
        // fallback for other strategy that provides an upper bound.
        void setExternalCostUpperBound(const int c) {
            externalUpperBoundCost = c;
        }

    private:

        // Run BCH searches that find distances from last stops to pickups
        void runBchSearches(RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs, stats::PalsAssignmentsPerformanceStats& stats) {
            Timer timer;

            initPickupSearches(requestState, pdLocs);
            for (int i = 0; i < pdLocs.numPickups(); i += K)
                runSearchesForPickupBatch(i, requestState, pdDistances, pdLocs);

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.searchTime += searchTime;
            stats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            stats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            stats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            stats.numCandidateVehicles += vehiclesSeenForPickups.size();
        }

        // Enumerate assignments with pickup after last stop
        void enumerateAssignments(RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs, stats::PalsAssignmentsPerformanceStats& stats) {
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

                for (auto &p: pdLocs.pickups) {
                    asgn.pickup = p;
                    asgn.distToPickup = getDistanceToPickup(vehId, asgn.pickup.id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    // Compute cost lower bound for this pickup specifically
                    const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
                    const auto vehTimeTillDepAtThisPickup = depTimeAtThisPickup -
                                                            getVehDepTimeAtStopForRequest(vehId, numStops - 1,
                                                                                          requestState, routeState);
                    const auto psgTimeTillDepAtThisPickup =
                            depTimeAtThisPickup - requestState.originalRequest.requestTime;
                    const auto minDirectDistForThisPickup = pdDistances.getMinDirectDistanceForPickup(asgn.pickup.id);
                    const auto minCost = calculator.calcCostForPairedAssignmentAfterLastStop(vehTimeTillDepAtThisPickup,
                                                                                             psgTimeTillDepAtThisPickup,
                                                                                             minDirectDistForThisPickup,
                                                                                             asgn.pickup.walkingDist,
                                                                                             0,
                                                                                             requestState);
                    if (minCost > requestState.getBestCost())
                        continue;

                    for (auto &d: pdLocs.dropoffs) {
                        asgn.dropoff = d;

                        // Try inserting pair with pickup after last stop:
                        ++numAssignmentsTried;
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                        requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    }
                }
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryAssignmentsTime += tryAssignmentsTime;
        }


        inline int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getDistance(vehId, pickupId);
        }

        void initPickupSearches(const RequestState& requestState, const PDLocs& pdLocs) {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            upperBoundCost = std::min(requestState.getBestCost(), externalUpperBoundCost);
            externalUpperBoundCost = INFTY;
            vehiclesSeenForPickups.clear();
            const int numPickupBatches = pdLocs.numPickups() / K + (pdLocs.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId, const RequestState& requestState, const PDDistancesT& pdDistances, const PDLocs& pdLocs) {
            assert(firstPickupId % K == 0 && firstPickupId < pdLocs.numPickups());


            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < pdLocs.numPickups() ? pdLocs.pickups[firstPickupId + i]
                                                                      : pdLocs.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup);
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
        CostCalculator calculator;
        const RouteState &routeState;

        int externalUpperBoundCost;
        int upperBoundCost;
        RequestState const * curReqState;

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
