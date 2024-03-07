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

#include <atomic>
#include <tbb/parallel_for.h>

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
                assert(strat.requestState.minDirectPDDist < INFTY);

                int currUpperBoundCost = strat.upperBoundCost.load(std::memory_order_relaxed);
                if (currUpperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? currentPickupWalkingDists : 0;

                const DistanceLabel directDist = strat.requestState.minDirectPDDist;
                const auto detourTillDepAtPickup = distancesToPickups + DistanceLabel(strat.inputConfig.stopTime);
                auto tripTimeTillDepAtPickup = detourTillDepAtPickup;
                tripTimeTillDepAtPickup.max(walkingDists);
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, walkingDists, strat.requestState);

                costLowerBound.setIf(DistanceLabel(INFTY), ~(distancesToPickups < INFTY));

                return currUpperBoundCost < costLowerBound;
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the pickup cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToPickups needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtPickups,
                                                  const DistanceLabel &minDistancesToPickups) const {
                assert(strat.requestState.minDirectPDDist < INFTY);

                int currUpperBoundCost = strat.upperBoundCost.load(std::memory_order_relaxed);
                if (currUpperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtPickups[i] >= INFTY or
                    // minDistancesToPickups[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                }

                const DistanceLabel directDist = strat.requestState.minDirectPDDist;
                const auto detourTillDepAtPickup = minDistancesToPickups + DistanceLabel(strat.inputConfig.stopTime);
                auto depTimeAtPickup = arrTimesAtPickups + DistanceLabel(strat.inputConfig.stopTime);
                const auto reqTime = DistanceLabel(strat.requestState.originalRequest.requestTime);
                depTimeAtPickup.max(reqTime + currentPickupWalkingDists);
                const auto tripTimeTillDepAtPickup = depTimeAtPickup - reqTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, currentPickupWalkingDists,
                        strat.requestState);

                costLowerBound.setIf(DistanceLabel(INFTY),
                                     ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY)));
                return currUpperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                           const DistanceLabel &distancesToPickups) {

                int currUpperBoundCost = strat.upperBoundCost.load(std::memory_order_relaxed);
                if (currUpperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const DistanceLabel directDist = strat.requestState.minDirectPDDist;
                const auto detourTillDepAtPickup = distancesToPickups + strat.inputConfig.stopTime;
                const auto &stopIdx = strat.routeState.numStopsOf(vehId) - 1;
                const int vehDepTimeAtLastStop = time_utils::getVehDepTimeAtStopForRequest(vehId, stopIdx,
                                                                                           strat.requestState,
                                                                                           strat.routeState);
                auto depTimeAtPickups = vehDepTimeAtLastStop + distancesToPickups + strat.inputConfig.stopTime;
                depTimeAtPickups.max(curPassengerArrTimesAtPickups);
                const auto tripTimeTillDepAtPickup = depTimeAtPickups - strat.requestState.originalRequest.requestTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, currentPickupWalkingDists,
                        strat.requestState);

                costLowerBound.setIf(INFTY, ~(distancesToPickups < INFTY));
                return currUpperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int vehId, const DistanceLabel &distancesToPickups) {
                assert(allSet(distancesToPickups >= 0));
                const DistanceLabel cost = calc.template calcUpperBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        strat.fleet[vehId], distancesToPickups, curPassengerArrTimesAtPickups,
                        curDistancesToDest,
                        currentPickupWalkingDists, strat.requestState);

                // Compare and exchange for atomic int
                const int minNewCost = cost.horizontalMin();
                auto &upperBoundCostAtomic = strat.upperBoundCost;
                int expectedUpperBoundCost = upperBoundCostAtomic.load(std::memory_order_relaxed);
                while (expectedUpperBoundCost > minNewCost && !upperBoundCostAtomic.compare_exchange_strong(
                        expectedUpperBoundCost, minNewCost, std::memory_order_relaxed));
            }

            bool isVehicleEligible(const int &) const {
                // All vehicles can perform PALS assignments.
                return true;
            }

        private:

            friend IndividualBCHStrategy;

            IndividualBCHStrategy &strat;
            DistanceLabel currentPickupWalkingDists;
            DistanceLabel curPassengerArrTimesAtPickups;
            DistanceLabel curDistancesToDest;
            const CostCalculator &calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:

        IndividualBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculator &pCalculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const PDDistancesT &pdDistances,
                              const RouteState &routeState,
                              RequestState &requestState,
                              const int &bestCostBeforeQuery,
                              const InputConfig &inputConfig)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(pCalculator),
                  pdDistances(pdDistances),
                  routeState(routeState),
                  requestState(requestState),
                  bestCostBeforeQuery(bestCostBeforeQuery),
                  inputConfig(inputConfig),
                  localBestCosts([&] { return requestState.getBestCost(); }),
                  localBestAssignments([&] { return requestState.getBestAssignment(); }),
                  localSearchTime(0),
                  localTryAssignmentsTime(0),
                  distances(fleet.size()),
                  threadLocalPruners(PickupAfterLastStopPruner(*this, calculator)),
                  search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                         threadLocalPruners),
                  vehiclesSeenForPickups(fleet.size()) {}

        void tryPickupAfterLastStop() {
            // Helper lambda to get sum of stats from thread local queries
            static const auto sumInts = [](const int &n1, const int &n2) { return n1 + n2; };

            Timer timer;
            numAssignmentsTried.store(0, std::memory_order_relaxed);
            initPickupSearches();

            tbb::parallel_for(int(0), static_cast<int>(requestState.numPickups()), K, [&](int i) {
                runBchSearchesAndEnumerate(i);
            });

            // Try assignment sequentially for local best assignment calculated by the individual thread
            for (auto &local : localBestAssignments)
                requestState.tryAssignment(local);

            const auto searchAndTryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();

            requestState.stats().palsAssignmentsStats.searchAndTryAssignmentsTime += searchAndTryAssignmentsTime;
            requestState.stats().palsAssignmentsStats.searchTimeLocal += localSearchTime.combine(sumInts);
            requestState.stats().palsAssignmentsStats.tryAssignmentsTimeLocal += localTryAssignmentsTime.combine(
                    sumInts);

            requestState.stats().palsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations.load(
                    std::memory_order_relaxed);
            requestState.stats().palsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled.load(
                    std::memory_order_relaxed);
            requestState.stats().palsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned.load(
                    std::memory_order_relaxed);
            requestState.stats().palsAssignmentsStats.numCandidateVehicles += vehiclesSeenForPickups.size();
            requestState.stats().palsAssignmentsStats.numAssignmentsTried += numAssignmentsTried.load(
                    std::memory_order_relaxed);

        }

    private:

        void initPickupSearches() {
            for (auto &local: localSearchTime)
                local = 0;
            for (auto &local: localTryAssignmentsTime)
                local = 0;

            for (auto &local: localBestCosts)
                local = requestState.getBestCost();
            for (auto &local: localBestAssignments)
                local = requestState.getBestAssignment();
                
            totalNumEdgeRelaxations.store(0);
            totalNumVerticesSettled.store(0);
            totalNumEntriesScanned.store(0);

            upperBoundCost.store(bestCostBeforeQuery);
            vehiclesSeenForPickups.clear();

            const int numPickupBatches = requestState.numPickups() / K + (requestState.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        // Run BCH searches and enumerate assignments within a thread
        void runBchSearchesAndEnumerate(const int firstPickupId) {
            Timer timer;
            runSearchesForPickupBatch(firstPickupId);
            localSearchTime.local() += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            enumeratePickupBatch(firstPickupId);
            localTryAssignmentsTime.local() += timer.elapsed<std::chrono::nanoseconds>();
        }

        inline int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getDistance(vehId, pickupId);
        }

        void runSearchesForPickupBatch(const int firstPickupId) {
            assert(firstPickupId % K == 0 && firstPickupId < requestState.numPickups());

            auto &localPruner = threadLocalPruners.local();

            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                localPruner.currentPickupWalkingDists[i] = pickup.walkingDist;
                localPruner.curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                localPruner.curDistancesToDest[i] = pdDistances.getDirectDistance(pickup.id, 0);
            }

            search.run(pickupTails, travelTimes);

            // After a search batch of K PDLocs, write the distances back to the global vectors
            distances.updateDistancesInGlobalVectors(firstPickupId);

            totalNumEdgeRelaxations.add_fetch(search.getLocalNumEdgeRelaxations(), std::memory_order_relaxed);
            totalNumVerticesSettled.add_fetch(search.getLocalNumVerticesSettled(), std::memory_order_relaxed);
            totalNumEntriesScanned.add_fetch(search.getLocalNumEntriesScanned(), std::memory_order_relaxed);

        }

        void enumeratePickupBatch(const int firstPickupId) {
            using namespace time_utils;

            int &localBestCost = localBestCosts.local();
            Assignment &localBestAssignment = localBestAssignments.local();
            
            Assignment asgn;
            int numAssignmentsTriedLocal = 0;

            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];

                enumeratePickup(pickup, localBestCost, localBestAssignment, numAssignmentsTriedLocal);
            }
            
            // Try assignment once for best assignment calculated by current thread
            // requestState.tryAssignment(localBestAssignment);

            numAssignmentsTried.add_fetch(numAssignmentsTriedLocal, std::memory_order_relaxed);
        }

        void enumeratePickup(const PDLoc &pickup, int &localBestCost, Assignment &localBestAssignment, int &numAssignmentsTriedLocal) {
            using namespace time_utils;
            Assignment asgn;
            asgn.pickup = &pickup;

            for (const auto &vehId: vehiclesSeenForPickups) {

                asgn.distToPickup = getDistanceToPickup(vehId, asgn.pickup->id);
                if (asgn.distToPickup >= INFTY)
                    continue;

                const int numStops = routeState.numStopsOf(vehId);
                if (numStops == 0)
                    continue;

                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = numStops - 1;
                asgn.dropoffStopIdx = numStops - 1;

                // Compute cost lower bound for this pickup specifically
                const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState,
                                                                            routeState, inputConfig);
                const auto vehTimeTillDepAtThisPickup = depTimeAtThisPickup -
                                                        getVehDepTimeAtStopForRequest(vehId, numStops - 1,
                                                                                        requestState, routeState);
                const auto psgTimeTillDepAtThisPickup =
                        depTimeAtThisPickup - requestState.originalRequest.requestTime;
                const auto minDirectDistForThisPickup = pdDistances.getMinDirectDistanceForPickup(asgn.pickup->id);
                const auto minCost = calculator.calcCostForPairedAssignmentAfterLastStop(vehTimeTillDepAtThisPickup,
                                                                                            psgTimeTillDepAtThisPickup,
                                                                                            minDirectDistForThisPickup,
                                                                                            asgn.pickup->walkingDist,
                                                                                            0,
                                                                                            requestState);
                if (minCost > localBestCost)
                    continue;

                for (auto &d: requestState.dropoffs) {
                    asgn.dropoff = &d;

                    // Try inserting pair with pickup after last stop:
                    ++numAssignmentsTriedLocal;
                    asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                    tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                
                }
            }
        }

        void tryAssignmentLocal(const Assignment &asgn, int &localBestCost, Assignment &localBestAssignment) const {

            const auto cost = calculator.calc(asgn, requestState);
            if (cost < localBestCost || (cost == localBestCost && breakCostTie(asgn, localBestAssignment))) {
                localBestCost = cost;
                localBestAssignment = asgn;
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const PDDistancesT &pdDistances;
        const RouteState &routeState;
        RequestState &requestState;
        const int &bestCostBeforeQuery;
        const InputConfig &inputConfig;

        std::atomic_int upperBoundCost;

        enumerable_thread_specific<int> localBestCosts;
        enumerable_thread_specific<Assignment> localBestAssignments;

        enumerable_thread_specific<int64_t> localSearchTime;
        enumerable_thread_specific<int64_t> localTryAssignmentsTime;

        TentativeLastStopDistances<LabelSetT> distances;
        tbb::enumerable_thread_specific<PickupAfterLastStopPruner> threadLocalPruners;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        ThreadSafeSubset vehiclesSeenForPickups;

        CAtomic<int> numAssignmentsTried;
        CAtomic<int> totalNumEdgeRelaxations;
        CAtomic<int> totalNumVerticesSettled;
        CAtomic<int> totalNumEntriesScanned;

    };

}
