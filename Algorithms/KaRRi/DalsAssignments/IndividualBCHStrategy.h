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
#include "DataStructures/Utilities/Permutation.h"

#include "DataStructures/Containers/Parallel/ThreadSafeFastResetFlagArray.h"
#include "Parallel/atomic_wrapper.h"

#include <atomic>
#include <tbb/parallel_for.h>

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

            DropoffAfterLastStopPruner(const IndividualBCHStrategy &strat,
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

                const DistanceLabel walkingDists = considerWalkingDists ? currentDropoffWalkingDists : 0;
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
                        currentDropoffWalkingDists, minDistancesToDropoffs, arrTimesAtDropoffs, strat.requestState);

                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                           const DistanceLabel &distancesToDropoffs) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToDropoffs < INFTY);
                }

                const DistanceLabel costLowerBound = calc.template calcKVehicleDependentCostLowerBoundsForDALSWithKnownDistToDropoff<LabelSet>(
                        vehId, currentDropoffWalkingDists, distancesToDropoffs, 0, strat.requestState);
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
                       (strat.relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId) ||
                        strat.relevantOrdinaryPickups.hasRelevantSpotsFor(vehId));
            }

        private:
            friend IndividualBCHStrategy;

            const IndividualBCHStrategy &strat;
            DistanceLabel currentDropoffWalkingDists;
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
                              RequestState &requestState,
                              const RelevantPDLocs &relevantOrdinaryPickups,
                              const RelevantPDLocs &relevantPickupsBeforeNextStop)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  curVehLocToPickupSearches(curVehLocToPickupSearchesT),
                  routeState(routeState),
                  requestState(requestState),
                  relevantOrdinaryPickups(relevantOrdinaryPickups),
                  relevantPickupsBeforeNextStop(relevantPickupsBeforeNextStop),
                  checkPBNSForVehicle(fleet.size()),
                  localBestCosts([&] { return requestState.getBestCost(); }),
                  localBestAssignments([&] { return requestState.getBestAssignment(); }),
                  localPruners(DropoffAfterLastStopPruner(*this, calculator)),
                  search(lastStopBucketsEnv, lastStopDistances, chEnv, routeState, localPruners),
                  lastStopDistances(fleet.size()),
                  localSearchTime(0),
                  localTryAssignmentsTime(0),
                  relevantVehiclesPBNSOrder([&] {
                      return Permutation::getRandomPermutation(
                              relevantPickupsBeforeNextStop.getVehiclesWithRelevantPDLocs().size(),
                              std::minstd_rand(seedCounter.fetch_add(1, std::memory_order_relaxed)));
                  }) {}

        void init() {
            curVehLocToPickupSearches.initialize();
        }

        void tryDropoffAfterLastStop() {
            // Helper lambda to get sum of stats from thread local queries
            static const auto sumInts = [](const int &n1, const int &n2) { return n1 + n2; };

            Timer timer;
            numAssignmentsTried.store(0, std::memory_order_relaxed);
            initDropoffSearches();

            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();

            tbb::parallel_for(int(0), static_cast<int>(requestState.numDropoffs()), K, [&](int i)
            {
                runBchSearchesAndEnumerate(i);
            });
            
            // Try assignment sequentially for local best assignment calculated by the individual thread
            for (const auto &asgn: localBestAssignments) {
                if (asgn.vehicle && asgn.pickup && asgn.dropoff)
                    requestState.tryAssignment(asgn);
            }

            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;


            const auto searchAndTryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();

            requestState.stats().dalsAssignmentsStats.searchAndTryAssignmentsTime += searchAndTryAssignmentsTime;
            requestState.stats().dalsAssignmentsStats.searchTimeLocal += localSearchTime.combine(sumInts);
            requestState.stats().dalsAssignmentsStats.tryAssignmentsTimeLocal += (
                    localTryAssignmentsTime.combine(sumInts) - pbnsTime);
            requestState.stats().pbnsAssignmentsStats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            requestState.stats().pbnsAssignmentsStats.directCHSearchTimeLocal += curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();


            // Find total number of candidate dropoffs for statistics
            int totalNumberOfCandidateDropoffs = 0;
            for (const auto &vehId: lastStopDistances.getVehiclesSeen())
                for (const auto &dropoff: requestState.dropoffs)
                    totalNumberOfCandidateDropoffs += (getDistanceToDropoff(vehId, dropoff.id) < INFTY);
            requestState.stats().dalsAssignmentsStats.numCandidateDropoffsAcrossAllVehicles += totalNumberOfCandidateDropoffs;

            requestState.stats().dalsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations.load(
                    std::memory_order_relaxed);
            requestState.stats().dalsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled.load(
                    std::memory_order_relaxed);
            requestState.stats().dalsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned.load(
                    std::memory_order_relaxed);
            requestState.stats().dalsAssignmentsStats.numCandidateVehicles += lastStopDistances.getVehiclesSeen().size();
            requestState.stats().dalsAssignmentsStats.numAssignmentsTried += numAssignmentsTried.load(
                    std::memory_order_relaxed);
        }

    private:

        // Run BCH searches and enumerate assignments within a thread
        void runBchSearchesAndEnumerate(const unsigned int firstDropoffId) {
            Timer timer;
            runSearchesForDropoffBatch(firstDropoffId);
            localSearchTime.local() += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            enumerateDropoffBatch(firstDropoffId);
            localTryAssignmentsTime.local() += timer.elapsed<std::chrono::nanoseconds>();

        }

        inline int getDistanceToDropoff(const int vehId, const unsigned int dropoffId) {
            return lastStopDistances.getDistance(vehId, dropoffId);
        }

        void initDropoffSearches() {
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

            upperBoundCost = requestState.getBestCost();
            checkPBNSForVehicle.reset();
            relevantVehiclesPBNSOrder.clear();

            // Construct more space for dropoff labels if needed.
            const int numDropoffBatches =
                    requestState.numDropoffs() / K + (requestState.numDropoffs() % K != 0);
            lastStopDistances.init(numDropoffBatches);

        }

        void runSearchesForDropoffBatch(const unsigned int firstDropoffId) {
            assert(firstDropoffId % K == 0 && firstDropoffId < requestState.numDropoffs());

            auto &localPruner = localPruners.local();

            std::array<int, K> dropoffTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &dropoff =
                        firstDropoffId + i < requestState.numDropoffs() ? requestState.dropoffs[firstDropoffId + i]
                                                                        : requestState.dropoffs[firstDropoffId];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                travelTimes[i] = inputGraph.travelTime(dropoff.loc);
                localPruner.currentDropoffWalkingDists[i] = dropoff.walkingDist;
            }

            search.run(dropoffTails, travelTimes);

            // After a search batch of K PDLocs, write the distances back to the global vectors
            lastStopDistances.updateDistancesInGlobalVectors(firstDropoffId);

            totalNumEdgeRelaxations.add_fetch(search.getLocalNumEdgeRelaxations(), std::memory_order_relaxed);
            totalNumVerticesSettled.add_fetch(search.getLocalNumVerticesSettled(), std::memory_order_relaxed);
            totalNumEntriesScanned.add_fetch(search.getLocalNumEntriesScanned(), std::memory_order_relaxed);

        }

        void enumerateDropoffBatch(const int firstDropoffId) {

            int &localBestCost = localBestCosts.local();
            Assignment &localBestAssignment = localBestAssignments.local();

            Assignment asgn;
            int numAssignmentsTriedLocal = 0;

            for (int i = 0; i < K; ++i) {
                const auto &dropoff =
                        firstDropoffId + i < requestState.numDropoffs() ? requestState.dropoffs[firstDropoffId + i]
                                                                        : requestState.dropoffs[firstDropoffId];
                enumerateDropoffWithOrdinaryPickup(dropoff, localBestCost, localBestAssignment, numAssignmentsTriedLocal);
                enumerateDropoffWithPBNS(dropoff, localBestCost, localBestAssignment, numAssignmentsTriedLocal);
            }

            numAssignmentsTried.add_fetch(numAssignmentsTriedLocal, std::memory_order_relaxed);
        }

        void enumerateDropoffWithOrdinaryPickup(const PDLoc &dropoff, int &localBestCost, Assignment &localBestAssignment, int &numAssignmentsTriedLocal) {
            Assignment asgn;
            asgn.dropoff = &dropoff;

            for (const auto &vehId: lastStopDistances.getVehiclesSeen()) {

                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId, true);
                    continue;
                }

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(
                        vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                if (asgn.distToDropoff >= INFTY)
                    continue; // no need to check pickup before next stop

                assert(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                int curPickupIndex = numStops - 1;
                auto pickupIt = relevantPickupsInRevOrder.begin();
                for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                    const auto &entry = *pickupIt;
                    const auto stopIdx = routeState.stopPositionOf(entry.stopId);

                    if (stopIdx < curPickupIndex) {
                        // New smaller pickup index reached: Check if seating capacity and cost lower bound admit
                        // any valid assignments at this or earlier indices.
                        if (occupancies[stopIdx] + requestState.originalRequest.numRiders >
                            asgn.vehicle->capacity)
                            break;

                        assert(stopIdx < numStops - 1);
                        const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                           routeState.schedArrTimesFor(vehId)[stopIdx + 1];

                        const auto minCostFromHere = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                                asgn.dropoff->walkingDist, asgn.distToDropoff, minTripTimeToLastStop, requestState);
                        if (minCostFromHere > localBestCost)
                            break;

                        curPickupIndex = stopIdx;
                    }

                    asgn.pickup = &requestState.pickups[entry.pdId];
                    if (asgn.pickup->loc == asgn.dropoff->loc)
                        continue;
                    ++numAssignmentsTriedLocal;
                    asgn.pickupStopIdx = stopIdx;
                    asgn.distToPickup = entry.distToPDLoc;
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;

                    tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                }

                if (pickupIt == relevantPickupsInRevOrder.end()) {
                    // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                    // need to consider the pickup before next stop case.
                    checkPBNSForVehicle.set(vehId, true);
                }
            }
        }

        void enumerateDropoffWithPBNS(const PDLoc &dropoff, int &localBestCost, Assignment &localBestAssignment, int &numAssignmentsTriedLocal) {
            Assignment asgn;
            asgn.pickupStopIdx = 0;
            asgn.dropoff = &dropoff;

            const auto &relVehicles = relevantPickupsBeforeNextStop.getVehiclesWithRelevantPDLocs();
            for (const auto &permIdx: relevantVehiclesPBNSOrder.local()) {

                const auto vehId = *(relVehicles.begin() + permIdx);

                if (!lastStopDistances.getVehiclesSeen().contains(vehId))
                    continue;

                if (!checkPBNSForVehicle[vehId])
                    continue;

                if (routeState.numStopsOf(vehId) == 0 ||
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders >
                    fleet[vehId].capacity)
                    continue;

                const auto numStops = routeState.numStopsOf(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                for (auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {

                    asgn.pickup = &requestState.pickups[entry.pdId];
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;
                    if (asgn.pickup->loc == asgn.dropoff->loc)
                        continue;

                    asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                    if (asgn.distToDropoff >= INFTY)
                        continue;

                    if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup->id)) {
                        asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup->id);

                        tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                        ++numAssignmentsTriedLocal;
                        continue;
                    }

                    asgn.distToPickup = entry.distToPDLoc;
                    const auto lowerBoundCost = calculator.calc(asgn, requestState);
                    if (lowerBoundCost < localBestCost || (lowerBoundCost == localBestCost &&
                                                           breakCostTie(asgn, localBestAssignment))) {
                        // In this case, we need the exact distance to the pickup via the current location of the
                        // vehicle.
                        curVehLocToPickupSearches.computeExactDistancesVia(fleet[asgn.vehicle->vehicleId],
                                                                            asgn.pickup->id, asgn.distToPickup);

                        assert(asgn.pickup->id >= 0 && asgn.pickup->id < requestState.numPickups());
                        assert(asgn.dropoff->id >= 0 && asgn.dropoff->id < requestState.numDropoffs());

                        asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId,
                                                                                    asgn.pickup->id);
                        if (asgn.distToPickup >= INFTY)
                            continue;

                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;

                        asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff->id);
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        ++numAssignmentsTriedLocal;
                        asgn.dropoffStopIdx = numStops - 1;

                        tryAssignmentLocal(asgn, localBestCost, localBestAssignment);
                    }
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
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const RouteState &routeState;
        RequestState &requestState;
        const RelevantPDLocs &relevantOrdinaryPickups;
        const RelevantPDLocs &relevantPickupsBeforeNextStop;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        ThreadSafeFastResetFlagArray<> checkPBNSForVehicle;

        int upperBoundCost;

        tbb::enumerable_thread_specific<int> localBestCosts;
        tbb::enumerable_thread_specific<Assignment> localBestAssignments;

        // Vehicles seen by any last stop search
        tbb::enumerable_thread_specific<DropoffAfterLastStopPruner> localPruners;
        DropoffBCHQuery search;
        TentativeLastStopDistances<LabelSet> lastStopDistances;

        tbb::enumerable_thread_specific<int64_t> localSearchTime;
        tbb::enumerable_thread_specific<int64_t> localTryAssignmentsTime;

        // Counter for generating differing seeds for random permutations between threads
        std::atomic_int seedCounter;
        // Each thread generates one random permutation of thread ids. The permutation defines the order in which
        // a threads local results are written to the global result. This helps to alleviate contention on the
        // spin locks (separate per stop id) used to synchronize global writes.
        tbb::enumerable_thread_specific<Permutation> relevantVehiclesPBNSOrder;


        CAtomic<int> numAssignmentsTried;
        CAtomic<int> totalNumEdgeRelaxations;
        CAtomic<int> totalNumVerticesSettled;
        CAtomic<int> totalNumEntriesScanned;

    };

}