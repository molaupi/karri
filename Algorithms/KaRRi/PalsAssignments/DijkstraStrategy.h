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

#include "Tools/Timer.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/KaRRi/LastStopSearches/TentativeLastStopDistances.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"

namespace karri::PickupAfterLastStopStrategies {

    template<typename InputGraphT, typename PDDistancesT, typename DijLabelSet, typename LastStopsAtVerticesT>
    struct DijkstraStrategy {

    private:

        static constexpr int K = DijLabelSet::K;
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;

        struct TryToInsertPickupAfterLastStop {
            TryToInsertPickupAfterLastStop(DijkstraStrategy &strategy,
                                           const CostCalculator &calculator,
                                           const RequestState &requestState)
                    : strategy(strategy),
                      calculator(calculator),
                      requestState(requestState) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {

                const DistanceLabel costLowerBound = calculator.calcCostLowerBoundForKPickupsAfterLastStopIndependentOfVehicle(
                        distFromV, DistanceLabel(requestState.minDirectPDDist), requestState);
                const LabelMask bestCostExceeded = DistanceLabel(strategy.upperBoundCost) < costLowerBound;
                if (allSet(bestCostExceeded))
                    return true;

                strategy.updateDistancesForLastStopsAt(v, distFromV);
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
                         const RouteStateData &routeState,
                         const LastStopsAtVerticesT &lastStopsAtVertices,
                         const CostCalculator &calculator,
                         const PDDistancesT &pdDistances,
                         const RequestState &requestState,
                         stats::PalsAssignmentsPerformanceStats& stats)
                : inputGraph(inputGraph),
                  reverseGraph(reverseGraph),
                  requestState(requestState),
                  stats(stats),
                  pdDistances(pdDistances),
                  calculator(calculator),
                  fleet(fleet),
                  routeState(routeState),
                  lastStopsAtVertices(lastStopsAtVertices),
                  dijSearchToPickup(reverseGraph, {*this, calculator, requestState}),
                  lastStopDistances(fleet.size()),
                  vehiclesSeen(fleet.size()) {}

        void tryPickupAfterLastStop(BestAsgn& bestAsgn) {
            runDijkstraSearches(bestAsgn);
            enumerateAssignments(bestAsgn);
        }

    private:

        void runDijkstraSearches(BestAsgn& bestAsgn) {
            numLastStopsVisited = 0;
            upperBoundCost = bestAsgn.cost();
            const int numBatches = requestState.numPickups() / K + (requestState.numPickups() % K != 0);
            lastStopDistances.init(numBatches);
            vehiclesSeen.clear();

            int numEdgeRelaxations = 0;
            int numVerticesSettled = 0;

            Timer timer;

            for (unsigned int i = 0; i < numBatches; ++i) {
                runSearchesForPickupBatch(i);

                numEdgeRelaxations += dijSearchToPickup.getNumEdgeRelaxations();
                numVerticesSettled += dijSearchToPickup.getNumVerticesSettled();
            }

            const int64_t searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numEdgeRelaxationsInSearchGraph += numEdgeRelaxations;
            stats.numVerticesOrLabelsSettled += numVerticesSettled;
            stats.numEntriesOrLastStopsScanned += numLastStopsVisited;
            stats.searchTime += searchTime;
            stats.numCandidateVehicles += numLastStopsVisited;
        }

        // Enumerate PALS assignments:
        void enumerateAssignments(BestAsgn& bestAsgn) {
            using namespace time_utils;
            int numAssignmentsTried = 0;
            Timer timer;

            Assignment asgn;
            for (const auto &vehId: vehiclesSeen) {

                const int numStops = routeState.numStopsOf(vehId);
                if (numStops == 0)
                    continue;

                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = numStops - 1;
                asgn.dropoffStopIdx = numStops - 1;

                for (auto &p: requestState.pickups) {
                    asgn.pickup = &p;
                    asgn.distToPickup = lastStopDistances.getDistance(vehId, asgn.pickup->id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    // Compute cost lower bound for this pickup specifically
                    const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
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
                    if (minCost > bestAsgn.cost())
                        continue;


                    for (auto &d: requestState.dropoffs) {
                        asgn.dropoff = &d;

                        // Try inserting pair with pickup after last stop:
                        ++numAssignmentsTried;
                        asgn.distToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        bestAsgn.tryAssignment(asgn);
                    }
                }
            }

            const int64_t enumAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryAssignmentsTime += enumAssignmentsTime;
        }

        void runSearchesForPickupBatch(const int batchIdx) {

            std::array<int, K> pickupTails;
            std::array<int, K> offsets;
            for (int i = 0; i < K; ++i) {
                curPickupIds[i] = batchIdx * K + i;
                if (curPickupIds[i] >= requestState.numPickups())
                    curPickupIds[i] = batchIdx * K; // fill last batch with copies of first pickup in batch
                const auto &pickup = requestState.pickups[curPickupIds[i]];
                curWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                curMinDirectDistances[i] = pdDistances.getMinDirectDistanceForPickup(pickup.id);
                curDistancesToDest[i] = pdDistances.getDirectDistance(pickup.id, 0);
                assert(pdDistances.getMinDirectDistance() <= curMinDirectDistances[i]);
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                offsets[i] = inputGraph.travelTime(pickup.loc);
            }

            // Search accesses the batch through curPickupIds
            lastStopDistances.setCurBatchIdx(batchIdx);
            dijSearchToPickup.runWithOffset(pickupTails, offsets);
        }


        void updateDistancesForLastStopsAt(const int v, const DistanceLabel &distFromV) {

            if (!lastStopsAtVertices.isAnyLastStopAtVertex(v))
                return;

            const auto minCost =
                    calculator.calcLowerBoundCostForKPairedAssignmentsAfterLastStop<DijLabelSet>(distFromV,
                                                                                                 curMinDirectDistances,
                                                                                                 curWalkingDists,
                                                                                                 requestState);
            LabelMask notExceedingUpperBound = ~(DistanceLabel(upperBoundCost) < minCost);

            if (!anySet(notExceedingUpperBound))
                return;

            for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(v)) {
                ++numLastStopsVisited;
                lastStopDistances.setDistancesForCurBatchIf(vehId, distFromV, notExceedingUpperBound);
                vehiclesSeen.insert(vehId);

                const DistanceLabel cost = calculator.calcUpperBoundCostForKPairedAssignmentsAfterLastStop<DijLabelSet>(
                        fleet[vehId], distFromV, curPassengerArrTimesAtPickups,
                        curDistancesToDest, curWalkingDists, requestState);

                upperBoundCost = std::min(upperBoundCost, cost.horizontalMin());
            }
        }

        const InputGraphT &inputGraph;
        const InputGraphT &reverseGraph;
        const RequestState &requestState;
        stats::PalsAssignmentsPerformanceStats& stats;

        std::array<unsigned int, K> curPickupIds;
        DistanceLabel curWalkingDists;
        DistanceLabel curMinDirectDistances;
        DistanceLabel curDistancesToDest;
        DistanceLabel curPassengerArrTimesAtPickups;
        int upperBoundCost;

        int numLastStopsVisited;

        const PDDistancesT &pdDistances;
        const CostCalculator &calculator;
        const Fleet &fleet;
        const RouteStateData &routeState;
        const LastStopsAtVerticesT &lastStopsAtVertices;


        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TryToInsertPickupAfterLastStop> dijSearchToPickup;

        TentativeLastStopDistances<DijLabelSet> lastStopDistances;
        Subset vehiclesSeen;

    };

}