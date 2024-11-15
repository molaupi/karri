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
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"

namespace karri {

// Finds pickup-after-last-stop (PALS) insertions using the encapsulated strategy.
    template<typename InputGraphT, typename PDDistancesT, typename StrategyT, typename LastStopsAtVerticesT>
    class PALSAssignmentsFinder {

    public:

        PALSAssignmentsFinder(StrategyT &strategy, const InputGraphT &inputGraph, const Fleet &fleet,
                              const CostCalculator &calculator, const LastStopsAtVerticesT &lastStopsAtVertices,
                              const RouteState &routeState, RequestState &requestState)
                : strategy(strategy),
                  inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  lastStopsAtVertices(lastStopsAtVertices),
                  routeState(routeState),
                  requestState(requestState) {}

        void findAssignments(const PDDistancesT& pdDistances) {
            findAssignmentsWherePickupCoincidesWithLastStop(pdDistances);
            strategy.tryPickupAfterLastStop(pdDistances);
        }

        void init() {
            // no op
        }

    private:

        // Simple case for pickups that coincide with last stops of vehicles is the same regardless of strategy, so it
        // is treated here.
        void findAssignmentsWherePickupCoincidesWithLastStop(const PDDistancesT& pdDistances) {
            int numInsertionsForCoinciding = 0;
            int numCandidateVehiclesForCoinciding = 0;
            Timer timer;

            Assignment asgn;
            asgn.legs.emplace_back();
            asgn.legs.back().travelTimeToPickup = 0;
            asgn.legs.back().detourCostToPickup = 0;
            for (const auto &p: requestState.pickups) {
                asgn.pickup = &p;

                const int head = inputGraph.edgeHead(asgn.pickup->loc);
                for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(head)) {
                    ++numCandidateVehiclesForCoinciding;
                    const auto numStops = routeState.numStopsOf(vehId);
                    if (routeState.stopLocationsFor(vehId)[numStops - 1] != asgn.pickup->loc)
                        continue;

                    // Calculate lower bound on insertion cost with this pickup and vehicle
                    const auto lowerBoundCost = calculator.calcCostLowerBoundForPickupAfterLastStop(
                            fleet[vehId], *asgn.pickup, 0, requestState.minDirectPDDist, requestState);
                    if (lowerBoundCost > requestState.getBestCost())
                        continue;

                    // If necessary, check paired insertion with each dropoff
                    asgn.legs.back().vehicle = &fleet[vehId];
                    asgn.legs.back().pickupStopIdx = numStops - 1;
                    asgn.legs.back().dropoffStopIdx = numStops - 1;

                    for (const auto &d: requestState.dropoffs) {
                        asgn.dropoff = &d;
                        asgn.legs.back().travelTimeToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        asgn.legs.back().detourCostToDropoff = pdDistances.getDirectDistance(*asgn.pickup, *asgn.dropoff);
                        ++numInsertionsForCoinciding;
                        requestState.tryAssignment(asgn);
                    }
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().palsAssignmentsStats.pickupAtLastStop_tryAssignmentsTime += time;
            requestState.stats().palsAssignmentsStats.pickupAtLastStop_numCandidateVehicles += numCandidateVehiclesForCoinciding;
            requestState.stats().palsAssignmentsStats.pickupAtLastStop_numAssignmentsTried += numInsertionsForCoinciding;
        }

        StrategyT &strategy;

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const LastStopsAtVerticesT &lastStopsAtVertices;
        const RouteState &routeState;
        RequestState &requestState;

    };
}