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


#include "MinCostPairAfterLastStopQuery.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "IndividualBCHStrategy.h"


namespace karri::PickupAfterLastStopStrategies {

// A collective reverse BCH search that finds the best vehicle, pickup and dropoff for an assignment at the end of any
// vehicle.
    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename VehicleToPDLocQueryT,
            typename PDDistancesT,
            typename FallbackLabelSetT>
    class CollectiveBCHStrategy {

        using MinCostPairAfterLastStopQueryInst = MinCostPairAfterLastStopQuery<InputGraphT, CHEnvT, LastStopBucketsEnvT, PDDistancesT>;

        using FallbackIndividualBCHStrategy = IndividualBCHStrategy<InputGraphT, CHEnvT, LastStopBucketsEnvT, PDDistancesT, FallbackLabelSetT>;

    public:

        CollectiveBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              VehicleToPDLocQueryT &vehicleToPDLocQuery,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              PDDistancesT &pdDistances,
                              const CostCalculator &calculator,
                              const RouteState &routeState,
                              RequestState &requestState,
                              const InputConfig &inputConfig)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  inputConfig(inputConfig),
                  minCostSearch(inputGraph, fleet, chEnv, routeState, pdDistances, calculator, lastStopBucketsEnv,
                                requestState, inputConfig),
                  vehicleToPDLocQuery(vehicleToPDLocQuery),
                  pdDistances(pdDistances),
                  fallbackStrategy(inputGraph, fleet, chEnv, calculator, lastStopBucketsEnv, pdDistances, routeState,
                                   requestState, minCostSearch.getUpperBoundCostWithHardConstraints(), inputConfig) {}

        void tryPickupAfterLastStop() {

            auto &stats = requestState.stats().palsAssignmentsStats;
            Timer timer;


            // Find out if any pickup after any last stop can plausibly lead to a better assignment than the best known.
            // First lower bound: Only the minimum PD distance
            int costLowerBound = calculator.calcCostLowerBoundForPickupAfterLastStopIndependentOfVehicle(0,
                                                                                                         requestState.minDirectPDDist,
                                                                                                         requestState);
            if (costLowerBound > requestState.getBestCost())
                return;

            const auto colInitTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.collective_initializationTime += colInitTime;

            minCostSearch.run(requestState.getBestCost());

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.searchTime += searchTime;

            stats.collective_numInitialLabelsGenerated += minCostSearch.getNumInitialLabelsGenerated();
            stats.collective_numInitialLabelsNotPruned += minCostSearch.getNumInitialLabelsNotPruned();
            stats.collective_initializationTime += minCostSearch.getInitializationTime(); // Also contained in minCostSearch.getRunTime()
            stats.collective_numDominationRelationTests += minCostSearch.getNumDominationRelationTests();
            stats.numEdgeRelaxationsInSearchGraph += minCostSearch.getNumEdgeRelaxations();
            stats.numVerticesOrLabelsSettled += minCostSearch.getNumLabelsRelaxed();
            stats.numEntriesOrLastStopsScanned += minCostSearch.getNumEntriesScanned();
            stats.numCandidateVehicles += 1;

            timer.restart();


            const int &minCost = minCostSearch.getBestCostWithoutConstraints();
            const auto &asgn = minCostSearch.getBestAssignment();
            if (!asgn.vehicle)
                return;

            const auto totalDetour = asgn.distToPickup + inputConfig.stopTime + asgn.distToDropoff + inputConfig.stopTime;
            using time_utils::isServiceTimeConstraintViolated;
            if (!isServiceTimeConstraintViolated(*asgn.vehicle, requestState, totalDetour, routeState)) {
                // If assignment found by collective search adheres to service time constraint, we have found the
                // best PALS assignment.
                assert(calculator.calc(asgn, requestState) == minCost);
                requestState.tryAssignmentWithKnownCost(asgn, minCost);

                const auto tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
                stats.tryAssignmentsTime += tryAssignmentsTime;
                stats.numAssignmentsTried += 1;
                return;
            }

            const auto tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.collective_usedFallback = true;

            // Otherwise fall back to computing distances explicitly:
            fallbackStrategy.tryPickupAfterLastStop();
        }

    private:

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;
        const InputConfig &inputConfig;

        MinCostPairAfterLastStopQueryInst minCostSearch;
        VehicleToPDLocQueryT &vehicleToPDLocQuery;

        PDDistancesT &pdDistances;


        FallbackIndividualBCHStrategy fallbackStrategy;
    };

}