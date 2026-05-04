/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Algorithms/KaRRi/LastStopSearches/TentativeLastStopDistances.h"

#include "Algorithms/KaRRi/CostCalculator.h"
#include "RelevantDropoffsAfterLastStop.h"

namespace karri::Transfers {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename LabelSet>
    class TransfersDropoffALSBCHStrategy {
    private:


        static constexpr int K = LabelSet::K;
        using LabelMask = typename LabelSet::LabelMask;
        using DistanceLabel = typename LabelSet::DistanceLabel;


        struct DropoffAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = false;

            DropoffAfterLastStopPruner(TransfersDropoffALSBCHStrategy &strat,
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

                const DistanceLabel walkingDists = considerWalkingDists ? strat.currentDropoffWalkingDists : 0;
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
                        strat.currentDropoffWalkingDists, minDistancesToDropoffs, arrTimesAtDropoffs, strat.requestState);

                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                            const DistanceLabel& distancesToDropoffs) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToDropoffs < INFTY);
                }

                const DistanceLabel costLowerBound = calc.template calcKVehicleDependentCostLowerBoundsForDALSWithKnownDistToDropoff<LabelSet>(
                        vehId, strat.currentDropoffWalkingDists, distancesToDropoffs, 0, strat.requestState);
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int, const DistanceLabel &) {
                // issue: for an upper bound on the cost, we need an upper bound on what the cost for a pickup using this
                // vehicle is (i.e. an upper bound on the detour, trip time and added trip time for existing passengers
                // until the last stop). However, for pickups before the next stop, we can't derive an upper bound on the
                // detour since we only know lower bounds from the elliptic BCH queries
            }

            bool isVehicleEligible(const int vehId) const {
                return strat.routeState.numStopsOf(vehId) > 1;
            }

        private:
            TransfersDropoffALSBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using DropoffBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, DropoffAfterLastStopPruner, LabelSet>;

    public: 

        TransfersDropoffALSBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              CostCalculator &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const RouteState &routeState,
                              RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  routeState(routeState),
                  requestState(requestState),
                  checkPBNSForVehicle(fleet.size()),
                  vehiclesSeenForDropoffs(fleet.size()),
                  search(lastStopBucketsEnv, lastStopDistances, chEnv, routeState,
                         vehiclesSeenForDropoffs,
                         DropoffAfterLastStopPruner(*this, calculator)),
                  lastStopDistances(fleet.size()) {}

        RelevantDropoffsAfterLastStop findDropoffsAfterLastStop(const PDLocs &pdLocs) {
            runBchQueries(pdLocs);
            return constructResult(pdLocs);
        }

    private:

        // Run BCH queries that obtain distances from last stops to dropoffs
        void runBchQueries(const PDLocs &pdLocs) {
            // Timer timer;

            initDropoffSearches(pdLocs);
            for (unsigned int i = 0; i < pdLocs.numDropoffs(); i += K)
                runSearchesForDropoffBatch(i, pdLocs);

            // const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            // requestState.stats().dalsAssignmentsStats.searchTime += searchTime;
            // requestState.stats().dalsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            // requestState.stats().dalsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            // requestState.stats().dalsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            // requestState.stats().dalsAssignmentsStats.numCandidateVehicles += vehiclesSeenForDropoffs.size();
        }

        void initDropoffSearches(const PDLocs &pdLocs) {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            upperBoundCost = requestState.getBestCost();
            vehiclesSeenForDropoffs.clear();

            // Construct more space for dropoff labels if needed.
            const int numDropoffBatches =
                    pdLocs.numDropoffs() / K + (pdLocs.numDropoffs() % K != 0);
            lastStopDistances.init(numDropoffBatches);

        }

        void runSearchesForDropoffBatch(const unsigned int firstDropoffId, const PDLocs &pdLocs) {
            KASSERT(firstDropoffId % K == 0 && firstDropoffId < pdLocs.numDropoffs());
            const int batchIdx = firstDropoffId / K;

            std::array<int, K> dropoffTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &dropoff =
                        firstDropoffId + i < pdLocs.numDropoffs() ? pdLocs.dropoffs[firstDropoffId + i]
                                                                        : pdLocs.dropoffs[firstDropoffId];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                travelTimes[i] = inputGraph.travelTime(dropoff.loc);
                currentDropoffWalkingDists[i] = dropoff.walkingDist;
            }

            lastStopDistances.setCurBatchIdx(batchIdx);
            search.run(dropoffTails, travelTimes);

            totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            totalNumVerticesSettled += search.getNumVerticesSettled();
            totalNumEntriesScanned += search.getNumEntriesScanned();
        }

        RelevantDropoffsAfterLastStop constructResult(const PDLocs &pdLocs) {
            RelevantDropoffsAfterLastStop result(fleet.size());
            for (const auto& vehId : vehiclesSeenForDropoffs) {
                const auto numBefore = result.relevantSpots.size();
                for (unsigned int dropoffId = 0; dropoffId < pdLocs.numDropoffs(); ++dropoffId) {
                    const auto dist = lastStopDistances.getDistance(vehId, dropoffId);
                    // TODO: cost based filtering here
                    if (dist >= INFTY)
                        continue;
                    result.relevantSpots.push_back(RelevantDropoffsAfterLastStop::RelevantDropoff(dropoffId, dist));
                }
                if (result.relevantSpots.size() == numBefore)
                    continue;  // No relevant dropoffs for this vehicle
                result.vehicleToPdLocs[vehId].start = numBefore;
                result.vehicleToPdLocs[vehId].end = result.relevantSpots.size();
                result.vehiclesWithRelevantSpots.push_back(vehId);
            }
            return result;
        }


        const InputGraphT &inputGraph;
        const Fleet &fleet;
        CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        int upperBoundCost;

        // Vehicles seen by any last stop search
        LightweightSubset vehiclesSeenForDropoffs;
        DropoffBCHQuery search;
        DistanceLabel currentDropoffWalkingDists;
        TentativeLastStopDistances<LabelSet> lastStopDistances;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

    };

}