/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <vector>
#include <Tools/Logging/NullLogger.h>
#include <DataStructures/Labels/ParentInfo.h>
#include <DataStructures/Labels/BasicLabelSet.h>
#include <DataStructures/Labels/SimdLabelSet.h>

#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include <Algorithms/KaRRi/RouteState.h>
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "Algorithms/PHAST/RPHASTEnvironment.h"
#include "StopWithRank.h"

namespace karri {

    template<typename InputGraphT,
            typename OrdinaryStopsRPHASTSelectionT,
            typename FeasibleEllipticDistancesT,
            typename LabelSetT,
            bool StoreMeetingVertices = false>
    class EllipticRPHASTSearches {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;


        struct PruneIfDistanceExceeded {

            PruneIfDistanceExceeded(const DistanceLabel &maxDist)
                    : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return allSet(distToV > maxDist);
            }

        private:
            const DistanceLabel &maxDist;
        };

//        using PruningCriterion = dij::CompoundCriterion<PruneIfDistanceExceeded,
//                typename OrdinaryStopsRPHASTSelectionT::QueryPruningCriterion>;
        using PruningCriterion = dij::NoCriterion;
        using Query = typename RPHASTEnvironment::Query<LabelSetT, PruningCriterion>;

    public:

        EllipticRPHASTSearches(const InputGraphT &inputGraph,
                               const Fleet &fleet,
                               const CH &ch,
                               const RPHASTEnvironment &rphastEnv,
                               const OrdinaryStopsRPHASTSelectionT &ordinaryStopsRphastSelection,
                               const RouteState &routeState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  ch(ch),
                  routeState(routeState),
                  sourceStopsByRank(ordinaryStopsRphastSelection.getSourceStopsByRank()),
                  sourcesSelection(ordinaryStopsRphastSelection.getSourcesSelection()),
                  targetStopsByRank(ordinaryStopsRphastSelection.getTargetStopsByRank()),
                  targetsSelection(ordinaryStopsRphastSelection.getTargetsSelection()),
                  toQuery(rphastEnv.getReverseRPHASTQuery<LabelSetT, PruningCriterion>()),
//                          dij::CompoundCriterion(PruneIfDistanceExceeded(distUpperBound), ordinaryStopsRphastSelection.getSourcesQueryPrune()))),
                  fromQuery(rphastEnv.getForwardRPHASTQuery<LabelSetT, PruningCriterion>())
//                          dij::CompoundCriterion(PruneIfDistanceExceeded(distUpperBound), ordinaryStopsRphastSelection.getTargetsQueryPrune())))
                           {}


        // Run Elliptic RPHAST searches for pickups and dropoffs
        void run(FeasibleEllipticDistancesT &feasibleEllipticPickups,
                 FeasibleEllipticDistancesT &feasibleEllipticDropoffs,
                 const RequestState &requestState,
                 const PDLocs &pdLocs,
                 stats::EllipticBCHPerformanceStats &stats) {

            // Run for pickups:
            runRPHASTSearchesFromAndTo(requestState, pdLocs.pickups, feasibleEllipticPickups);
            stats.pickupSearchTime += totalSearchTime;
            stats.pickupPostprocessTime += totalPostprocessTime;
            stats.pickupNumEdgeRelaxations += totalNumEdgeRelaxations;
            stats.pickupNumVerticesSettled += totalNumVerticesSettled;
            stats.pickupNumEntriesScanned += totalNumEntriesScanned;
            stats.pickupNumVehiclesSeen += feasibleEllipticPickups.numVehiclesWithRelevantPDLocs();
            stats.pickupNumStopsSeen += feasibleEllipticPickups.numStopsWithRelevantPDLocs();

            // Run for dropoffs:
            runRPHASTSearchesFromAndTo(requestState, pdLocs.dropoffs, feasibleEllipticDropoffs);
            stats.dropoffSearchTime += totalSearchTime;
            stats.dropoffPostprocessTime += totalPostprocessTime;
            stats.dropoffNumEdgeRelaxations += totalNumEdgeRelaxations;
            stats.dropoffNumVerticesSettled += totalNumVerticesSettled;
            stats.dropoffNumEntriesScanned += totalNumEntriesScanned;
            stats.dropoffNumVehiclesSeen += feasibleEllipticDropoffs.numVehiclesWithRelevantPDLocs();
            stats.dropoffNumStopsSeen += feasibleEllipticDropoffs.numStopsWithRelevantPDLocs();
        }

        void init(const RequestState &, const PDLocs &, stats::EllipticBCHPerformanceStats &) {
            // no op
        }

    private:

        template<typename SpotContainerT>
        void runRPHASTSearchesFromAndTo(const RequestState &requestState, SpotContainerT &pdLocs,
                                        FeasibleEllipticDistancesT &feasibleEllipticDistances) {

            numSearchesRun = 0;
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;
            totalSearchTime = 0;
            totalPostprocessTime = 0;

            // Set an upper bound distance for the searches comprised of the maximum leeway or an upper bound based on the
            // current best costs (we compute the maximum detour that would still allow an assignment with costs smaller
            // than the best known and add the maximum leg length since a distance to a PD loc dist cannot lead to a
            // better assignment than the best known if dist - max leg length > max allowed detour).
            const int maxDistBasedOnVehCost = CostCalculator::CostFunction::calcMinDistFromOrToPDLocSuchThatVehCostReachesMinCost(
                    requestState.getBestCost(), routeState.getMaxLegLength());
            distUpperBound = std::min(maxDistBasedOnVehCost, routeState.getMaxLeeway());

            // Process in batches of size K
            for (int i = 0; i < pdLocs.size(); i += K) {
                runRegularRPHASTSearchesTo(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs,
                                           feasibleEllipticDistances);
            }

            for (int i = 0; i < pdLocs.size(); i += K) {
                runRegularRPHASTSearchesFrom(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs,
                                             feasibleEllipticDistances);
            }
        }

        template<typename SpotContainerT>
        void runRegularRPHASTSearchesFrom(const int startId, const int endId,
                                          const SpotContainerT &pdLocs,
                                          FeasibleEllipticDistancesT &feasibleEllipticDistances) {
            KASSERT(endId > startId && endId - startId <= K);

            Timer timer;

            std::array<int, K> pdLocHeads;

            for (unsigned int i = 0; i < K; ++i) {
                int location;
                if (startId + i < endId) {
                    location = pdLocs[startId + i].loc;
                } else {
                    location = pdLocs[startId].loc; // Fill rest of a partial batch with copies of the first PD loc
                }
                pdLocHeads[i] = ch.rank(inputGraph.edgeHead(location));
            }

            fromQuery.run(targetsSelection, pdLocHeads);

            totalSearchTime += timer.elapsed<std::chrono::nanoseconds>();

            ++numSearchesRun;
            totalNumEdgeRelaxations += fromQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += fromQuery.getNumVerticesSettled();

            timer.restart();
            // Store results for each stop where a feasible distance has been found:
            for (const auto &[prevStopId, rank, offset]: targetStopsByRank) {
                const auto rankInSubgraph = targetsSelection.fullToSubMapping[rank];
                const auto leeway = routeState.leewayOfLegStartingAt(prevStopId);
                const auto dist = fromQuery.getDistances(rankInSubgraph) + offset;
                const auto holdsLeeway = dist <= leeway;
                if (anySet(holdsLeeway)) {
                    const DistanceLabel meetingVertices = 0;
                    if constexpr (StoreMeetingVertices) {
                        meetingVertices = fromQuery.getMeetingVertices(rankInSubgraph);
                    }
                    feasibleEllipticDistances.updateDistanceFromPDLocToNextStop(prevStopId, startId, dist,
                                                                                meetingVertices);
                }
            }
            totalPostprocessTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        template<typename SpotContainerT>
        void runRegularRPHASTSearchesTo(const int startId, const int endId,
                                        const SpotContainerT &pdLocs,
                                        FeasibleEllipticDistancesT &feasibleEllipticDistances) {
            KASSERT(endId > startId && endId - startId <= K);

            Timer timer;

            std::array<int, K> travelTimes;
            std::array<int, K> pdLocTails;

            for (int i = 0; i < K; ++i) {
                int location;
                if (startId + i < endId) {
                    location = pdLocs[startId + i].loc;
                } else {
                    location = pdLocs[startId].loc; // Fill rest of a partial batch with copies of the first PD loc
                }
                travelTimes[i] = inputGraph.travelTime(location);
                pdLocTails[i] = ch.rank(inputGraph.edgeTail(location));
            }

            toQuery.run(sourcesSelection, pdLocTails, travelTimes);

            totalSearchTime += timer.elapsed<std::chrono::nanoseconds>();

            ++numSearchesRun;
            totalNumEdgeRelaxations += toQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += toQuery.getNumVerticesSettled();

            timer.restart();
            // Store results for each stop where a feasible distance has been found:
            for (const auto &[stopId, rank]: sourceStopsByRank) {
                const auto rankInSubgraph = sourcesSelection.fullToSubMapping[rank];
                const auto leeway = routeState.leewayOfLegStartingAt(stopId);
                const auto dist = toQuery.getDistances(rankInSubgraph);
                const auto holdsLeeway = dist <= leeway;
                if (anySet(holdsLeeway)) {
                    const DistanceLabel meetingVertices = 0;
                    if constexpr (StoreMeetingVertices) {
                        meetingVertices = fromQuery.getMeetingVertices(rankInSubgraph);
                    }
                    feasibleEllipticDistances.updateDistanceFromStopToPDLoc(stopId, startId, dist, meetingVertices);
                }
            }
            totalPostprocessTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CH &ch;
        const RouteState &routeState;
        const std::vector<StopWithRank> &sourceStopsByRank;
        const RPHASTSelection &sourcesSelection;
        const std::vector<StopWithRankAndOffset> &targetStopsByRank;
        const RPHASTSelection &targetsSelection;

        Query toQuery;
        Query fromQuery;

        DistanceLabel distUpperBound;

        int numSearchesRun;
        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
        int totalNumEntriesScannedWithDistSmallerLeeway;

        int64_t totalSearchTime;
        int64_t totalPostprocessTime;
    };
}