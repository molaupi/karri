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

#include <vector>
#include <Tools/Logging/NullLogger.h>
#include <DataStructures/Labels/ParentInfo.h>
#include <DataStructures/Labels/BasicLabelSet.h>
#include <DataStructures/Labels/SimdLabelSet.h>

#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include <Algorithms/KaRRi/RouteState.h>
#include "BucketEntryWithLeeway.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"

namespace karri {

    template<typename InputGraphT,
            typename CHEnvT,
            typename CostFunctionT,
            typename EllipticBucketsEnvT,
            typename LastStopsAtVerticesT,
            typename FeasibleEllipticDistancesT,
            typename LabelSetT>
    class EllipticBCHSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;


        using Buckets = typename EllipticBucketsEnvT::BucketContainer;

        struct StopBCHQuery {

            StopBCHQuery(const int &distUpperBound, int &numTimesStoppingCriterionMet)
                    : distUpperBound(distUpperBound),
                      numTimesStoppingCriterionMet(numTimesStoppingCriterionMet) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                const LabelMask distExceedsUpperBound = distToV > DistanceLabel(distUpperBound);
                const bool stop = allSet(distExceedsUpperBound);
                numTimesStoppingCriterionMet += stop;
                return stop;
            }

        private:
            const int &distUpperBound;
            int &numTimesStoppingCriterionMet;
        };


        template<typename UpdateDistancesT>
        struct ScanOrdinaryBucket {
            explicit ScanOrdinaryBucket(const Buckets &buckets, UpdateDistancesT &updateDistances,
                                        int &numEntriesVisited, int &numEntriesVisitedWithDistSmallerLeeway)
                    : buckets(buckets),
                      updateDistances(updateDistances),
                      numEntriesVisited(numEntriesVisited),
                      numEntriesVisitedWithDistSmallerLeeway(numEntriesVisitedWithDistSmallerLeeway) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {

                for (const auto &entry: buckets.getBucketOf(v)) {
                    ++numEntriesVisited;
                    const auto distViaV = distToV + entry.distToTarget;

                    if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                        // Entries in a bucket are ordered by the remaining leeway, i.e. the leeway minus the distance from/to
                        // the stop to/from this vertex.
                        // If all distances break the remaining leeway for this entry, then they also break the remaining leeway
                        // of the rest of the entries in this bucket, and we can stop scanning the bucket.
                        if (allSet(entry.leeway < distViaV))
                            break;
                    }

                    // Otherwise, check if the tentative distances needs to be updated.
                    ++numEntriesVisitedWithDistSmallerLeeway;
                    updateDistances(v, entry, distViaV);
                }

                return false;
            }

        private:
            const Buckets &buckets;
            UpdateDistancesT &updateDistances;
            int &numEntriesVisited;
            int &numEntriesVisitedWithDistSmallerLeeway;
        };


        struct UpdateDistancesToPDLocs {

            UpdateDistancesToPDLocs() : curFeasible(nullptr), curFirstIdOfBatch(INVALID_ID) {}

            LabelMask operator()(const int meetingVertex, const BucketEntryWithLeeway &entry,
                                 const DistanceLabel &distsToPDLocs) {

                assert(curFeasible);
                return curFeasible->updateDistanceFromStopToPDLoc(entry.targetId, curFirstIdOfBatch,
                                                                  distsToPDLocs, meetingVertex);
            }


            void setCurFeasible(FeasibleEllipticDistancesT *const newCurFeasible) {
                curFeasible = newCurFeasible;
            }

            void setCurFirstIdOfBatch(int const newCurFirstIdOfBatch) {
                curFirstIdOfBatch = newCurFirstIdOfBatch;
            }

        private:
            FeasibleEllipticDistancesT *curFeasible;
            int curFirstIdOfBatch;
        };

        struct UpdateDistancesFromPDLocs {

            UpdateDistancesFromPDLocs(const RouteState &routeState)
                    : routeState(routeState), curFeasible(nullptr), curFirstIdOfBatch(INVALID_ID) {}

            LabelMask operator()(const int meetingVertex, const BucketEntryWithLeeway &entry,
                                 const DistanceLabel &distsFromPDLocs) {

                const auto &prevStopId = routeState.idOfPreviousStopOf(entry.targetId);

                // If the given stop is the first stop in the vehicle's route, there is no previous stop.
                if (prevStopId == INVALID_ID)
                    return LabelMask(false);

                assert(curFeasible);
                return curFeasible->updateDistanceFromPDLocToNextStop(prevStopId, curFirstIdOfBatch,
                                                                      distsFromPDLocs, meetingVertex);
            }

            void setCurFeasible(FeasibleEllipticDistancesT *const newCurFeasible) {
                curFeasible = newCurFeasible;
            }

            void setCurFirstIdOfBatch(int const newCurFirstIdOfBatch) {
                curFirstIdOfBatch = newCurFirstIdOfBatch;
            }

        private:
            const RouteState &routeState;
            FeasibleEllipticDistancesT *curFeasible;
            int curFirstIdOfBatch;
        };


        using ScanSourceBuckets = ScanOrdinaryBucket<UpdateDistancesToPDLocs>;
        using ScanTargetBuckets = ScanOrdinaryBucket<UpdateDistancesFromPDLocs>;



    public:

        EllipticBCHSearches(const InputGraphT &inputGraph,
                            const Fleet &fleet,
                            const EllipticBucketsEnvT &ellipticBucketsEnv,
                            const LastStopsAtVerticesT &lastStopsAtVertices,
                            const CHEnvT &chEnv,
                            const RouteState &routeState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  sourceBuckets(ellipticBucketsEnv.getSourceBuckets()),
                  lastStopsAtVertices(lastStopsAtVertices),
                  distUpperBound(INFTY),
                  updateDistancesToPdLocs(),
                  updateDistancesFromPdLocs(routeState),
                  toQuery(chEnv.template getReverseSearch<ScanSourceBuckets, StopBCHQuery, LabelSetT>(
                          ScanSourceBuckets(ellipticBucketsEnv.getSourceBuckets(), updateDistancesToPdLocs,
                                            totalNumEntriesScanned, totalNumEntriesScannedWithDistSmallerLeeway),
                          StopBCHQuery(distUpperBound, numTimesStoppingCriterionMet))),
                  fromQuery(chEnv.template getForwardSearch<ScanTargetBuckets, StopBCHQuery, LabelSetT>(
                          ScanTargetBuckets(ellipticBucketsEnv.getTargetBuckets(), updateDistancesFromPdLocs,
                                            totalNumEntriesScanned, totalNumEntriesScannedWithDistSmallerLeeway),
                          StopBCHQuery(distUpperBound, numTimesStoppingCriterionMet))) {}


        // Run Elliptic BCH searches for pickups and dropoffs
        void run(FeasibleEllipticDistancesT &feasibleEllipticPickups,
                 FeasibleEllipticDistancesT &feasibleEllipticDropoffs,
                 const RequestState& requestState,
                 const PDLocs& pdLocs,
                 stats::EllipticBCHPerformanceStats& stats) {

            // Run for pickups:
            Timer timer;
            updateDistancesToPdLocs.setCurFeasible(&feasibleEllipticPickups);
            updateDistancesFromPdLocs.setCurFeasible(&feasibleEllipticPickups);
            runBCHSearchesFromAndTo(requestState, pdLocs.pickups);
            const int64_t pickupTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.pickupTime += pickupTime;
            stats.pickupNumEdgeRelaxations += totalNumEdgeRelaxations;
            stats.pickupNumVerticesSettled += totalNumVerticesSettled;
            stats.pickupNumEntriesScanned += totalNumEntriesScanned;

            // Run for dropoffs:
            timer.restart();
            updateDistancesToPdLocs.setCurFeasible(&feasibleEllipticDropoffs);
            updateDistancesFromPdLocs.setCurFeasible(&feasibleEllipticDropoffs);

            runBCHSearchesFromAndTo(requestState, pdLocs.dropoffs);
            const int64_t dropoffTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.dropoffTime += dropoffTime;
            stats.dropoffNumEdgeRelaxations += totalNumEdgeRelaxations;
            stats.dropoffNumVerticesSettled += totalNumVerticesSettled;
            stats.dropoffNumEntriesScanned += totalNumEntriesScanned;
        }

        void init(const RequestState&, const PDLocs&, stats::EllipticBCHPerformanceStats&) {
            // no op
        }

    private:

        friend UpdateDistancesFromPDLocs;
        friend UpdateDistancesToPDLocs;

        template<typename SpotContainerT>
        void runBCHSearchesFromAndTo(const RequestState& requestState, SpotContainerT &pdLocs) {

            numSearchesRun = 0;
            numTimesStoppingCriterionMet = 0;
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            // Set an upper bound distance for the searches comprised of the maximum leeway or an upper bound based on the
            // current best costs (we compute the maximum detour that would still allow an assignment with costs smaller
            // than the best known and add the maximum leg length since a distance to a PD loc dist cannot lead to a
            // better assignment than the best known if dist - max leg length > max allowed detour).
            const int maxDistBasedOnVehCost = CostFunctionT::calcMinDistFromOrToPDLocSuchThatVehCostReachesMinCost(
                    requestState.getBestCost(), routeState.getMaxLegLength());
            distUpperBound = std::min(maxDistBasedOnVehCost, routeState.getMaxLeeway());

            // Process in batches of size K
            for (int i = 0; i < pdLocs.size(); i += K) {
                runRegularBCHSearchesTo(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs);
            }

            for (int i = 0; i < pdLocs.size(); i += K) {
                runRegularBCHSearchesFrom(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs);
            }
        }

        template<typename SpotContainerT>
        void runRegularBCHSearchesFrom(const int startId, const int endId,
                                       const SpotContainerT &pdLocs) {
            assert(endId > startId && endId - startId <= K);

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

            updateDistancesFromPdLocs.setCurFirstIdOfBatch(startId);
            fromQuery.runWithOffset(pdLocHeads, {});

            ++numSearchesRun;
            totalNumEdgeRelaxations += fromQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += fromQuery.getNumVerticesSettled();
        }

        template<typename SpotContainerT>
        void runRegularBCHSearchesTo(const int startId, const int endId,
                                     const SpotContainerT &pdLocs) {
            assert(endId > startId && endId - startId <= K);

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

            updateDistancesToPdLocs.setCurFirstIdOfBatch(startId);
            toQuery.runWithOffset(pdLocTails, travelTimes);

            ++numSearchesRun;
            totalNumEdgeRelaxations += toQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += toQuery.getNumVerticesSettled();
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CH &ch;
        const RouteState &routeState;

        const typename EllipticBucketsEnvT::BucketContainer &sourceBuckets;
        const LastStopsAtVerticesT &lastStopsAtVertices;

        int distUpperBound;
        UpdateDistancesToPDLocs updateDistancesToPdLocs;
        UpdateDistancesFromPDLocs updateDistancesFromPdLocs;
        typename CHEnvT::template UpwardSearch<ScanSourceBuckets, StopBCHQuery, LabelSetT> toQuery;
        typename CHEnvT::template UpwardSearch<ScanTargetBuckets, StopBCHQuery, LabelSetT> fromQuery;

        int numSearchesRun;
        int numTimesStoppingCriterionMet;
        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
        int totalNumEntriesScannedWithDistSmallerLeeway;
    };
}