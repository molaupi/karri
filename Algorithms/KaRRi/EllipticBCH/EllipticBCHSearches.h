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
#include "Algorithms/KaRRi/EllipticBCH/LocalFeasibleDistancesFilter.h"

#include <atomic>
#include <omp.h>
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocAtExistingStop.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBchBatchQueryResult.h"

namespace karri {

    template<typename InputGraphT,
            typename CHEnvT,
            typename CostFunctionT,
            typename EllipticBucketsEnvT,
            typename LastStopsAtVerticesT,
            typename LabelSetT>
    class EllipticBCHSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;


        using Buckets = typename EllipticBucketsEnvT::BucketContainer;
        using ResultForBatch = EllipticBchBatchQueryResult<LabelSetT>;

        struct StopBCHQuery {

            StopBCHQuery(const int &distUpperBound)
                    : distUpperBound(distUpperBound) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                const LabelMask distExceedsUpperBound = distToV > DistanceLabel(distUpperBound);
                return allSet(distExceedsUpperBound);
            }

        private:
            const int &distUpperBound;
        };


        template<typename UpdateDistancesT>
        struct ScanOrdinaryBucket {
            explicit ScanOrdinaryBucket(const Buckets &buckets, UpdateDistancesT updateDistances,
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
            UpdateDistancesT updateDistances;
            int &numEntriesVisited;
            int &numEntriesVisitedWithDistSmallerLeeway;
        };


        struct UpdateDistancesToPDLocs {

            UpdateDistancesToPDLocs(ResultForBatch &distances)
                    : distances(distances) {}

            LabelMask operator()(const int meetingVertex, const BucketEntryWithLeeway &entry,
                                 const DistanceLabel &distsToPDLocs) {

                return distances.updateDistanceFromStopToPDLoc(entry.targetId, distsToPDLocs,
                                                               meetingVertex);
            }


        private:
            ResultForBatch &distances;
        };


        struct UpdateDistancesFromPDLocs {

            UpdateDistancesFromPDLocs(const RouteState &routeState, ResultForBatch &distances)
                    : routeState(routeState), distances(distances) {}

            LabelMask operator()(const int meetingVertex, const BucketEntryWithLeeway &entry,
                                 const DistanceLabel &distsFromPDLocs) {

                const auto &prevStopId = routeState.idOfPreviousStopOf(entry.targetId);

                // If the given stop is the first stop in the vehicle's route, there is no previous stop.
                if (prevStopId == INVALID_ID)
                    return LabelMask(false);

                return distances.updateDistanceFromPDLocToNextStop(prevStopId, distsFromPDLocs,
                                                                   meetingVertex);
            }


        private:
            const RouteState &routeState;
            ResultForBatch &distances;
        };


        using ScanSourceBuckets = ScanOrdinaryBucket<UpdateDistancesToPDLocs>;
        using ScanTargetBuckets = ScanOrdinaryBucket<UpdateDistancesFromPDLocs>;

        using PDLocsAtExistingStops = std::vector<PDLocAtExistingStop>;

        typedef typename CHEnvT::template UpwardSearch<ScanSourceBuckets, StopBCHQuery, LabelSetT> ToQueryType;
        typedef typename CHEnvT::template UpwardSearch<ScanTargetBuckets, StopBCHQuery, LabelSetT> FromQueryType;

    public:

        EllipticBCHSearches(const InputGraphT &inputGraph,
                            const Fleet &fleet,
                            const EllipticBucketsEnvT &ellipticBucketsEnv,
                            const LastStopsAtVerticesT &lastStopsAtVertices,
                            const CHEnvT &chEnv,
                            const RouteState &routeState,

                            RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  chEnv(chEnv),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  sourceBuckets(ellipticBucketsEnv.getSourceBuckets()),
                  lastStopsAtVertices(lastStopsAtVertices),
                  distUpperBound(INFTY),
                  tlQueries(omp_get_max_threads(), ellipticBucketsEnv.getSourceBuckets(), ellipticBucketsEnv.getTargetBuckets(), distUpperBound, chEnv, routeState)
        {}


        // Run Elliptic BCH searches for pickups and dropoffs
        std::pair<std::vector<RelevantPDLoc>, std::vector<RelevantPDLoc>>
        run(const PDLocsAtExistingStops &pickupsAtExistingStops,
            const PDLocsAtExistingStops &dropoffsAtExistingStops) {

            // Run for pickups and dropoffs in parallel:
            Timer timer;

            // Set an upper bound distance for the searches comprised of the maximum leeway or an upper bound based on the
            // current best costs (we compute the maximum detour that would still allow an assignment with costs smaller
            // than the best known and add the maximum leg length since a distance to a PD loc dist cannot lead to a
            // better assignment than the best known if dist - max leg length > max allowed detour).
            const int maxDistBasedOnVehCost = CostFunctionT::calcMinDistFromOrToPDLocSuchThatVehCostReachesMinCost(
                    requestState.getBestCost(), routeState.getMaxLegLength());
            distUpperBound = std::min(maxDistBasedOnVehCost, routeState.getMaxLeeway());

            std::vector<RelevantPDLoc> relevantPickups;
            std::vector<RelevantPDLoc> relevantDropoffs;
            int pickupNumEdgeRelaxations = 0;
            int dropoffNumEdgeRelaxations = 0;
            int pickupNumVerticesVisited = 0;
            int dropoffNumVerticesVisited = 0;
            int pickupNumEntriesScanned = 0;
            int dropoffNumEntriesScanned = 0;
            int pickupNumEntriesScannedWithDistSmallerLeeway = 0;
            int dropoffNumEntriesScannedWithDistSmallerLeeway = 0;
            const int64_t initTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            relevantPickups = runBCHSearchesFromAndTo<PICKUP>(requestState.pickups,
                                                              pickupsAtExistingStops,
                                                              [](const int) { return true; },
                                                              pickupNumEdgeRelaxations,
                                                              pickupNumVerticesVisited,
                                                              pickupNumEntriesScanned,
                                                              pickupNumEntriesScannedWithDistSmallerLeeway);
            const int64_t pickupTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();

            relevantDropoffs = runBCHSearchesFromAndTo<DROPOFF>(requestState.dropoffs,
                                                                dropoffsAtExistingStops,
                                                                [](const int) { return true; },
                                                                dropoffNumEdgeRelaxations,
                                                                dropoffNumVerticesVisited,
                                                                dropoffNumEntriesScanned,
                                                                dropoffNumEntriesScannedWithDistSmallerLeeway);

            const int64_t dropoffTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ellipticBchStats.initializationTime += initTime;
            requestState.stats().ellipticBchStats.pickupTime += pickupTime;
            requestState.stats().ellipticBchStats.dropoffTime += dropoffTime;
            requestState.stats().ellipticBchStats.pickupNumEdgeRelaxations += pickupNumEdgeRelaxations;
            requestState.stats().ellipticBchStats.pickupNumVerticesSettled += pickupNumVerticesVisited;
            requestState.stats().ellipticBchStats.pickupNumEntriesScanned += pickupNumEntriesScanned;
            requestState.stats().ellipticBchStats.dropoffNumEdgeRelaxations += dropoffNumEdgeRelaxations;
            requestState.stats().ellipticBchStats.dropoffNumVerticesSettled += dropoffNumVerticesVisited;
            requestState.stats().ellipticBchStats.dropoffNumEntriesScanned += dropoffNumEntriesScanned;

            return {relevantPickups, relevantDropoffs};
        }

        void init() {
            // no op
        }

    private:

        friend UpdateDistancesFromPDLocs;
        friend UpdateDistancesToPDLocs;


        template<typename SourceBuckets, typename TargetBuckets>
        struct ThreadLocalQueries {

            ThreadLocalQueries(const uint32_t numThreads, const SourceBuckets &sourceBuckets,
                               const TargetBuckets &targetBuckets, const int &distUpperBound, const CHEnvT &chEnv,
                               const RouteState &routeState)
                    : numEntriesScannedPerThread(numThreads),
                      numEntriesScannedWithDistSmallerLeewayPerThread(numThreads),
                      resultForBatchPerThread(numThreads) {
                for (auto i = 0; i < numThreads; ++i) {
                    toQueryPerThread.push_back(
                            chEnv.template getReverseSearch<ScanSourceBuckets, StopBCHQuery, LabelSetT>(
                                    ScanSourceBuckets(sourceBuckets,
                                                      UpdateDistancesToPDLocs(resultForBatchPerThread[i]),
                                                      numEntriesScannedPerThread[i],
                                                      numEntriesScannedWithDistSmallerLeewayPerThread[i]),
                                    StopBCHQuery(distUpperBound)));
                    fromQueryPerThread.push_back(
                            chEnv.template getForwardSearch<ScanTargetBuckets, StopBCHQuery, LabelSetT>(
                                    ScanTargetBuckets(targetBuckets,
                                                      UpdateDistancesFromPDLocs(routeState, resultForBatchPerThread[i]),
                                                      numEntriesScannedPerThread[i],
                                                      numEntriesScannedWithDistSmallerLeewayPerThread[i]),
                                    StopBCHQuery(distUpperBound)));
                }
            }

            int& localNumEntriesScannedPerThread() {
                return numEntriesScannedPerThread[omp_get_thread_num()];
            }

            int& localNumEntriesScannedWithDistSmallerLeewayPerThread() {
                return numEntriesScannedWithDistSmallerLeewayPerThread[omp_get_thread_num()];
            }

            ResultForBatch & localResultForBatch() {
                return resultForBatchPerThread[omp_get_thread_num()];
            }

            ToQueryType &localToQuery() {
                return toQueryPerThread[omp_get_thread_num()];
            }

            FromQueryType &localFromQuery() {
                return fromQueryPerThread[omp_get_thread_num()];
            }

            std::vector<int> numEntriesScannedPerThread;
            std::vector<int> numEntriesScannedWithDistSmallerLeewayPerThread;
            std::vector<ResultForBatch> resultForBatchPerThread;
            std::vector<ToQueryType> toQueryPerThread;
            std::vector<FromQueryType> fromQueryPerThread;
        };


        template<PDLocType type, typename SpotContainerT, typename IsStopEligibleT>
        std::vector<RelevantPDLoc> runBCHSearchesFromAndTo(const SpotContainerT &pdLocs,
                                                           const PDLocsAtExistingStops &pdLocsAtExistingStops,
                                                           const IsStopEligibleT &isStopEligible,
                                                           int &totalNumEdgeRelaxations,
                                                           int &totalNumVerticesVisited,
                                                           int &totalNumEntriesScanned,
                                                           int &totalNumEntriesScannedWithDistSmallerLeeway) {

            std::vector<RelevantPDLoc> globalResult;

#pragma omp parallel
            {

                int threadNum = omp_get_thread_num();
                unused(threadNum);
                int localNumEdgeRelaxations = 0;
                int localNumVerticesVisited = 0;
                int& localNumEntriesScanned = tlQueries.localNumEntriesScannedPerThread();
                int& localNumEntriesScannedWithDistSmallerLeeway = tlQueries.localNumEntriesScannedWithDistSmallerLeewayPerThread();
                ResultForBatch& resultForBatch = tlQueries.localResultForBatch();
                ToQueryType &localToQuery = tlQueries.localToQuery();
                FromQueryType &localFromQuery = tlQueries.localFromQuery();
                std::vector<RelevantPDLoc> localRelevantPdLocs;

                localNumEntriesScanned = 0;
                localNumEntriesScannedWithDistSmallerLeeway = 0;
                resultForBatch.updateNumStops(routeState.getMaxStopId() + 1);

#pragma omp for schedule(dynamic, 1) nowait
//#pragma omp for
                for (auto i = 0; i < pdLocs.size() / K; ++i) {
                    KASSERT(std::all_of(resultForBatch.indexInEntriesVector.begin(),
                                        resultForBatch.indexInEntriesVector.end(),
                                        [&](const auto i) { return i == INVALID_INDEX; }));
                    KASSERT(resultForBatch.entries.empty());

                    const auto fromPdId = i * K;
                    const auto toPdId = std::min((i + 1) * K, static_cast<int>(pdLocs.size()));

                    initializePdLocsAtExistingStopsInLocal(pdLocsAtExistingStops, resultForBatch,
                                                           fromPdId);

                    runRegularBCHSearchesTo(fromPdId, toPdId, pdLocs, localToQuery,
                                            localNumEdgeRelaxations, localNumVerticesVisited);
                    runRegularBCHSearchesFrom(fromPdId, toPdId, pdLocs, localFromQuery,
                                              localNumEdgeRelaxations, localNumVerticesVisited);


                    // Filter entries for current batch for relevancy and convert to RelevantPDLocs
                    DistanceLabel pdLocLocations = {};
                    DistanceLabel pdLocWalkingTimes = {};
                    int j = fromPdId;
                    for (j = fromPdId; j < toPdId; ++j) {
                        pdLocLocations[j - fromPdId] = pdLocs[j].loc;
                        pdLocWalkingTimes[j - fromPdId] = pdLocs[j].walkingDist;
                    }
                    for (; j < (i + 1) * K; ++j) {
                        pdLocLocations[j - fromPdId] = INVALID_EDGE;
                        pdLocWalkingTimes[j - fromPdId] = INFTY;
                    }
                    for (const auto &e: resultForBatch.entries) {
                        const auto &veh = fleet[routeState.vehicleIdOf(e.stopId)];
                        const auto &numStops = routeState.numStopsOf(veh.vehicleId);
                        const auto &occupancies = routeState.occupanciesFor(veh.vehicleId);
                        assert(numStops > 1);

                        const auto &stopIdx = routeState.stopPositionOf(e.stopId);

                        if (!isStopEligible(e.stopId) ||
                            (stopIdx == numStops - 1 && type == PICKUP) ||
                            (occupancies[stopIdx] == veh.capacity && (type == PICKUP || stopIdx == 0))) {
                            continue;
                        }

                        const auto &distTo = e.distFromStopToPdLoc;
                        const auto &distFrom = e.distFromPdLocToNextStop;

                        const LabelMask notRelevant =
                                type == PICKUP ? ~isPickupRelevant<LabelSetT>(veh, stopIdx, pdLocLocations,
                                                                              pdLocWalkingTimes, distTo, distFrom,
                                                                              requestState, routeState) :
                                ~isDropoffRelevant<LabelSetT>(veh, stopIdx, pdLocLocations,
                                                              pdLocWalkingTimes, distTo, distFrom,
                                                              requestState, routeState);
                        if (allSet(notRelevant))
                            continue;
                        for (int idxInBatch = 0; idxInBatch < toPdId - fromPdId; ++idxInBatch) {
                            if (notRelevant[idxInBatch])
                                continue;
                            const int distToPd = distTo[idxInBatch];
                            const int distFromPd = distFrom[idxInBatch];
                            localRelevantPdLocs.push_back(
                                    RelevantPDLoc(e.stopId, fromPdId + idxInBatch, distToPd, distFromPd));
                        }
                    }

                    // Reset resultForBatch
                    for (const auto &e: resultForBatch.entries)
                        resultForBatch.indexInEntriesVector[e.stopId] = INVALID_INDEX;
                    resultForBatch.entries.clear();
                }

#pragma omp critical (combineResults)
                {
                    globalResult.insert(globalResult.end(), localRelevantPdLocs.begin(), localRelevantPdLocs.end());
                    totalNumEdgeRelaxations += localNumEdgeRelaxations;
                    totalNumVerticesVisited += localNumVerticesVisited;
                    totalNumEntriesScanned += localNumEntriesScanned;
                    totalNumEntriesScannedWithDistSmallerLeeway += localNumEntriesScannedWithDistSmallerLeeway;
                }
            }

            return globalResult;
        }

        template<typename SpotContainerT>
        void runRegularBCHSearchesTo(const int startId, const int endId,
                                     const SpotContainerT &pdLocs,
                                     ToQueryType &query, int &numEdgeRelaxations, int &numVerticesVisited) {
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

            query.runWithOffset(pdLocTails, travelTimes);
            numEdgeRelaxations += query.getNumEdgeRelaxations();
            numVerticesVisited += query.getNumVerticesSettled();
        }

        template<typename SpotContainerT>
        void runRegularBCHSearchesFrom(const int startId, const int endId,
                                       const SpotContainerT &pdLocs,
                                       FromQueryType &query, int &numEdgeRelaxations, int &numVerticesVisited) {
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

            query.runWithOffset(pdLocHeads, {});
            numEdgeRelaxations += query.getNumEdgeRelaxations();
            numVerticesVisited += query.getNumVerticesSettled();
        }

        template<PDLocType type, typename PDLocsT>
        std::vector<PDLocAtExistingStop>
        findPDLocsAtExistingStops(const PDLocsT &pdLocs) {
            std::vector<PDLocAtExistingStop> res;

            for (const auto &pdLoc: pdLocs) {
                const auto head = inputGraph.edgeHead(pdLoc.loc);
                const auto headRank = ch.rank(head);
                for (const auto &e: sourceBuckets.getBucketOf(headRank)) {
                    if (e.distToTarget == 0) {
                        const int vehId = routeState.vehicleIdOf(e.targetId);
                        const int stopIdx = routeState.stopPositionOf(e.targetId);
                        const auto &stopLoc = routeState.stopLocationsFor(vehId)[stopIdx];
                        if (routeState.numStopsOf(vehId) > 1 && stopLoc == pdLoc.loc) {
                            res.push_back({pdLoc.id, vehId, stopIdx});
                        }
                    }
                }

                if constexpr (type == DROPOFF) {
                    // Additionally find dropoffs that coincide with the last stop:
                    if (!lastStopsAtVertices.isAnyLastStopAtVertex(head))
                        continue;

                    for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(head)) {
                        const auto numStops = routeState.numStopsOf(vehId);
                        if (numStops > 1 && routeState.stopLocationsFor(vehId)[numStops - 1] == pdLoc.loc) {
                            res.push_back({pdLoc.id, vehId, numStops - 1});
                        }
                    }
                }
            }

            return res;
        }

        template<typename PDLocsAtExistingStops>
        void initializePdLocsAtExistingStopsInLocal(const PDLocsAtExistingStops &pdLocsAtExistingStops,
                                                    ResultForBatch &localDistances,
                                                    const int firstPdLocIdInBatch) {
            for (const PDLocAtExistingStop &pdLocAtExistingStop: pdLocsAtExistingStops) {
                const int &pdId = pdLocAtExistingStop.pdId;
                if (pdId < firstPdLocIdInBatch || pdId >= firstPdLocIdInBatch + K)
                    continue;

                const auto &vehId = pdLocAtExistingStop.vehId;
                assert(pdLocAtExistingStop.stopIndex < routeState.numStopsOf(vehId));
                const auto &stopId = routeState.stopIdsFor(vehId)[pdLocAtExistingStop.stopIndex];
                const auto &stopVertex = inputGraph.edgeHead(
                        routeState.stopLocationsFor(vehId)[pdLocAtExistingStop.stopIndex]);

                DistanceLabel label = INFTY;

                label[pdId % K] = 0;
                localDistances.updateDistanceFromStopToPDLoc(stopId, label, stopVertex);

                const auto lengthOfLegStartingHere = time_utils::calcLengthOfLegStartingAt(
                        pdLocAtExistingStop.stopIndex, vehId, routeState);
                label[pdId % K] = lengthOfLegStartingHere;
                localDistances.updateDistanceFromPDLocToNextStop(stopId, label, stopVertex);
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CHEnvT &chEnv;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;

        const typename EllipticBucketsEnvT::BucketContainer &sourceBuckets;
        const LastStopsAtVerticesT &lastStopsAtVertices;

        int distUpperBound;

        ThreadLocalQueries<Buckets, Buckets> tlQueries;

    };
}