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
#include "Algorithms/KaRRi/LastStopSearches/LastStopsAtVertices.h"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <atomic>

namespace karri {

    using namespace tbb;

    template<typename InputGraphT,
            typename CHEnvT,
            typename CostFunctionT,
            typename EllipticBucketsEnvT,
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
                return curFeasible->updateDistanceFromStopToPDLoc(entry.targetId, curFirstIdOfBatch.local(),
                                                                  distsToPDLocs, meetingVertex);
            }


            void setCurFeasible(FeasibleEllipticDistancesT *const newCurFeasible) {
                curFeasible = newCurFeasible;
            }

            void setCurFirstIdOfBatch(int const newCurFirstIdOfBatch) {
                curFirstIdOfBatch.local() = newCurFirstIdOfBatch;
            }

            void curFeasibleSynchronizeDistances() {
                curFeasible->updateToDistancesInGlobalVectors(curFirstIdOfBatch.local());
            }

            void endToSearches() {
                curFeasible->resetLocalPairs();
            }

        private:
            FeasibleEllipticDistancesT *curFeasible;
            enumerable_thread_specific<int> curFirstIdOfBatch;
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
                return curFeasible->updateDistanceFromPDLocToNextStop(prevStopId, curFirstIdOfBatch.local(),
                                                                      distsFromPDLocs, meetingVertex);
            }

            void setCurFeasible(FeasibleEllipticDistancesT *const newCurFeasible) {
                curFeasible = newCurFeasible;
            }

            void setCurFirstIdOfBatch(int const newCurFirstIdOfBatch) {
                curFirstIdOfBatch.local() = newCurFirstIdOfBatch;
            }

            void curFeasibleSynchronizeDistances() {
                curFeasible->updateFromDistancesInGlobalVectors(curFirstIdOfBatch.local());
            }

            void endFromSearches() {
                curFeasible->resetLocalPairs();
            }

        private:
            const RouteState &routeState;
            FeasibleEllipticDistancesT *curFeasible;
            enumerable_thread_specific<int> curFirstIdOfBatch;
        };


        using ScanSourceBuckets = ScanOrdinaryBucket<UpdateDistancesToPDLocs>;
        using ScanTargetBuckets = ScanOrdinaryBucket<UpdateDistancesFromPDLocs>;

        // Info about a PD loc that coincides with an existing stop of a vehicle.
        struct PDLocAtExistingStop {
            int pdId = INVALID_ID;
            int vehId = INVALID_ID;
            int stopIndex = INVALID_INDEX;
        };

        typedef typename CHEnvT::template UpwardSearch<ScanSourceBuckets, StopBCHQuery, LabelSetT> ToQueryType;
        typedef typename CHEnvT::template UpwardSearch<ScanTargetBuckets, StopBCHQuery, LabelSetT> FromQueryType;

    public:

        EllipticBCHSearches(const InputGraphT &inputGraph,
                            const Fleet &fleet,
                            const EllipticBucketsEnvT &ellipticBucketsEnv,
                            const LastStopsAtVertices &lastStopsAtVertices,
                            const CHEnvT &chEnv,
                            const RouteState &routeState,
                            FeasibleEllipticDistancesT &feasibleEllipticPickups,
                            FeasibleEllipticDistancesT &feasibleEllipticDropoffs,
                            RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  sourceBuckets(ellipticBucketsEnv.getSourceBuckets()),
                  lastStopsAtVertices(lastStopsAtVertices),
                  feasibleEllipticPickups(feasibleEllipticPickups),
                  feasibleEllipticDropoffs(feasibleEllipticDropoffs),
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
        void run() {

            // Run for pickups:
            Timer timer;
            updateDistancesToPdLocs.setCurFeasible(&feasibleEllipticPickups);
            updateDistancesFromPdLocs.setCurFeasible(&feasibleEllipticPickups);
            runBCHSearchesFromAndTo(requestState.pickups);
            const int64_t pickupTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ellipticBchStats.pickupTime += pickupTime;
            requestState.stats().ellipticBchStats.pickupNumEdgeRelaxations += totalNumEdgeRelaxations;
            requestState.stats().ellipticBchStats.pickupNumVerticesSettled += totalNumVerticesSettled;
            requestState.stats().ellipticBchStats.pickupNumEntriesScanned += totalNumEntriesScanned;

            // Run for dropoffs:
            timer.restart();
            updateDistancesToPdLocs.setCurFeasible(&feasibleEllipticDropoffs);
            updateDistancesFromPdLocs.setCurFeasible(&feasibleEllipticDropoffs);

            runBCHSearchesFromAndTo(requestState.dropoffs);
            const int64_t dropoffTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ellipticBchStats.dropoffTime += dropoffTime;
            requestState.stats().ellipticBchStats.dropoffNumEdgeRelaxations += totalNumEdgeRelaxations;
            requestState.stats().ellipticBchStats.dropoffNumVerticesSettled += totalNumVerticesSettled;
            requestState.stats().ellipticBchStats.dropoffNumEntriesScanned += totalNumEntriesScanned;
        }

        // Initialize searches for new request
        void init() {

            Timer timer;

            // Find pickups at existing stops for new request and initialize distances.
            const auto pickupsAtExistingStops = findPDLocsAtExistingStops<PICKUP>(requestState.pickups);
            feasibleEllipticPickups.init(requestState.numPickups(), pickupsAtExistingStops, inputGraph);

            // Find dropoffs at existing stops for new request and initialize distances.
            const auto dropoffsAtExistingStops = findPDLocsAtExistingStops<DROPOFF>(requestState.dropoffs);
            feasibleEllipticDropoffs.init(requestState.numDropoffs(), dropoffsAtExistingStops, inputGraph);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ellipticBchStats.initializationTime += time;
        }

    private:

        friend UpdateDistancesFromPDLocs;
        friend UpdateDistancesToPDLocs;

        template<typename SpotContainerT>
        void runBCHSearchesFromAndTo(const SpotContainerT &pdLocs) {

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

            // Interleaved to & from searches
            // für jede PDLoc -> 2 Jobs erstellen

            // Parallel for with lambda function
            parallel_for(int(0), static_cast<int>(pdLocs.size()), K, [=] (int i) 
            {
                runRegularBCHSearchesTo(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs);
            }, static_partitioner());

            // Done with to searches
            updateDistancesToPdLocs.endToSearches();

            // Parallel for with lambda function
            parallel_for(int(0), static_cast<int>(pdLocs.size()), K, [=] (int i) 
            {
                runRegularBCHSearchesFrom(i, std::min(i + K, static_cast<int>(pdLocs.size())), pdLocs);
            }, static_partitioner());
            
            // Done with from searches
            updateDistancesFromPdLocs.endFromSearches();
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
            FromQueryType& localFromQuery = fromQuery.local();
            localFromQuery.runWithOffset(pdLocHeads, {});

            ++numSearchesRun;
            totalNumEdgeRelaxations += localFromQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += localFromQuery.getNumVerticesSettled();

            // After a search batch of K PDLocs, write the distances back to the global vectors
             updateDistancesFromPdLocs.curFeasibleSynchronizeDistances();
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
            ToQueryType& localToQuery = toQuery.local();
            localToQuery.runWithOffset(pdLocTails, travelTimes);

            ++numSearchesRun;
            totalNumEdgeRelaxations += localToQuery.getNumEdgeRelaxations();
            totalNumVerticesSettled += localToQuery.getNumVerticesSettled();
            
            // After a search batch of K PDLocs, write the distances back to the global vectors
             updateDistancesToPdLocs.curFeasibleSynchronizeDistances();
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
                        if (stopLoc == pdLoc.loc) {
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
                        if (routeState.stopLocationsFor(vehId)[numStops - 1] == pdLoc.loc) {
                            res.push_back({pdLoc.id, vehId, numStops - 1});
                        }
                    }
                }
            }

            return res;
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;

        const typename EllipticBucketsEnvT::BucketContainer &sourceBuckets;
        const LastStopsAtVertices &lastStopsAtVertices;

        FeasibleEllipticDistancesT &feasibleEllipticPickups;
        FeasibleEllipticDistancesT &feasibleEllipticDropoffs;

        int distUpperBound;
        UpdateDistancesToPDLocs updateDistancesToPdLocs;
        UpdateDistancesFromPDLocs updateDistancesFromPdLocs;
        
        // ToQueryType toQuery;
        enumerable_thread_specific<ToQueryType> toQuery;
        
        // FromQueryType fromQuery;
        enumerable_thread_specific<FromQueryType> fromQuery;

        int numSearchesRun;
        int numTimesStoppingCriterionMet;
        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
        int totalNumEntriesScannedWithDistSmallerLeeway;
    };
}