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
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/SharedSearchSpaceBucketContainer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "PDDistances.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/parallel_for.h"

namespace karri::PDDistanceQueryStrategies {

    template<typename InputGraphT,
            typename CHEnvT,
            typename VehicleToPDLocQueryT,
            typename LabelSetT>
    class BCHStrategy {

    public:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

    private:

        struct DropoffBatchLabel {

            static constexpr unsigned int invalid_target = std::numeric_limits<unsigned int>::max();

            unsigned int targetId = invalid_target; // the batch id for this batch of K dropoffs with sequential ids (i.e. targetId ranges from 0 to numDropoffs / K ( + 1)).
            DistanceLabel distToDropoff = INFTY; // the distances from this vertex to the K dropoffs

            friend bool operator==(const DropoffBatchLabel &lhs, const DropoffBatchLabel &rhs) noexcept {
                return lhs.targetId == rhs.targetId;
            }

            void cmpAndUpdate(const DropoffBatchLabel &other) {
                assert(targetId == invalid_target || targetId == other.targetId);
                distToDropoff.min(other.distToDropoff);
                if (targetId == invalid_target)
                    targetId = other.targetId;
            }
        };

        using BucketContainer = SharedSearchSpaceBucketContainer<DropoffBatchLabel>;
        using ThreadLocalBuckets = typename BucketContainer::ThreadLocalBuckets;
        using ThreadLocalPDDistances = typename PDDistances<LabelSetT>::ThreadLocalPDDistances;


        struct StopWhenMaxDistExceeded {

            explicit StopWhenMaxDistExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                const auto exceedsMaxDist = maxDist < distToV;
                const auto doesNotExceedMaxDist = ~exceedsMaxDist;
                const bool anyDoesNoExceedMaxDist = anySet(doesNotExceedMaxDist);
                return !anyDoesNoExceedMaxDist;
//                return !anySet(~(exceedsMaxDist));
            }


        private:
            const int &maxDist;
        };

        struct UpdateBucketEntries {

            UpdateBucketEntries() : curBuckets(nullptr), dropoffBatchId(INVALID_ID) {}

            bool operator()(const int v, const DistanceLabel &distToV) {

                assert(curBuckets);
                return curBuckets->insertOrUpdate(v, {dropoffBatchId, distToV});
            }

            void setCurLocalBuckets(ThreadLocalBuckets *const newCurBuckets) {
                curBuckets = newCurBuckets;
            }

            void setCurDropoffBatchId(int const newCurDropoffBatchId) {
                dropoffBatchId = newCurDropoffBatchId;
            }

        private:
            ThreadLocalBuckets *curBuckets;
            unsigned int dropoffBatchId;
        };

        struct WriteBucketEntry {
            explicit WriteBucketEntry(UpdateBucketEntries &updateBuckets) 
                : updateBuckets(updateBuckets) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                updateBuckets(v, distToV);
                return false;
            }

        private:

            UpdateBucketEntries &updateBuckets;
        };


        struct UpdatePDDistances {

            UpdatePDDistances() : curDistances(nullptr), curFirstPickupId(INVALID_ID) {}

            void operator()(const unsigned int dropoffId, const DistanceLabel &dist) {

                assert(curDistances);
                return curDistances->updateDistanceBatchIfSmaller(curFirstPickupId, dropoffId, dist);
            }

            void setCurLocalDistances(ThreadLocalPDDistances *const newCurDistances) {
                curDistances = newCurDistances;
            }

            void setCurFirstPickupId(int const newCurFirstPickupId) {
                curFirstPickupId = newCurFirstPickupId;
            }

        private:
            ThreadLocalPDDistances *curDistances;
            int curFirstPickupId;
        };

        struct ScanDropoffBucketAndUpdatePDDistances {
            explicit ScanDropoffBucketAndUpdatePDDistances(RequestState &requestState, BucketContainer &dropoffBuckets, UpdatePDDistances &updatePDDistances) 
                    : requestState(requestState),
                      dropoffBuckets(dropoffBuckets),
                      updatePDDistances(updatePDDistances) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                for (const auto &dropoffBatchLabel: dropoffBuckets.getBucketOf(v)) {
                    // Update distances to each dropoff in the batch label:
                    const auto firstDropoffIdInBatch = dropoffBatchLabel.targetId * K;
                    if (firstDropoffIdInBatch == DropoffBatchLabel::invalid_target)
                        continue;
                    for (int i = 0; i < K && firstDropoffIdInBatch + i < requestState.numDropoffs(); ++i) {
                        updatePDDistances(firstDropoffIdInBatch + i,
                                                   distToV + dropoffBatchLabel.distToDropoff[i]);
                    }
                }
                return false;
            }

        private:

            RequestState &requestState;
            BucketContainer &dropoffBuckets;
            UpdatePDDistances &updatePDDistances;
        };

        using FillBucketsSearch = typename CHEnvT::template UpwardSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>;
        using FindPDDistancesSearch = typename CHEnvT::template UpwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>;

        friend UpdatePDDistances;
        friend WriteBucketEntry;
        friend ScanDropoffBucketAndUpdatePDDistances;
        friend StopWhenMaxDistExceeded;

    public:

        BCHStrategy(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                    PDDistances<LabelSetT> &distances,
                    RequestState &requestState,
                    VehicleToPDLocQueryT &vehicleToPDLocQuery)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  vehicleToPDLocQuery(vehicleToPDLocQuery),
                  distances(distances),
                  updateBucketEntries(),
                  updatePDDistances(),
                  dropoffBuckets(inputGraph.numVertices()),
                  fillBucketsSearch([&]() { return chEnv.template getReverseSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>(
                                  WriteBucketEntry(updateBucketEntries.local()), 
                                  StopWhenMaxDistExceeded(upperBoundDirectPDDist)); }),
                  findPDDistancesSearch([&]() { return chEnv.template getForwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>(
                                  ScanDropoffBucketAndUpdatePDDistances(requestState, dropoffBuckets, updatePDDistances.local()),
                                  StopWhenMaxDistExceeded(upperBoundDirectPDDist)); }) {}


        // Computes all distances from every pickup to every dropoff and stores them in the given DirectPDDistances.
        void run() {
            assert(requestState.pickups[0].loc == requestState.originalRequest.origin
                   && requestState.dropoffs[0].loc == requestState.originalRequest.destination);
            Timer timer;

            if (requestState.numPickups() == 1 && requestState.numDropoffs() == 1) {
                requestState.minDirectPDDist = requestState.originalReqDirectDist;
                return;
            }

            const auto numDropoffSearches = requestState.numDropoffs() / K + (requestState.numDropoffs() % K != 0);
            dropoffBuckets.init(numDropoffSearches);


            // Compute upper bound on every PD distance by adding the longest vehicle distance from any pickup to the
            // origin, the distance from the origin to the destination, and the longest distance from the destination
            // to any dropoff.
            vehicleToPDLocQuery.runReverse(requestState.pickups);
            vehicleToPDLocQuery.runForward(requestState.dropoffs);

            int maxPickupToOriginVehDist = 0;
            for (const auto &pickup: requestState.pickups) {
                maxPickupToOriginVehDist = std::max(maxPickupToOriginVehDist, pickup.vehDistToCenter);
            }

            int maxDestToDropoffVehDist = 0;
            for (const auto &dropoff: requestState.dropoffs) {
                maxDestToDropoffVehDist = std::max(maxDestToDropoffVehDist, dropoff.vehDistFromCenter);
            }

            if (maxPickupToOriginVehDist >= INFTY || maxDestToDropoffVehDist >= INFTY) {
                upperBoundDirectPDDist = INFTY;
            } else {
                upperBoundDirectPDDist = maxPickupToOriginVehDist + requestState.originalReqDirectDist +
                                         maxDestToDropoffVehDist; // read by stopping criterion of searches
            }


            // Fill dropoff buckets:
            tbb::parallel_for(int(0), static_cast<int>(requestState.numDropoffs()), K, [&] (int i)
            {
                fillDropoffBuckets(i, std::min(i + K, static_cast<int>(requestState.numDropoffs())));
            });
            // for (int i = 0; i < requestState.numDropoffs(); i += K) {
            //     fillDropoffBuckets(i, std::min(i + K, static_cast<int>(requestState.numDropoffs())));
            // }

            const int64_t dropoffBucketEntryGenTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.dropoffBucketEntryGenTime = dropoffBucketEntryGenTime;
            timer.restart();

            // Run pickup searches against dropoff buckets:
            tbb::parallel_for(int(0), static_cast<int>(requestState.numPickups()), K, [&] (int i)
            {
                runPickupSearches(i, std::min(i + K, static_cast<int>(requestState.numPickups())));
            });
            // for (int i = 0; i < requestState.numPickups(); i += K) {
            //     runPickupSearches(i, std::min(i + K, static_cast<int>(requestState.numPickups())));
            // }

            requestState.minDirectPDDist = distances.getMinDirectDistance().load(std::memory_order_relaxed);

            const int64_t pickupSearchesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.pickupBchSearchTime += pickupSearchesTime;
        }

        void init() {
            Timer timer;
            distances.clear();
            distances.updateDistanceIfSmaller(0, 0, requestState.originalReqDirectDist);
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += time;
        }

    private:

        // void
        // updatePDDistances(const unsigned int firstPickupId, const unsigned int dropoffId, const DistanceLabel &dist) {
        //     distances.updateDistanceBatchIfSmaller(firstPickupId, dropoffId, dist);
        // }

        void fillDropoffBuckets(const int startId, const int endId) {
            assert(endId > startId && endId - startId <= K);

            // Get reference to thread local result structure once and have search work on it.
            auto localDropoffBuckets = dropoffBuckets.getThreadLocalBuckets();
            localDropoffBuckets.initForSearch();

            auto& localUpdateBuckets = updateBucketEntries.local();
            localUpdateBuckets.setCurLocalBuckets(&localDropoffBuckets);

            std::array<int, K> tailRanks{};
            std::array<int, K> dropoffOffsets{};

            for (int i = 0; i < K; ++i) {
                int tailRank;
                int dropoffOffset;
                if (startId + i < endId) {
                    tailRank = ch.rank(inputGraph.edgeTail(requestState.dropoffs[startId + i].loc));
                    dropoffOffset = inputGraph.travelTime(requestState.dropoffs[startId + i].loc);
                    
                } else {
                    tailRank = ch.rank(inputGraph.edgeTail(requestState.dropoffs[startId].loc));
                    dropoffOffset = inputGraph.travelTime(requestState.dropoffs[startId].loc); // Fill rest of a partial batch with copies of first in batch
                }
                tailRanks[i] = tailRank;
                dropoffOffsets[i] = dropoffOffset;
            }

            localUpdateBuckets.setCurDropoffBatchId(endId / K - 1);
            FillBucketsSearch &localFillBucketsSearch = fillBucketsSearch.local();

            localFillBucketsSearch.runWithOffset(tailRanks, dropoffOffsets);
            // After a search batch of K, write the bucket entries back to the global vectors
            dropoffBuckets.updateBucketEntriesInGlobalVectors(startId);

        }

        void runPickupSearches(const int startId, const int endId) {
            assert(endId > startId && endId - startId <= K);

            // Get reference to thread local result structure once and have search work on it.
            auto localPDDistances = distances.getThreadLocalPDDistances();
            localPDDistances.initForSearch();

            auto& localUpdateDistances = updatePDDistances.local();
            localUpdateDistances.setCurLocalDistances(&localPDDistances);


            std::array<int, K> zeroOffsets{};
            zeroOffsets.fill(0);

            std::array<int, K> headRanks{};

            for (int i = 0; i < K; ++i) {
                int headRank;
                if (startId + i < endId) {
                    headRank = ch.rank(inputGraph.edgeHead(requestState.pickups[startId + i].loc));
                    
                } else {
                    headRank = ch.rank(inputGraph.edgeHead(requestState.pickups[startId].loc)); // Fill rest of a partial batch with copies of first in batch
                }
                headRanks[i] = headRank;
            }

            localUpdateDistances.setCurFirstPickupId(startId);
            FindPDDistancesSearch &localFindPDDistancesSearch = findPDDistancesSearch.local();
            localFindPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);

            // After a search batch of K, write the pd distances back to the global vectors
            distances.updatePDDistancesInGlobalVectors(startId);

        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;
        VehicleToPDLocQueryT &vehicleToPDLocQuery;

        PDDistances<LabelSetT> &distances;

        int upperBoundDirectPDDist;

        tbb::enumerable_thread_specific<UpdateBucketEntries> updateBucketEntries;
        tbb::enumerable_thread_specific<UpdatePDDistances> updatePDDistances;

        BucketContainer dropoffBuckets;
        tbb::enumerable_thread_specific<FillBucketsSearch> fillBucketsSearch;
        tbb::enumerable_thread_specific<FindPDDistancesSearch> findPDDistancesSearch;

    };

}