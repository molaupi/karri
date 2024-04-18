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
#include "DropoffsBucketContainer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "PDDistances.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/parallel_for.h"

namespace karri::PDDistanceQueryStrategies {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LabelSetT>
    class BCHStrategy {

    public:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

    private:

        using BucketContainer = DropoffsBucketContainer<DistanceLabel>;


        struct StopWhenMaxDistExceeded {

            explicit StopWhenMaxDistExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                const auto exceedsMaxDist = maxDist < distToV;
                const auto doesNotExceedMaxDist = ~exceedsMaxDist;
                const bool anyDoesNotExceedMaxDist = anySet(doesNotExceedMaxDist);
                return !anyDoesNotExceedMaxDist;
            }


        private:
            const int &maxDist;
        };

        struct WriteBucketEntry {
            explicit WriteBucketEntry(std::vector<std::pair<int, DistanceLabel>> &searchSpaceWithDistances)
                    : searchSpaceWithDistances(searchSpaceWithDistances) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                searchSpaceWithDistances.emplace_back(v, distToV);
                return false;
            }

        private:
            // Every time a vertex v is settled, the pair of v and its current DistanceLabel is stored into
            // searchSpaceWithDistances.
            std::vector<std::pair<int, DistanceLabel>> &searchSpaceWithDistances;
        };

        struct ScanDropoffBucketAndUpdatePDDistances {
            explicit ScanDropoffBucketAndUpdatePDDistances(RequestState &requestState, BucketContainer &dropoffBuckets,
                                                           PDDistances<LabelSetT>& distances,
                                                           int& firstPickupId)
                    : requestState(requestState),
                      dropoffBuckets(dropoffBuckets),
                      distances(distances),
                      firstPickupId(firstPickupId) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {

                const auto &bucket = dropoffBuckets.getBucketOf(v);

                if (bucket.size() == 0)
                    return false;

                for (int i = 0; i < requestState.numDropoffs(); ++i) {
                    const int &distFromVToDropoff = bucket[i / K][i % K];
                    distances.updateDistanceBatchIfSmaller(firstPickupId, i, distToV + distFromVToDropoff);
                }

                return false;
            }

        private:

            RequestState &requestState;
            BucketContainer &dropoffBuckets;
            PDDistances<LabelSetT> &distances;
            int &firstPickupId;
        };

        using FillBucketsSearch = typename CHEnvT::template UpwardSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>;
        using FindPDDistancesSearch = typename CHEnvT::template UpwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>;

        friend WriteBucketEntry;
        friend ScanDropoffBucketAndUpdatePDDistances;
        friend StopWhenMaxDistExceeded;

    public:

        BCHStrategy(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                    PDDistances<LabelSetT> &distances,
                    RequestState &requestState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  distances(distances),
                  searchSpaceWithDistances(),
                  dropoffBuckets(inputGraph.numVertices()),
                  fillBucketsSearch([&]() {
                      return chEnv.template getReverseSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>(
                              WriteBucketEntry(searchSpaceWithDistances.local()),
                              StopWhenMaxDistExceeded(upperBoundDirectPDDist));
                  }),
                  findPDDistancesSearch([&]() {
                      return chEnv.template getForwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>(
                              ScanDropoffBucketAndUpdatePDDistances(requestState, dropoffBuckets, distances,
                                                                    firstPickupIdInBatch.local()),
                              StopWhenMaxDistExceeded(upperBoundDirectPDDist));
                  }) {}


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
            int maxPickupToOriginVehDist = 0;
            for (const auto &pickup: requestState.pickups) {
                assert(pickup.vehDistToCenter != INFTY);
                maxPickupToOriginVehDist = std::max(maxPickupToOriginVehDist, pickup.vehDistToCenter);
            }

            int maxDestToDropoffVehDist = 0;
            for (const auto &dropoff: requestState.dropoffs) {
                assert(dropoff.vehDistFromCenter != INFTY);
                maxDestToDropoffVehDist = std::max(maxDestToDropoffVehDist, dropoff.vehDistFromCenter);
            }

            if (maxPickupToOriginVehDist >= INFTY || maxDestToDropoffVehDist >= INFTY) {
                upperBoundDirectPDDist = INFTY;
            } else {
                upperBoundDirectPDDist = maxPickupToOriginVehDist + requestState.originalReqDirectDist +
                                         maxDestToDropoffVehDist; // read by stopping criterion of searches
            }

            const int64_t initTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += initTime;
            timer.restart();

            // Fill dropoff buckets in parallel over dropoffs:
            tbb::parallel_for(int(0), static_cast<int>(requestState.numDropoffs()), K, [&](int i) {
                fillDropoffBuckets(i, std::min(i + K, static_cast<int>(requestState.numDropoffs())));
            });

            const int64_t dropoffBucketEntryGenTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.dropoffBucketEntryGenTime = dropoffBucketEntryGenTime;
            timer.restart();

            // Run pickup searches against dropoff buckets in parallel over pickups:
            tbb::parallel_for(int(0), static_cast<int>(requestState.numPickups()), K, [&](int i) {
                runPickupSearches(i, std::min(i + K, static_cast<int>(requestState.numPickups())));
            });

            distances.computeGlobalMinDirectDistance();
            requestState.minDirectPDDist = distances.getMinDirectDistance();

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

        void fillDropoffBuckets(const int startId, const int endId) {
            assert(endId > startId && endId - startId <= K);

            auto &localSearchSpaceWithDistances = searchSpaceWithDistances.local();
            localSearchSpaceWithDistances.clear();

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
                    dropoffOffset = inputGraph.travelTime(
                            requestState.dropoffs[startId].loc); // Fill rest of a partial batch with copies of first in batch
                }
                tailRanks[i] = tailRank;
                dropoffOffsets[i] = dropoffOffset;
            }

            FillBucketsSearch &localFillBucketsSearch = fillBucketsSearch.local();
            localFillBucketsSearch.runWithOffset(tailRanks, dropoffOffsets);

            // After a search batch of K, write the local results back to the global dropoff bucket container
            dropoffBuckets.updateDistancesInGlobalVectors(startId / K, localSearchSpaceWithDistances);
        }

        void runPickupSearches(const int startId, const int endId) {
            assert(endId > startId && endId - startId <= K);

            firstPickupIdInBatch.local() = startId;

            std::array<int, K> zeroOffsets{};
            zeroOffsets.fill(0);

            std::array<int, K> headRanks{};

            for (int i = 0; i < K; ++i) {
                int headRank;
                if (startId + i < endId) {
                    headRank = ch.rank(inputGraph.edgeHead(requestState.pickups[startId + i].loc));

                } else {
                    headRank = ch.rank(inputGraph.edgeHead(
                            requestState.pickups[startId].loc)); // Fill rest of a partial batch with copies of first in batch
                }
                headRanks[i] = headRank;
            }

            FindPDDistancesSearch &localFindPDDistancesSearch = findPDDistancesSearch.local();
            localFindPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);

            distances.computeMinDirectDistancesForPickupBatch(startId);
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;

        PDDistances<LabelSetT> &distances;

        int upperBoundDirectPDDist;

        tbb::enumerable_thread_specific<std::vector<std::pair<int, DistanceLabel>>> searchSpaceWithDistances;
        tbb::enumerable_thread_specific<int> firstPickupIdInBatch;

        BucketContainer dropoffBuckets;
        tbb::enumerable_thread_specific<FillBucketsSearch> fillBucketsSearch;
        tbb::enumerable_thread_specific<FindPDDistancesSearch> findPDDistancesSearch;

    };

}