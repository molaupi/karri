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

        struct WriteBucketEntry {
            explicit WriteBucketEntry(BCHStrategy &computer) : computer(computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                computer.dropoffBuckets.insertOrUpdate(v, {computer.dropoffBatchId, distToV});
                return false;
            }

        private:

            BCHStrategy &computer;
        };

        struct ScanDropoffBucketAndUpdatePDDistances {
            explicit ScanDropoffBucketAndUpdatePDDistances(BCHStrategy &computer) : computer(computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                for (const auto &dropoffBatchLabel: computer.dropoffBuckets.getBucketOf(v)) {
                    // Update distances to each dropoff in the batch label:
                    const auto firstDropoffIdInBatch = dropoffBatchLabel.targetId * K;
                    if (firstDropoffIdInBatch == DropoffBatchLabel::invalid_target)
                        continue;
                    for (int i = 0; i < K && firstDropoffIdInBatch + i < computer.requestState.numDropoffs(); ++i) {
                        computer.updatePDDistances(computer.curFirstPickupId, firstDropoffIdInBatch + i,
                                                   distToV + dropoffBatchLabel.distToDropoff[i]);
                    }
                }
                return false;
            }

        private:

            BCHStrategy &computer;
        };

        using FillBucketsSearch = typename CHEnvT::template UpwardSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>;
        using FindPDDistancesSearch = typename CHEnvT::template UpwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>;

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
                  dropoffBuckets(inputGraph.numVertices()),
                  fillBucketsSearch(
                          chEnv.template getReverseSearch<WriteBucketEntry, StopWhenMaxDistExceeded, LabelSetT>(
                                  WriteBucketEntry(*this), StopWhenMaxDistExceeded(upperBoundDirectPDDist))),
                  findPDDistancesSearch(
                          chEnv.template getForwardSearch<ScanDropoffBucketAndUpdatePDDistances, StopWhenMaxDistExceeded, LabelSetT>(
                                  ScanDropoffBucketAndUpdatePDDistances(*this),
                                  StopWhenMaxDistExceeded(upperBoundDirectPDDist))) {}


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


            const int64_t initTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += initTime;
            timer.restart();

            // Fill dropoff buckets:
            std::array<int, K> tailRanks{};
            std::array<int, K> dropoffOffsets{};
            // Run searches for full pickup batches
            int dropoffId = 0;
            for (; dropoffId < requestState.numDropoffs();) {
                tailRanks[dropoffId % K] = ch.rank(inputGraph.edgeTail(requestState.dropoffs[dropoffId].loc));
                dropoffOffsets[dropoffId % K] = inputGraph.travelTime(requestState.dropoffs[dropoffId].loc);
                ++dropoffId;
                if (dropoffId % K == 0) {
                    dropoffBatchId = dropoffId / K - 1;
                    fillBucketsSearch.runWithOffset(tailRanks, dropoffOffsets);
                }
            }
            // Finish potential partially filled batch
            if (dropoffId % K != 0) {
                while (dropoffId % K != 0) {
                    tailRanks[dropoffId % K] = tailRanks[0]; // fill with copies of first in batch
                    dropoffOffsets[dropoffId % K] = dropoffOffsets[0];
                    ++dropoffId;
                }
                dropoffBatchId = dropoffId / K - 1;
                fillBucketsSearch.runWithOffset(tailRanks, dropoffOffsets);
            }

            const int64_t dropoffBucketEntryGenTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.dropoffBucketEntryGenTime = dropoffBucketEntryGenTime;
            timer.restart();

            // Run pickup searches against dropoff buckets:
            std::array<int, K> zeroOffsets{};
            zeroOffsets.fill(0);

            std::array<int, K> headRanks{};
            // Run searches for full pickup batches
            int pickupId = 0;
            for (; pickupId < requestState.numPickups();) {
                headRanks[pickupId % K] = ch.rank(inputGraph.edgeHead(requestState.pickups[pickupId].loc));
                ++pickupId;
                if (pickupId % K == 0) {
                    curFirstPickupId = pickupId - K;
                    findPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);
                }
            }

            // Finish potential partially filled batch
            if (pickupId % K != 0) {
                while (pickupId % K != 0) {
                    headRanks[pickupId % K] = headRanks[0]; // fill with copies of first in batch
                    ++pickupId;
                }
                curFirstPickupId = pickupId - K;
                findPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);
            }


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

        void
        updatePDDistances(const unsigned int firstPickupId, const unsigned int dropoffId, const DistanceLabel &dist) {
            distances.updateDistanceBatchIfSmaller(firstPickupId, dropoffId, dist);
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;
        VehicleToPDLocQueryT &vehicleToPDLocQuery;

        PDDistances<LabelSetT> &distances;

        BucketContainer dropoffBuckets;
        FillBucketsSearch fillBucketsSearch;
        FindPDDistancesSearch findPDDistancesSearch;

        int upperBoundDirectPDDist;
        unsigned int curFirstPickupId;
        unsigned int dropoffBatchId;
    };

}