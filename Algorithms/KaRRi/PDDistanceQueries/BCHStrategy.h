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
            typename LabelSetT>
    class BCHStrategy {

    public:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

    private:

        using TravelTimes = StampedDistanceLabelContainer<DistanceLabel>;

        struct DropoffBatchLabel {

            static constexpr unsigned int invalid_target = std::numeric_limits<unsigned int>::max();

            unsigned int targetId = invalid_target; // the batch id for this batch of K dropoffs with sequential ids (i.e. targetId ranges from 0 to numDropoffs / K ( + 1)).
            DistanceLabel costToDropoff = INFTY; // the traversal costs from this vertex to the K dropoffs (shortest distances)
            DistanceLabel travelTimeToDropoff = INFTY; // the travel times from this vertex to the K dropoffs (not necessarily shortest distances)

            friend bool operator==(const DropoffBatchLabel &lhs, const DropoffBatchLabel &rhs) noexcept {
                return lhs.targetId == rhs.targetId;
            }

            void cmpAndUpdate(const DropoffBatchLabel &other) {
                assert(targetId == invalid_target || targetId == other.targetId);
                const auto improved = (other.costToDropoff < costToDropoff) |
                        ((other.costToDropoff == costToDropoff) & (other.travelTimeToDropoff < travelTimeToDropoff));
                costToDropoff.setIf(other.costToDropoff, improved);
                travelTimeToDropoff.setIf(other.travelTimeToDropoff, improved);
                if (targetId == invalid_target)
                    targetId = other.targetId;
            }
        };

        using BucketContainer = SharedSearchSpaceBucketContainer<DropoffBatchLabel>;


        struct WriteBucketEntry {
            explicit WriteBucketEntry(BCHStrategy &computer) : computer(computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &costToV, const DistLabelContT &) {
                KASSERT((costToV < INFTY) == (computer.travelTimes[v] < INFTY));
                computer.dropoffBuckets.insertOrUpdate(v, {computer.dropoffBatchId, costToV, computer.travelTimes[v]});
                return false;
            }

        private:

            BCHStrategy &computer;
        };

        struct ScanDropoffBucketAndUpdatePDDistances {
            explicit ScanDropoffBucketAndUpdatePDDistances(BCHStrategy &computer) : computer(computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &costToV, const DistLabelContT &) {
                KASSERT((costToV < INFTY) == (computer.travelTimes[v] < INFTY));
                for (const auto &dropoffBatchLabel: computer.dropoffBuckets.getBucketOf(v)) {
                    // Update distances to each dropoff in the batch label:
                    const auto firstDropoffIdInBatch = dropoffBatchLabel.targetId * K;
                    if (firstDropoffIdInBatch == DropoffBatchLabel::invalid_target)
                        continue;
                    for (int i = 0; i < K && firstDropoffIdInBatch + i < computer.requestState.numDropoffs(); ++i) {
                        computer.updatePDDistances(computer.curFirstPickupId, firstDropoffIdInBatch + i,
                                                   costToV + dropoffBatchLabel.costToDropoff[i],
                                                   computer.travelTimes[v] + dropoffBatchLabel.travelTimeToDropoff[i]);
                    }
                }
                return false;
            }

        private:

            BCHStrategy &computer;
        };

        struct UpdateTravelTimeCallback {

            UpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                     TravelTimes &travelTimes) : searchGraph(searchGraph), travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                travelTimes[w].setIf(travelTimes[v] + searchGraph.travelTime(e), improved);
            }

            const CH::SearchGraph &searchGraph;
            TravelTimes &travelTimes;
        };

        using FillBucketsSearch = typename CHEnvT::template UpwardSearch<WriteBucketEntry, dij::NoCriterion, UpdateTravelTimeCallback, LabelSetT>;
        using FindPDDistancesSearch = typename CHEnvT::template UpwardSearch<ScanDropoffBucketAndUpdatePDDistances, dij::NoCriterion, UpdateTravelTimeCallback, LabelSetT>;

        friend WriteBucketEntry;
        friend ScanDropoffBucketAndUpdatePDDistances;

    public:

        BCHStrategy(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                    PDDistances<LabelSetT> &distances,
                    RequestState &requestState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  distances(distances),
                  travelTimes(inputGraph.numVertices()),
                  dropoffBuckets(inputGraph.numVertices()),
                  fillBucketsSearch(
                          chEnv.template getReverseSearch<WriteBucketEntry, dij::NoCriterion, UpdateTravelTimeCallback, LabelSetT>(
                                  WriteBucketEntry(*this), {}, UpdateTravelTimeCallback(ch.downwardGraph(), travelTimes))),
                  findPDDistancesSearch(
                          chEnv.template getForwardSearch<ScanDropoffBucketAndUpdatePDDistances, dij::NoCriterion, UpdateTravelTimeCallback, LabelSetT>(
                                  ScanDropoffBucketAndUpdatePDDistances(*this), {}, UpdateTravelTimeCallback(ch.upwardGraph(), travelTimes))) {}


        // Computes all distances from every pickup to every dropoff and stores them in the given DirectPDDistances.
        void run() {
            Timer timer;

            const auto numDropoffSearches = requestState.numDropoffs() / K + (requestState.numDropoffs() % K != 0);
            dropoffBuckets.init(numDropoffSearches);

            const int64_t initTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += initTime;
            timer.restart();

            // Fill dropoff buckets:
            std::array<int, K> tailRanks{};
            std::array<int, K> dropoffOffsets{};
            // Run searches for full pickup batches
            int dropoffId = 0;
            travelTimes.init();
            for (; dropoffId < requestState.numDropoffs();) {
                tailRanks[dropoffId % K] = ch.rank(inputGraph.edgeTail(requestState.dropoffs[dropoffId].loc));
                dropoffOffsets[dropoffId % K] = inputGraph.traversalCost(requestState.dropoffs[dropoffId].loc);
                travelTimes[tailRanks[dropoffId % K]][dropoffId % K] = inputGraph.travelTime(requestState.dropoffs[dropoffId].loc);
                ++dropoffId;
                if (dropoffId % K == 0) {
                    dropoffBatchId = dropoffId / K - 1;
                    fillBucketsSearch.runWithOffset(tailRanks, dropoffOffsets);
                    travelTimes.init();
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
            travelTimes.init();
            // Run searches for full pickup batches
            int pickupId = 0;
            for (; pickupId < requestState.numPickups();) {
                headRanks[pickupId % K] = ch.rank(inputGraph.edgeHead(requestState.pickups[pickupId].loc));
                travelTimes[headRanks[pickupId % K]][pickupId % K] = 0;
                ++pickupId;
                if (pickupId % K == 0) {
                    curFirstPickupId = pickupId - K;
                    findPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);
                    travelTimes.init();
                }
            }

            // Finish potential partially filled batch
            if (pickupId % K != 0) {
                while (pickupId % K != 0) {
                    headRanks[pickupId % K] = headRanks[0]; // fill with copies of first in batch
                    travelTimes[headRanks[pickupId % K]][pickupId % K] = 0;
                    ++pickupId;
                }
                curFirstPickupId = pickupId - K;
                findPDDistancesSearch.runWithOffset(headRanks, zeroOffsets);
            }

            requestState.minDirectPDCost = distances.getMinCost();
            requestState.minDirectPDTravelTime = distances.getMinTravelTime();

            const int64_t pickupSearchesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.pickupBchSearchTime += pickupSearchesTime;
        }

        void init() {
            Timer timer;
            distances.clear();
//            distances.updateDistanceIfSmaller(0, 0, requestState.directDistInFullVeh);
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += time;
        }

    private:

        void
        updatePDDistances(const unsigned int firstPickupId, const unsigned int dropoffId, const DistanceLabel &cost, const DistanceLabel & travelTime) {
            distances.updateDistanceBatchIfSmaller(firstPickupId, dropoffId, cost, travelTime);
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;

        PDDistances<LabelSetT> &distances;

        TravelTimes travelTimes;
        BucketContainer dropoffBuckets;
        FillBucketsSearch fillBucketsSearch;
        FindPDDistancesSearch findPDDistancesSearch;

        unsigned int curFirstPickupId;
        unsigned int dropoffBatchId;
    };

}