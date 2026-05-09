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
#include "DirectTransferDistances.h"

namespace karri {

    template<typename CHEnvT,
            typename LabelSetT
    >
    class BCHDirectTransferDistancesFinder {

    public:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

    private:

        struct PdLocBatchLabel {

            static constexpr unsigned int invalid_target = std::numeric_limits<unsigned int>::max();

            unsigned int targetId = invalid_target; // the batch id for this batch of K PD locs with sequential ids (i.e. targetId ranges from 0 to numPdLocs / K ( + 1)).
            DistanceLabel distToPdLoc = INFTY; // the distances from this vertex to the K PD locs

            friend bool operator==(const PdLocBatchLabel &lhs, const PdLocBatchLabel &rhs) noexcept {
                return lhs.targetId == rhs.targetId;
            }

            void cmpAndUpdate(const PdLocBatchLabel &other) {
                KASSERT(targetId == invalid_target || targetId == other.targetId);
                distToPdLoc.min(other.distToPdLoc);
                if (targetId == invalid_target)
                    targetId = other.targetId;
            }
        };

        using BucketContainer = SharedSearchSpaceBucketContainer<PdLocBatchLabel>;

        struct WriteBucketEntry {
            explicit WriteBucketEntry(BCHDirectTransferDistancesFinder &computer) : computer(computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                computer.pdLocBuckets.insertOrUpdate(v, {computer.curPdLocBatchId, distToV});
                return false;
            }

        private:

            BCHDirectTransferDistancesFinder &computer;
        };

        struct ScanPdLocBucketAndUpdatePDDistances {
            explicit ScanPdLocBucketAndUpdatePDDistances(BCHDirectTransferDistancesFinder &computer) : computer(
                    computer) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                for (const auto &pdLocBatchLabel: computer.pdLocBuckets.getBucketOf(v)) {
                    // Update distances to each dropoff in the batch label:
                    if (pdLocBatchLabel.targetId == PdLocBatchLabel::invalid_target)
                        continue;
                    const auto firstPdLocIdInBatch = pdLocBatchLabel.targetId * K;
                    computer.updateDistances(firstPdLocIdInBatch, computer.curTransferRank,
                                             distToV[0] + pdLocBatchLabel.distToPdLoc);
                }
                return false;
            }

        private:

            BCHDirectTransferDistancesFinder &computer;
        };

        using FillBucketsSearch = typename CHEnvT::template UpwardSearch<WriteBucketEntry, dij::NoCriterion, LabelSetT>;

        using FindDistancesLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<ScanPdLocBucketAndUpdatePDDistances, dij::NoCriterion, FindDistancesLabelSet>;

        friend WriteBucketEntry;
        friend ScanPdLocBucketAndUpdatePDDistances;

    public:

        BCHDirectTransferDistancesFinder(const int numVertices,
                                         const CHEnvT &chEnv,
                                         const PDLocType& type)
                : ch(chEnv.getCH()),
                  distances(numVertices),
                  pdLocBuckets(numVertices),
                  fillBucketsSearch(type == PDLocType::PICKUP ?
                                    chEnv.template getForwardSearch<WriteBucketEntry, dij::NoCriterion, LabelSetT>(
                                            WriteBucketEntry(*this)) :
                                    chEnv.template getReverseSearch<WriteBucketEntry, dij::NoCriterion, LabelSetT>(
                                            WriteBucketEntry(*this))),
                  findDistancesSearch(type == PDLocType::PICKUP ?
                                        chEnv.template getReverseSearch<ScanPdLocBucketAndUpdatePDDistances, dij::NoCriterion, FindDistancesLabelSet>(
                                                ScanPdLocBucketAndUpdatePDDistances(*this)) :
                                        chEnv.template getForwardSearch<ScanPdLocBucketAndUpdatePDDistances, dij::NoCriterion, FindDistancesLabelSet>(
                                                ScanPdLocBucketAndUpdatePDDistances(*this))) {}


        // Runs selection phase for PD locs for many-to-many searches. In BCH strategy, this means generating bucket
        // entries for PD locs rooted at given ranks and offsets.
        void runSelectionForPdLocs(const std::vector<int> &pdLocRanks, const std::vector<int> &pdLocOffsets) {
            KASSERT(pdLocRanks.size() == pdLocOffsets.size());
            const auto numPdLocs = pdLocRanks.size();
            distances.init(numPdLocs);

            const auto numPdLocBatches = numPdLocs / K + (numPdLocs % K != 0);
            pdLocBuckets.init(numPdLocBatches);

            // Fill pdLoc buckets:
            std::array<int, K> batchRanks{};
            std::array<int, K> batchOffsets{};

            // Run entry generation searches for full pdLoc batches
            int pdLocId = 0;
            for (; pdLocId < numPdLocs;) {
                batchRanks[pdLocId % K] = pdLocRanks[pdLocId];
                batchOffsets[pdLocId % K] = pdLocOffsets[pdLocId];
                ++pdLocId;
                if (pdLocId % K == 0) {
                    curPdLocBatchId = pdLocId / K - 1;
                    fillBucketsSearch.runWithOffset(batchRanks, batchOffsets);
                }
            }
            // Finish potential partially filled batch
            if (pdLocId % K != 0) {
                while (pdLocId % K != 0) {
                    batchRanks[pdLocId % K] = batchRanks[0]; // fill with copies of first in batch
                    batchOffsets[pdLocId % K] = batchOffsets[0];
                    ++pdLocId;
                }
                curPdLocBatchId = pdLocId / K - 1;
                fillBucketsSearch.runWithOffset(batchRanks, batchOffsets);
            }
        }

        // Runs query for given CH rank using buckets generated by last call to runSelectionForPdLocs().
        // Results are stored in DirectTransferDistances object, which can be queried with PD loc and transfer rank.
        // Does not allow passing an offset as multiple transfer edges may use same rank with different offsets.
        void runQueryForTransferRank(const int transferRank) {
            if (distances.knowsDistancesForTransferRank(transferRank))
                return;
            distances.allocateEntriesFor(transferRank);
            curTransferRank = transferRank;
            findDistancesSearch.run(transferRank);
        }

        const DirectTransferDistances<LabelSetT> &getDistances() const {
            return distances;
        }

    private:

        void
        updateDistances(const unsigned int firstPdLocId, const unsigned int transferRank, const DistanceLabel &dist) {
            distances.updateDistanceBatchIfSmaller(firstPdLocId, transferRank, dist);
        }

        const CH &ch;

        DirectTransferDistances<LabelSetT> distances;

        BucketContainer pdLocBuckets;
        FillBucketsSearch fillBucketsSearch;
        FindDistancesSearch findDistancesSearch;

        unsigned int curTransferRank;
        unsigned int curPdLocBatchId;
    };
}