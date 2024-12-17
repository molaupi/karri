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

#include "Tools/Simd/AlignedVector.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

// Representation of PD-distances, i.e. distances from pickups to dropoffs.

    template<typename LabelSetT>
    struct PDDistances {
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        static constexpr int K = LabelSetT::K;

    private:
        static size_t numLabelsPerDropoff(const int numPickups) {
            return numPickups / K + (numPickups % K != 0);
        }

    public:

        PDDistances(const int numPickups, const int numDropoffs) :
                numPickups(numPickups),
                numDropoffs(numDropoffs),
                distances(numLabelsPerDropoff(numPickups) * numDropoffs, INFTY),
                minDirectDist(INFTY),
                minDirectDistancesPerPickup(numLabelsPerDropoff(numPickups), INFTY) {}

        // IDs refer to the indices in the vectors of pickups/dropoffs given at the last initialize() call.
        int getDirectDistance(const unsigned int pickupId, const unsigned int dropoffId) const {
            const int res = labelFor(pickupId, dropoffId)[pickupId % K];
            KASSERT(res < INFTY);
            return res;
        }

        int getDirectDistance(const PDLoc &pickup, const PDLoc &dropoff) const {
            return getDirectDistance(pickup.id, dropoff.id);
        }

        const DistanceLabel &getDirectDistancesForBatchOfPickups(const unsigned int firstPickupIdInBatch,
                                                                 const unsigned int &dropoffId) const {
            return labelFor(firstPickupIdInBatch, dropoffId);
        }

        int getMinDirectDistance() const {
            return minDirectDist;
        }

        int getMinDirectDistanceForPickup(const int pickupId) const {
            return minDirectDistancesPerPickup[pickupId / K][pickupId % K];
        }

        // Compares a current PD-distance to the given distance and updates the distances if the given distances are smaller.
        void updateDistanceIfSmaller(const unsigned int pickupId, const unsigned int dropoffId, const int dist) {
            const auto offsetInBatch = pickupId % K;
            auto &label = labelFor(pickupId, dropoffId);
            if (dist < label[offsetInBatch]) {
                label[offsetInBatch] = dist;
            }
        }

        // Compares the current PD-distances for a batch of pickups to one dropoff to the given distances and updates the
        // ones for which the given distances are smaller.
        // Batch consists of K subsequent pickup IDs and is defined by the first ID in the batch.
        void updateDistanceBatchIfSmaller(const unsigned int firstPickupId, const unsigned int dropoffId,
                                          const DistanceLabel &dist) {
            auto &label = labelFor(firstPickupId, dropoffId);
            label.min(dist);
        }

        void computeGlobalMinDirectDistance() {
            for (int firstPickupIdInBatch = 0;
                 firstPickupIdInBatch < numPickups; firstPickupIdInBatch += K) {
                DistanceLabel &minLabelForBatch = minDirectDistancesPerPickup[firstPickupIdInBatch / K];
                minDirectDist = std::min(minDirectDist, minLabelForBatch.horizontalMin());

            }
        }

        void computeMinDirectDistancesForPickupBatch(const int firstPickupIdInBatch) {
            DistanceLabel &minLabelForBatch = minDirectDistancesPerPickup[firstPickupIdInBatch / K];
            for (int dropoffId = 0; dropoffId < numDropoffs; ++dropoffId) {
                const auto &label = labelFor(firstPickupIdInBatch, dropoffId);
                minLabelForBatch.min(label);
            }
        }

    private:

        DistanceLabel &labelFor(const unsigned int pickupId, const unsigned int dropoffId) {
            return distances[(pickupId / K) * numDropoffs + dropoffId];
        }

        const DistanceLabel &labelFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return distances[(pickupId / K) * numDropoffs + dropoffId];
        }

        int numPickups;
        int numDropoffs;

        // Distances are stored as vectors of size K (DistanceLabel).
        // Layout is [ [p(i * K)..p((i+1) * K - 1)] x d(0) .. [p(i * K)..p((i+1) * K - 1)] x d(Nd-1) ] for i=0 to Np / K
        AlignedVector<DistanceLabel> distances;

        int minDirectDist;
        std::vector<DistanceLabel> minDirectDistancesPerPickup;
    };
}