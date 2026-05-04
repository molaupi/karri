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

#include "../RequestState/RequestState.h"

namespace karri {
    // Representation of PD-distances, i.e. distances from pickups to dropoffs.
    struct PDDistances {
    private:
        static size_t numLabelsPerDropoff(const int numPickups, const int k) {
            return numPickups / k + (numPickups % k != 0);
        }

    public:

        PDDistances() : numDropoffs(0), batchWidth(0), distances(), minDirectDist(INFTY), minDirectDistancesPerPickup() {}

        PDDistances(const int numPickups, const int numDropoffs, const int batchWidth)
            : numDropoffs(numDropoffs),
              batchWidth(batchWidth),
              distances(numLabelsPerDropoff(numPickups, batchWidth) * numDropoffs, INFTY),
              minDirectDist(INFTY),
              minDirectDistancesPerPickup(numLabelsPerDropoff(numPickups, batchWidth), INFTY) {
        }

        // IDs refer to the indices in the vectors of pickups/dropoffs given at the last initialize() call.
        int getDirectDistance(const unsigned int pickupId, const unsigned int dropoffId) const {
            const int res = distanceFor(pickupId, dropoffId);
            KASSERT(res < INFTY);
            return res;
        }

        int getDirectDistance(const PDLoc &pickup, const PDLoc &dropoff) const {
            return getDirectDistance(pickup.id, dropoff.id);
        }

        // Get distances for batch of pickups and one dropoff.
        // Make sure width of DistanceLabel matches the batch size K of this data structure and that firstPickupIdInBatch is a multiple of K.
        template<typename DistanceLabel>
        DistanceLabel getDirectDistancesForBatchOfPickups(const unsigned int firstPickupIdInBatch,
                                                          const unsigned int &dropoffId) const {
            KASSERT(firstPickupIdInBatch % batchWidth == 0);
            int const * const startOfLabel = &distanceFor(firstPickupIdInBatch, dropoffId);
            DistanceLabel label;
            label.load(startOfLabel);
            return label;
        }

        int getDistanceToDestinationFrom(const int pickupID) const {
            return getDirectDistance(pickupID, 0);
        }

        const int &getMinDirectDistance() const {
            return minDirectDist;
        }

        int getMinDirectDistanceForPickup(const int pickupId) const {
            return minDirectDistancesPerPickup[pickupId];
        }

        // Compares a current PD-distance to the given distance and updates the distances if the given distances are smaller.
        void updateDistanceIfSmaller(const unsigned int pickupId, const unsigned int dropoffId, const int dist) {
            int &cur = distanceFor(pickupId, dropoffId);
            if (dist < cur) {
                cur = dist;
                minDirectDist = std::min(minDirectDist, dist);

                if (dist < minDirectDistancesPerPickup[pickupId]) {
                    minDirectDistancesPerPickup[pickupId] = dist;
                }
                KASSERT(minDirectDist <= minDirectDistancesPerPickup[pickupId]);
            }
        }

        // Compares the current PD-distances for a batch of pickups to one dropoff to the given distances and updates the
        // ones for which the given distances are smaller.
        // Batch consists of K subsequent pickup IDs and is defined by the first ID in the batch.
        // Make sure that width of DistanceLabel matches the batch size K of this data structure.
        template<typename DistanceLabel>
        void updateDistanceBatchIfSmaller(const unsigned int firstPickupId, const unsigned int dropoffId,
                                          const DistanceLabel &dist) {
            KASSERT(firstPickupId % batchWidth == 0);
            int *const startOfLabel = &distanceFor(firstPickupId, dropoffId);
            DistanceLabel label;
            label.load(startOfLabel);
            const auto smaller = dist < label;
            if (anySet(smaller)) {
                label.setIf(dist, smaller);
                label.store(startOfLabel);
                minDirectDist = std::min(minDirectDist, dist.horizontalMin());
                label.load(minDirectDistancesPerPickup.data() + firstPickupId);
                label.setIf(dist, smaller);
                label.store(minDirectDistancesPerPickup.data() + firstPickupId);
                KASSERT(minDirectDist <= label.horizontalMin());
            }
        }

    private:
        int &distanceFor(const unsigned int pickupId, const unsigned int dropoffId) {
            return distances[((pickupId / batchWidth) * numDropoffs + dropoffId) * batchWidth + pickupId % batchWidth];
        }

        const int &distanceFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return distances[((pickupId / batchWidth) * numDropoffs + dropoffId) * batchWidth + pickupId % batchWidth];
        }

        int numDropoffs;
        int batchWidth; // Number of pickups in a batch.

        // The distance from pickup pickupId to dropoff dropoffId is stored in
        // distances[((pickupId / K) * numDropoffs + dropoffId) * K + (pickupId % K)]
        // Thus, the distances for pickups i * K,..., (i+1) * K - 1 to each dropoff (and to all dropoffs by increasing
        // ID) are always stored in consecutive memory. This allows batch-wise updates with searches batched by pickup ID.
        // If the number of pickups is not a multiple of K, we pad with unused distances to maintain memory alignment.
        AlignedVector<int> distances;

        int minDirectDist;
        AlignedVector<int> minDirectDistancesPerPickup;
    };
}