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
#include "Parallel/atomic_wrapper.h"

#include <tbb/enumerable_thread_specific.h>
#include <atomic>

namespace karri {

// Representation of PD-distances, i.e. distances from pickups to dropoffs.

    template<typename LabelSetT>
    struct PDDistances {
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        static constexpr int K = LabelSetT::K;

    public:

        PDDistances(const RequestState &requestState) 
            : requestState(requestState) {}


        void clear() {
            minDirectDist.store(INFTY, std::memory_order_relaxed);
            const int numLabelsPerDropoff = (requestState.numPickups() / K + (requestState.numPickups() % K != 0));
            const int numNeededLabels = numLabelsPerDropoff * requestState.numDropoffs();
            distances.clear();
            distances.resize(numNeededLabels, DistanceLabel(INFTY));


            // minDirectDistancesPerPickup has one entry per pickup. Initialize to INFTY.
            minDirectDistancesPerPickup.clear();
            minDirectDistancesPerPickup.resize(requestState.numPickups());

            for (auto& min : minDirectDistancesPerPickup)
                min.store(INFTY, std::memory_order_relaxed);
        }

        // IDs refer to the indices in the vectors of pickups/dropoffs given at the last initialize() call.
        int getDirectDistance(const unsigned int pickupId, const unsigned int dropoffId) const {
            assert(pickupId < requestState.numPickups());
            assert(dropoffId < requestState.numDropoffs());
            const int res = labelFor(pickupId, dropoffId)[pickupId % K];
            assert(res < INFTY);
            return res;
        }

        int getDirectDistance(const PDLoc &pickup, const PDLoc &dropoff) const {
            return getDirectDistance(pickup.id, dropoff.id);
        }

        const DistanceLabel &getDirectDistancesForBatchOfPickups(const unsigned int firstPickupIdInBatch,
                                                                 const unsigned int &dropoffId) const {
            return labelFor(firstPickupIdInBatch, dropoffId);
        }

        int getDistanceToDestinationFrom(const int pickupID) const {
            return getDirectDistance(pickupID, 0);
        }

        const std::atomic_int &getMinDirectDistance() const {
            return minDirectDist;
        }

        int getMinDirectDistanceForPickup(const int pickupId) const {
            return minDirectDistancesPerPickup[pickupId].load(std::memory_order_relaxed);
        }

        // Compares a current PD-distance to the given distance and updates the distances if the given distances are smaller.
        void updateDistanceIfSmaller(const unsigned int pickupId, const unsigned int dropoffId, const int dist) {
            const auto offsetInBatch = pickupId % K;
            auto &label = labelFor(pickupId, dropoffId);
            if (dist < label[offsetInBatch]) {
                label[offsetInBatch] = dist;

                auto &minDirectDistAtomic = minDirectDist;
                int expectedMin = minDirectDistAtomic.load(std::memory_order_relaxed);
                while (expectedMin > dist &&
                        !minDirectDistAtomic.compare_exchange_strong(expectedMin, dist, std::memory_order_relaxed));


                auto &minDistForPickupAtomic = minDirectDistancesPerPickup[pickupId];
                int expectedMinForPickup = minDistForPickupAtomic.load(std::memory_order_relaxed);
                while (expectedMinForPickup > dist &&
                        !minDistForPickupAtomic.compare_exchange_strong(expectedMinForPickup, dist, std::memory_order_relaxed));

                assert(minDirectDist <= minDirectDistancesPerPickup[pickupId]);
            }
        }
        
        // Compares the current PD-distances for a batch of pickups to one dropoff to the given distances and updates the
        // ones for which the given distances are smaller.
        // Batch consists of K subsequent pickup IDs and is defined by the first ID in the batch.
        void updateDistanceBatchIfSmaller(const unsigned int firstPickupId, const unsigned int dropoffId,
                                          const DistanceLabel &dist) {
            auto &label = labelFor(firstPickupId, dropoffId);
            const auto smaller = dist < label;
            if (anySet(smaller)) {
                label.setIf(dist, smaller);

                const int minNewDist = dist.horizontalMin();
                auto &minDirectDistAtomic = minDirectDist;
                int expectedMin = minDirectDistAtomic.load(std::memory_order_relaxed);
                while (expectedMin > minNewDist &&
                        !minDirectDistAtomic.compare_exchange_strong(expectedMin, minNewDist, std::memory_order_relaxed));

                for (int i = 0; i < K; ++i) {
                    const int pickupId = firstPickupId + i;
                    if (pickupId < requestState.numPickups() && smaller[i]) {
                        auto &minDistForPickupAtomic = minDirectDistancesPerPickup[pickupId];
                        int expectedMinForPickup = minDistForPickupAtomic.load(std::memory_order_relaxed);
                        while (expectedMinForPickup > dist[i] &&
                                !minDistForPickupAtomic.compare_exchange_strong(expectedMinForPickup, dist[i], std::memory_order_relaxed));
                    }
                    assert(pickupId >= requestState.pickups.size() || minDirectDist <= minDirectDistancesPerPickup[pickupId]);
                }
            }
        }


    private:

        DistanceLabel &labelFor(const unsigned int pickupId, const unsigned int dropoffId) {
            return distances[(pickupId / K) * requestState.numDropoffs() + dropoffId];
        }

        const DistanceLabel &labelFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return distances[(pickupId / K) * requestState.numDropoffs() + dropoffId];
        }

        const RequestState &requestState;

        // Distances are stored as vectors of size K (DistanceLabel).
        // ceil(numPickups / K) labels per dropoff, sequentially arranged by increasing dropoffId.
        AlignedVector<DistanceLabel> distances;
        std::atomic_int minDirectDist;
        AlignedVector<CAtomic<int>> minDirectDistancesPerPickup;
    };
}