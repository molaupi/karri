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
    // Represents information that one thread computes during one bucket search. Can be incorporated into
    // global result at end of search.
    class ThreadLocalPDDistances {

        friend PDDistances;

    public:

        ThreadLocalPDDistances(const RequestState &requestState,
                                        std::vector<int>& indexInDistancesVector,
                                        std::vector<DistanceLabel>& distancesForSinglePickup) :
                requestState(requestState),
                indexInDistancesVector(indexInDistancesVector),
                distancesForSinglePickup(distancesForSinglePickup) {}

            void initForSearch() {
                distancesForSinglePickup.clear();
                const int numDropoffs = requestState.numDropoffs();

                if (indexInDistancesVector.size() < numDropoffs)
                    indexInDistancesVector.resize(numDropoffs);
                for (int i = 0; i < numDropoffs; ++i)
                    indexInDistancesVector[i] = INVALID_INDEX;
            }

            // Compares the current PD-distances for a batch of pickups to one dropoff to the given distances and updates the
            // ones for which the given distances are smaller.
            // Batch consists of K subsequent pickup IDs and is defined by the first ID in the batch.
            void updateDistanceBatchIfSmaller(const unsigned int, const unsigned int dropoffId,
                                            const DistanceLabel &dist) {
                if (indexInDistancesVector[dropoffId] == INVALID_INDEX) {
                    allocateLocalEntriesFor(dropoffId);
                }
                
                const auto &idx = indexInDistancesVector[dropoffId];
                auto &label = distancesForSinglePickup[idx];

                const auto smaller = dist < label;
                if (anySet(smaller)) {
                    label.setIf(dist, smaller);
                }
            }
            

        private:

            // Dynamic Allocation
            void allocateLocalEntriesFor(const int dropoffId) {
                assert(indexInDistancesVector[dropoffId] == INVALID_INDEX);

                indexInDistancesVector[dropoffId] = distancesForSinglePickup.size();
                distancesForSinglePickup.push_back(DistanceLabel(INFTY));
            }

            const RequestState &requestState;
            
            std::vector<int> &indexInDistancesVector;
            std::vector<DistanceLabel> &distancesForSinglePickup;

        };

        PDDistances(const RequestState &requestState) 
            : requestState(requestState),
              indexInDistancesVector(),
              distancesForSinglePickup() {}

        // Each thread gets an instance of a ThreadLocalPDDistances at the beginning of a search. This
        // object encapsulates the local result of the thread for that search. This way, the underlying TLS structures
        // are only queried once per search.
        ThreadLocalPDDistances getThreadLocalPDDistances() {
            return ThreadLocalPDDistances(requestState, indexInDistancesVector.local(), distancesForSinglePickup.local());
        }

        void clear() {
            minDirectDist.store(INFTY, std::memory_order_relaxed);
            const int numLabelsPerDropoff = (requestState.numPickups() / K + (requestState.numPickups() % K != 0));
            const int numNeededLabels = numLabelsPerDropoff * requestState.numDropoffs();
            distances.clear();
            distances.resize(numNeededLabels, DistanceLabel(INFTY));


            // minDirectDistancesPerPickup has one entry per pickup. Initialize to INFTY.
            minDirectDistancesPerPickup.clear();
            minDirectDistancesPerPickup.resize(numLabelsPerDropoff, DistanceLabel(INFTY));
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
            return minDirectDistancesPerPickup[pickupId / K][pickupId % K];
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

                if (dist < minDirectDistancesPerPickup[pickupId / K][offsetInBatch]) {
                    minDirectDistancesPerPickup[pickupId / K][offsetInBatch] = dist;
                }
                assert(minDirectDist <= minDirectDistancesPerPickup[pickupId / K].horizontalMin());
            }
        }
        
        void updatePDDistancesInGlobalVectors(const unsigned int pickupId) {
            const auto offsetInBatch = pickupId % K;

            const auto &localIndices = indexInDistancesVector.local();
            const auto &localEntries = distancesForSinglePickup.local();

            for (int dropoffId = 0; dropoffId < requestState.numDropoffs(); ++dropoffId) {
                const auto &idx = localIndices[dropoffId];
                const auto &localDist = localEntries[idx];
                if (pickupId == 9 && dropoffId == 0 && requestState.originalRequest.requestId == 0) {
                    std::cout << "";
                }

                if (idx != INVALID_INDEX) {
                    auto &label = labelFor(pickupId, dropoffId);
                    const auto smaller = localDist < label;
                    if (anySet(smaller)) {
                        label.setIf(localDist, smaller);
                        
                        const int minNewDist = localDist.horizontalMin();
                        auto &minDirectDistAtomic = minDirectDist;
                        int expectedMin = minDirectDistAtomic.load(std::memory_order_relaxed);
                        while (expectedMin > minNewDist &&
                                !minDirectDistAtomic.compare_exchange_strong(expectedMin, minNewDist, std::memory_order_relaxed));

                        if (minNewDist < minDirectDistancesPerPickup[pickupId / K][offsetInBatch]) {
                            minDirectDistancesPerPickup[pickupId / K][offsetInBatch] = minNewDist;
                        }
                        assert(minDirectDist <= minDirectDistancesPerPickup[pickupId / K].horizontalMin());
                    }
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
        AlignedVector<DistanceLabel> minDirectDistancesPerPickup;

        // Thread Local Storage for local bucket entries calculation
        tbb::enumerable_thread_specific<std::vector<int>> indexInDistancesVector;
        tbb::enumerable_thread_specific<std::vector<DistanceLabel>> distancesForSinglePickup;
    };
}