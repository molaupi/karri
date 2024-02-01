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

#include <cassert>
#include <vector>
#include "Tools/Constants.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "DataStructures/Utilities/Permutation.h"
#include "Parallel/atomic_wrapper.h"

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {


// Data structure for dynamically tracking distances from last stops to PD locs.
// Allocates entries for distances from a last stop s to all PD locs when one relevant distance from s is found
// for the first time.
    template<typename LabelSetT>
    class TentativeLastStopDistances {

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

    public:

        // Represents information that one thread computes during one Last Stop BCH search. Can be incorporated into
        // global result at end of search.
        class ThreadLocalTentativeLastStopDistances {

            friend TentativeLastStopDistances;

        public:

            ThreadLocalTentativeLastStopDistances(std::vector<int> &indexInDistanceVector,
                                                  std::vector<DistanceLabel> &distance) :
                    indexInDistanceVector(indexInDistanceVector),
                    distance(distance) {}

            void initForSearch() {
                distance.clear();

                for (int i = 0; i < indexInDistanceVector.size(); ++i)
                    indexInDistanceVector[i] = INVALID_INDEX;
            }

            void
            setDistancesForCurBatchIf(const int &vehId, const DistanceLabel &distanceBatch,
                                      const LabelMask &batchInsertMask) {
                assert(vehId >= 0 && vehId < indexInDistanceVector.size());
                assert(distanceBatch.horizontalMin() >= 0 && distanceBatch.horizontalMin() < INFTY);

                if (!anySet(batchInsertMask))
                    return;

                // If no entries exist yet for this stop, perform the allocation.    
                if (indexInDistanceVector[vehId] == INVALID_INDEX) {
                    allocateLocalEntriesFor(vehId);
                }

                const auto &idx = indexInDistanceVector[vehId];

                distance[idx].setIf(distanceBatch, batchInsertMask);

            }

            DistanceLabel getDistancesForCurBatch(const int &vehId) {
                assert(vehId < indexInDistanceVector.size());
                const int idx = indexInDistanceVector[vehId];
                if (idx == INVALID_INDEX)
                    return DistanceLabel(INFTY);
                return distance[idx];
            }

        private:

            // Dynamic Allocation
            void allocateLocalEntriesFor(const int vehId) {
                assert(indexInDistanceVector[vehId] == INVALID_INDEX);

                indexInDistanceVector[vehId] = distance.size();
                distance.push_back(DistanceLabel(INFTY));
            }

            std::vector<int> &indexInDistanceVector;
            std::vector<DistanceLabel> &distance;

        };

        TentativeLastStopDistances(const size_t pFleetSize)
                : fleetSize(pFleetSize),
                  startIdxForVeh(fleetSize, INVALID_INDEX),
                  distances(),
                  stopLocks(fleetSize, SpinLock()),
                  threadLocalIndexInDistanceVector(fleetSize),
                  threadLocalDistances(),
                  writeResultsToGlobalVehicleOrder([&] {
                      return Permutation::getRandomPermutation(fleetSize, std::minstd_rand(
                              seedCounter.fetch_add(1, std::memory_order_relaxed)));
                  }) {}

        void init(const int &numBatches) {
            curNumBatches = numBatches;
//            startIdxForVeh.clear();
            std::fill(startIdxForVeh.begin(), startIdxForVeh.end(), INVALID_INDEX);
            distances.clear();
        }

        void setCurBatchIdx(const int &batchIdx) {
            curBatchIdx = batchIdx;
        }

        int getDistance(const int &vehId, const int &pdLocId) {
            assert(vehId < startIdxForVeh.size());
            const int startIdx = startIdxForVeh[vehId];
            if (startIdx == INVALID_INDEX)
                return INFTY;

            const int batchIdx = pdLocId / K;
            return distances[startIdx + batchIdx][pdLocId % K];
        }

        // Each thread gets an instance of a ThreadLocalTentativeLastStopDistances at the beginning of a search. This
        // object encapsulates the local result of the thread for that search. This way, the underlying TLS structures
        // are only queried once per search.
        ThreadLocalTentativeLastStopDistances getThreadLocalTentativeDistances() {
            return ThreadLocalTentativeLastStopDistances(threadLocalIndexInDistanceVector.local(),
                                                         threadLocalDistances.local());
        }

        void updateDistancesInGlobalVectors(const int firstPDLocId) {

            const auto &localIndices = threadLocalIndexInDistanceVector.local();
            const auto &localDistances = threadLocalDistances.local();

            for (const auto &vehId: writeResultsToGlobalVehicleOrder.local()) {
                const auto &idx = localIndices[vehId];
                if (idx != INVALID_INDEX) {
                    // Allocate the entries in global storage if not yet done
                    allocateEntriesFor(vehId);

                    const auto &dist = localDistances[idx];
                    const LabelMask improved =
                            dist < distances[startIdxForVeh[vehId] + firstPDLocId / K];

                    distances[startIdxForVeh[vehId] + firstPDLocId / K].setIf(dist, improved);

                }
            }
        }


    private:

        void allocateEntriesFor(const int vehId) {
            SpinLock &currLock = stopLocks[vehId];
            currLock.lock();
            if (startIdxForVeh[vehId] != INVALID_INDEX) {
                currLock.unlock();
                return;
            }

            const auto distancesIt = distances.grow_by(curNumBatches, DistanceLabel(INFTY));

            startIdxForVeh[vehId] = distancesIt - distances.begin();

            currLock.unlock();

        }

        int curNumBatches;
        const size_t fleetSize;

        // Index and value array for global distances
//        TimestampedVector<int> startIdxForVeh;
        std::vector<int> startIdxForVeh;
        tbb::concurrent_vector<DistanceLabel> distances; // curNumBatches DistanceLabels per vehicle

        // One spinlock per vehicle to synchronize dynamic allocation in global result
        std::vector<SpinLock> stopLocks;

        // Thread Local Storage for local distances calculation
        tbb::enumerable_thread_specific<std::vector<int>> threadLocalIndexInDistanceVector;
        tbb::enumerable_thread_specific<std::vector<DistanceLabel>> threadLocalDistances;

        int curBatchIdx;

        // Counter for generating differing seeds for random permutations between threads
        std::atomic_int seedCounter;

        // Each thread generates one random permutation of thread ids. The permutation defines the order in which
        // a threads local results are written to the global result. This helps to alleviate contention on the
        // spin locks (separate per stop id) used to synchronize global writes.
        tbb::enumerable_thread_specific<Permutation> writeResultsToGlobalVehicleOrder;

    };
}