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

#include <type_traits>
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Tools/Simd/ConcurrentAlignedVector.h"
#include "Tools/Simd/AlignedVector.h"
#include "DataStructures/Containers/Subset.h"
#include "DataStructures/Containers/ThreadSafeSubset.h"

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Parallel/atomic_wrapper.h"
#include "DataStructures/Utilities/Permutation.h"

#include <atomic>
#include <thread>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {


    template<typename LabelSetT>
    class FeasibleEllipticDistances {


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        struct ResultEntry {
            int stopId = INVALID_ID;

            DistanceLabel distFromStopToPDLoc = INFTY;
            DistanceLabel meetingVertexFromStopToPDLoc = INVALID_VERTEX;

            DistanceLabel distFromPDLocToNextStop = INFTY;
            DistanceLabel meetingVertexFromPDLocToNextStop = INVALID_VERTEX;
        };
        using ConcurrentResultEntriesVector = ConcurrentAlignedVector<ResultEntry>;

    public:



        // Represents information that one thread computes during one elliptic BCH search. Can be incorporated into
        // global result at end of search.
        class ThreadLocalFeasibleEllipticDistances {

            friend FeasibleEllipticDistances;

        public:

            ThreadLocalFeasibleEllipticDistances(const int &maxStopId,
                                                 std::vector<int> &indexInPairVector,
                                                 std::vector<ResultEntry> &entries)
                    : maxStopId(maxStopId),
                      indexInPairVector(indexInPairVector),
                      entries(entries) {}

            void initForSearch() {

                if (indexInPairVector.size() < maxStopId + 1)
                    indexInPairVector.resize(maxStopId + 1, INVALID_INDEX);
                for (const auto &e: entries)
                    indexInPairVector[e.stopId] = INVALID_INDEX;
                entries.clear();
            }

            // Updates the distance from stop to the PD loc. Distance is written if there are
            // entries for the stop already or dynamic allocation of entries is allowed.
            // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
            // allocation is not allowed).
            LabelMask updateDistanceFromStopToPDLoc(const int stopId,
                                                    const DistanceLabel newDistToPDLoc, const int meetingVertex) {
                assert(stopId >= 0 && stopId <= maxStopId);
                assert(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

                // If no entries exist yet for this stop, perform the allocation.
                if (indexInPairVector[stopId] == INVALID_INDEX) {
                    allocateLocalEntriesFor(stopId);
                }

                const auto &idx = indexInPairVector[stopId];
                auto &entry = entries[idx];

                const LabelMask improvedLocal = newDistToPDLoc < entry.distFromStopToPDLoc;
                entry.distFromStopToPDLoc.setIf(newDistToPDLoc, improvedLocal);
                entry.meetingVertexFromStopToPDLoc.setIf(meetingVertex, improvedLocal);

                return improvedLocal;
            }

            // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
            // for the stop exist already.
            // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
            LabelMask updateDistanceFromPDLocToNextStop(const int stopId,
                                                        const DistanceLabel newDistFromPDLocToNextStop,
                                                        const int meetingVertex) {

                // We assume the same thread runs the to search and then the from search for a PDLoc. If the to search
                // did not find a result for this stop, we do not need to consider it in the from search.
                if (indexInPairVector[stopId] == INVALID_INDEX)
                    return LabelMask(false);

                const auto &idx = indexInPairVector[stopId];
                auto &entry = entries[idx];

                const LabelMask improvedLocal = newDistFromPDLocToNextStop < entry.distFromPDLocToNextStop;
                entry.distFromPDLocToNextStop.setIf(newDistFromPDLocToNextStop, improvedLocal);
                entry.meetingVertexFromPDLocToNextStop.setIf(meetingVertex, improvedLocal);

                return improvedLocal;
            }


        private:

            // Dynamic Allocation
            void allocateLocalEntriesFor(const int stopId) {
                assert(indexInPairVector[stopId] == INVALID_INDEX);

                indexInPairVector[stopId] = entries.size();
                entries.push_back({stopId});
            }

            const int &maxStopId;

            std::vector<int> &indexInPairVector;
            std::vector<ResultEntry> &entries;

        };

        explicit FeasibleEllipticDistances(const int fleetSize, const RouteState &routeState)
                : routeState(routeState),
                  maxStopId(routeState.getMaxStopId()),
                  startOfRange(fleetSize, INVALID_INDEX),
                  stopLocks(fleetSize, SpinLock()),
                  indexInPairVector(),
                  localResults(),
                  writeResultsToGlobalStopOrder([&] {
                      return Permutation::getRandomPermutation(maxStopId + 1, std::minstd_rand(
                              seedCounter.fetch_add(1, std::memory_order_relaxed)));
                  }),
                  vehiclesWithRelevantPDLocs(fleetSize),
                  minDistToPDLoc(fleetSize),
                  minDistFromPDLocToNextStop(fleetSize) {}

        template<typename PDLocsAtExistingStopsT, typename InputGraphT>
        void init(const int newNumPDLocs, const PDLocsAtExistingStopsT &pdLocsAtExistingStops,
                  const InputGraphT &inputGraph) {
            numLabelsPerStop = newNumPDLocs / K + (newNumPDLocs % K != 0);
            std::fill(startOfRange.begin(), startOfRange.end(), INVALID_INDEX);

            if (maxStopId >= startOfRange.size()) {
                stopLocks.resize(maxStopId + 1, SpinLock());
                startOfRange.resize(maxStopId + 1, INVALID_INDEX);
                minDistToPDLoc.resize(maxStopId + 1);
                minDistFromPDLocToNextStop.resize(maxStopId + 1);
            }

            vehiclesWithRelevantPDLocs.clear();
            globalResults.clear();

            for (auto& min : minDistToPDLoc)
                min.store(INFTY, std::memory_order_relaxed);
            for (auto& min : minDistFromPDLocToNextStop)
                min.store(INFTY, std::memory_order_relaxed);

            // Delete thread local random permutation of stop IDs (used for writing local results to global result)
            // since previous permutations may not contain .
            writeResultsToGlobalStopOrder.clear();


            // Pre-allocate entries for PD locs at existing stops. The distance 0 may otherwise not be found by the
            // BCH searches. Also, this way, the distance for such a PD loc never has to be updated, and we already
            // allocate the entry array for this stop, which is good since it will likely also be reachable by other PD locs.
            for (const auto &pdLocAtExistingStop: pdLocsAtExistingStops) {
                const auto &vehId = pdLocAtExistingStop.vehId;
                assert(pdLocAtExistingStop.stopIndex < routeState.numStopsOf(vehId));
                const auto &stopId = routeState.stopIdsFor(vehId)[pdLocAtExistingStop.stopIndex];
                const auto &stopVertex = inputGraph.edgeHead(
                        routeState.stopLocationsFor(vehId)[pdLocAtExistingStop.stopIndex]);
                // Write values for new entry and set pointer from PD loc to the entries directly into global storages
                allocateEntriesFor(stopId);

                DistanceLabel zeroLabel = INFTY;
                zeroLabel[pdLocAtExistingStop.pdId % K] = 0;
                const auto firstIdInBatch = (pdLocAtExistingStop.pdId / K) * K;
                const auto offsetInBatch = pdLocAtExistingStop.pdId % K;
                int idx = startOfRange[stopId] + firstIdInBatch / K;

                globalResults[idx].distFromStopToPDLoc[offsetInBatch] = 0;
                globalResults[idx].meetingVertexFromStopToPDLoc[offsetInBatch] = stopVertex;
                minDistToPDLoc[stopId].store(0, std::memory_order_relaxed);

                const auto lengthOfLegStartingHere = time_utils::calcLengthOfLegStartingAt(
                        pdLocAtExistingStop.stopIndex, pdLocAtExistingStop.vehId, routeState);
                globalResults[idx].distFromPDLocToNextStop[offsetInBatch] = lengthOfLegStartingHere;
                globalResults[idx].meetingVertexFromPDLocToNextStop[offsetInBatch] = stopVertex;
                minDistFromPDLocToNextStop[stopId].store(lengthOfLegStartingHere, std::memory_order_relaxed);
            }
        }

        // Each thread gets an instance of a ThreadLocalFeasibleEllipticDistances at the beginning of a search. This
        // object encapsulates the local result of the thread for that search. This way, the underlying TLS structures
        // are only queried once per search.
        ThreadLocalFeasibleEllipticDistances getThreadLocalFeasibleDistances() {
            return ThreadLocalFeasibleEllipticDistances(maxStopId, indexInPairVector.local(), localResults.local());
        }

        bool hasPotentiallyRelevantPDLocs(const int stopId) const {
            assert(stopId <= maxStopId);
            return startOfRange[stopId] != INVALID_INDEX;
        }

        // Writes the distances computed by a single thread for a batch of K PDLocs to the global result.
        void writeThreadLocalResultToGlobalResult(const int firstPDLocId,
                                                  const ThreadLocalFeasibleEllipticDistances &localResult) {
            const auto &localIndices = localResult.indexInPairVector;
            const auto &localEntries = localResult.entries;

            // Each search has similar search spaces so results for all threads contain similar stops with relevant
            // distances. To avoid lock contention when allocating distance labels in global result, we iterate
            // through stops in a random order that differs between threads.
            for (const auto &stopId: writeResultsToGlobalStopOrder.local()) {
                const auto &idx = localIndices[stopId];
                if (idx == INVALID_INDEX)
                    continue;

                // Allocate the entries in global storage if not yet done
                allocateEntriesFor(stopId);

                const auto &e = localEntries[idx];
                assert(e.stopId == stopId);

                updateToGlobalEntry(stopId, firstPDLocId, e);
                updateFromGlobalEntry(stopId, firstPDLocId, e);
            }
        }

        // Updates global entry for distances and meeting vertices from stop to batch of PDLocs with given local result.
        void updateToGlobalEntry(const int stopId, const int firstPDLocId, const ResultEntry& localEntry) {
            assert(startOfRange[stopId] + firstPDLocId / K < globalResults.size());

            const auto &localDist = localEntry.distFromStopToPDLoc;
            const auto &localMeetingVertex = localEntry.meetingVertexFromStopToPDLoc;

            auto& globalEntry = globalResults[startOfRange[stopId] + firstPDLocId / K];
            auto& globalDist = globalEntry.distFromStopToPDLoc;
            auto& globalMeetingVertex = globalEntry.meetingVertexFromStopToPDLoc;

            const LabelMask improved = localDist < globalDist;
            if (!anySet(improved))
                return;

            // Update distances and meeting vertices where necessary
            globalDist.setIf(localDist, improved);
            globalMeetingVertex.setIf(localMeetingVertex, improved);

            // Update minima
            const int minNewDistToPDLoc = localDist.horizontalMin();
            auto &minToPDLocAtomic = minDistToPDLoc[stopId];
            int expectedMinForStop = minToPDLocAtomic.load(std::memory_order_relaxed);
            while (expectedMinForStop > minNewDistToPDLoc &&
                   !minToPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistToPDLoc,
                                                             std::memory_order_relaxed));
        }

        // Updates global entry for distances and meeting vertices from batch of PDLocs to stop following the one with
        // ID stopId with given local result.
        void updateFromGlobalEntry(const int stopId, const int firstPDLocId, const ResultEntry& localEntry) {
            assert(startOfRange[stopId] + firstPDLocId / K < globalResults.size());

            const auto &localDist = localEntry.distFromPDLocToNextStop;
            const auto &localMeetingVertex = localEntry.meetingVertexFromPDLocToNextStop;

            auto& globalEntry = globalResults[startOfRange[stopId] + firstPDLocId / K];
            auto& globalDist = globalEntry.distFromPDLocToNextStop;
            auto& globalMeetingVertex = globalEntry.meetingVertexFromPDLocToNextStop;

            const LabelMask improved = localDist < globalDist;
            if (!anySet(improved))
                return;

            // Update distances and meeting vertices where necessary
            globalDist.setIf(localDist, improved);
            globalMeetingVertex.setIf(localMeetingVertex, improved);

            // Update minima
            const int minNewDistFromPDLocToNextStop = localDist.horizontalMin();
            auto &minFromPDLocAtomic = minDistFromPDLocToNextStop[stopId];
            int expectedMinForStop = minFromPDLocAtomic.load(std::memory_order_relaxed);
            while (expectedMinForStop > minNewDistFromPDLocToNextStop &&
                   !minFromPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistFromPDLocToNextStop,
                                                               std::memory_order_relaxed));
        }

        // Represents to and from distances for a single stop.
        // Allows random access to distances for a single PDLoc in the block given a PD loc id.
        // Used to hide intrinsics of DistanceLabels to caller.
        class PerPDLocDistancesFacade {

            using It = typename ConcurrentResultEntriesVector::const_iterator;

        public:
            // Returns distance from stop to PDLoc and distance from PDLoc to following stop.
            std::pair<int, int> operator[](const unsigned int pdLocId) const {
                const auto& e = labelBegin[pdLocId / K];
                return {e.distFromStopToPDLoc[pdLocId % K], e.distFromPDLocToNextStop[pdLocId % K]};
            }

        private:
            friend FeasibleEllipticDistances;
            explicit PerPDLocDistancesFacade(const It labelBegin) : labelBegin(labelBegin) {}
            const It labelBegin;
        };

        // Represents to and from meeting vertices for a single stop.
        // Allows random access to meeting vertices for a single PDLoc in the block given a PD loc id.
        // Used to hide intrinsics of DistanceLabels to caller.
        class PerPDLocMeetingVerticesFacade {

            using It = typename ConcurrentResultEntriesVector::const_iterator;

        public:
            // Returns distance from stop to PDLoc and distance from PDLoc to following stop.
            std::pair<int, int> operator[](const unsigned int pdLocId) const {
                const auto& e = labelBegin[pdLocId / K];
                return {e.meetingVertexFromStopToPDLoc[pdLocId % K], e.meetingVertexFromPDLocToNextStop[pdLocId % K]};
            }

        private:
            friend FeasibleEllipticDistances;
            explicit PerPDLocMeetingVerticesFacade(const It labelBegin) : labelBegin(labelBegin) {}
            const It labelBegin;
        };


        PerPDLocDistancesFacade distancesToAndFromRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRange[stopId] != INVALID_INDEX);
            const auto start = startOfRange[stopId];
            assert(globalResults.begin() + start + numLabelsPerStop <= globalResults.end());
            return PerPDLocDistancesFacade(globalResults.begin() + start);
        }

        PerPDLocMeetingVerticesFacade meetingVerticesToAndFromRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRange[stopId] != INVALID_INDEX);
            const auto start = startOfRange[stopId];
            assert(globalResults.begin() + start + numLabelsPerStop <= globalResults.end());
            return PerPDLocMeetingVerticesFacade(globalResults.begin() + start);
        }

        int minDistToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRange[stopId] != INVALID_INDEX);
            return minDistToPDLoc[stopId];
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRange[stopId] != INVALID_INDEX);
            return minDistFromPDLocToNextStop[stopId];
        }

        const ThreadSafeSubset &getVehiclesWithRelevantPDLocs() const {
            return vehiclesWithRelevantPDLocs;
        }

    private:

        void allocateEntriesFor(const int stopId) {
            SpinLock &currLock = stopLocks[stopId];
            currLock.lock();

            if (startOfRange[stopId] != INVALID_INDEX) {
                currLock.unlock();
                return;
            }

            const auto resultsIt = globalResults.grow_by(numLabelsPerStop, {stopId});
            startOfRange[stopId] = resultsIt - globalResults.begin();
            currLock.unlock();

            vehiclesWithRelevantPDLocs.insert(routeState.vehicleIdOf(stopId));

        }

        const RouteState &routeState;

        int numLabelsPerStop{};
        const int &maxStopId;

        // Points from a stop id to the start of the ResultEntries for PD locs that are relevant
        // for this stop. Not used in case of static allocation
        std::vector<int> startOfRange;

        // One spinlock per stop to synchronize dynamic allocation in global result
        std::vector<SpinLock> stopLocks;

        // Value arrays.
        ConcurrentResultEntriesVector globalResults;

        // Counter for generating differing seeds for random permutations between threads
        CAtomic<int> seedCounter;

        // Thread Local Storage for local distances calculation
        tbb::enumerable_thread_specific<std::vector<int>> indexInPairVector;
        tbb::enumerable_thread_specific<std::vector<ResultEntry>> localResults;

        // Each thread generates one random permutation of thread ids. The permutation defines the order in which
        // a threads local results are written to the global result. This helps to alleviate contention on the
        // spin locks (separate per stop id) used to synchronize global writes.
        tbb::enumerable_thread_specific<Permutation> writeResultsToGlobalStopOrder;

        ThreadSafeSubset vehiclesWithRelevantPDLocs;

        std::vector<CAtomic<int>> minDistToPDLoc;
        std::vector<CAtomic<int>> minDistFromPDLocToNextStop;

    };

}