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
#include "Algorithms/KaRRi/RequestState/RelevantPDLoc.h"

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


        using ConcurrentResultEntriesVector = tbb::concurrent_vector<RelevantPDLoc>;
        using ResultEntryIterator = typename ConcurrentResultEntriesVector::const_iterator;

    public:


        // Represents information that one thread computes during one elliptic BCH search. Can be incorporated into
        // global result at end of search.
        class ThreadLocalFeasibleEllipticDistances {

            friend FeasibleEllipticDistances;

            struct LocalResultEntry {

                explicit LocalResultEntry(const int stopId) : stopId(stopId) {}

                int stopId = INVALID_ID;

                DistanceLabel distFromStopToPdLoc = INFTY;
                DistanceLabel meetingVertexFromStopToPdLoc = INVALID_VERTEX;

                DistanceLabel distFromPdLocToNextStop = INFTY;
                DistanceLabel meetingVertexFromPdLocToNextStop = INVALID_VERTEX;
            };

        public:

            static constexpr int K = FeasibleEllipticDistances::K;

            ThreadLocalFeasibleEllipticDistances(const int &maxStopId,
                                                 std::vector<int> &indexInEntriesVector,
                                                 std::vector<LocalResultEntry> &entries)
                    : indexInEntriesVector(indexInEntriesVector),
                      entries(entries),
                      maxStopId(maxStopId) {}

            void initForSearch() {
                assert(entries.empty());
                assert(std::all_of(indexInEntriesVector.begin(), indexInEntriesVector.end(),
                                   [this](const int &i) { return i == indexInEntriesVector[0]; }));

                if (indexInEntriesVector.size() < maxStopId + 1)
                    indexInEntriesVector.resize(maxStopId + 1, INVALID_INDEX);
            }

            // Updates the distance from stop to the PD loc. Distance is written if there are
            // entries for the stop already or dynamic allocation of entries is allowed.
            // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
            // allocation is not allowed).
            LabelMask updateDistanceFromStopToPDLoc(const int stopId,
                                                    const DistanceLabel newDistToPDLoc,
                                                    const int meetingVertex) {
                assert(stopId >= 0 && stopId <= maxStopId);
                assert(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

                // If no entries exist yet for this stop, perform the allocation.
                if (indexInEntriesVector[stopId] == INVALID_INDEX) {
                    indexInEntriesVector[stopId] = entries.size();
                    entries.push_back(LocalResultEntry(stopId));
                }

                const auto &idx = indexInEntriesVector[stopId];
                auto &entry = entries[idx];

                const LabelMask improvedLocal = newDistToPDLoc < entry.distFromStopToPdLoc;
                entry.distFromStopToPdLoc.setIf(newDistToPDLoc, improvedLocal);
                entry.meetingVertexFromStopToPdLoc.setIf(meetingVertex, improvedLocal);

                return improvedLocal;
            }

            // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
            // for the stop exist already.
            // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
            LabelMask updateDistanceFromPDLocToNextStop(const int stopId,
                                                        const DistanceLabel newDistFromPdLocToNextStop,
                                                        const int meetingVertex) {

                // We assume the same thread runs the to search and then the from search for a PDLoc. If the to search
                // did not find a result for this stop, we do not need to consider it in the from search.
                if (indexInEntriesVector[stopId] == INVALID_INDEX)
                    return LabelMask(false);

                const auto &idx = indexInEntriesVector[stopId];
                auto &entry = entries[idx];

                const LabelMask improvedLocal = newDistFromPdLocToNextStop < entry.distFromPdLocToNextStop;
                entry.distFromPdLocToNextStop.setIf(newDistFromPdLocToNextStop, improvedLocal);
                entry.meetingVertexFromPdLocToNextStop.setIf(meetingVertex, improvedLocal);

                return improvedLocal;
            }

            std::vector<int> &indexInEntriesVector;
            std::vector<LocalResultEntry> &entries;

        private:
            const int &maxStopId;

        };

        explicit FeasibleEllipticDistances(const RouteState &routeState, const int fleetSize)
                : routeState(routeState),
                  maxStopId(routeState.getMaxStopId()),
                  vehiclesWithFeasibleDistances(fleetSize),
                  indexInEntriesVector(),
                  localResults(),
                  numRelPdLocsPerStop() {}

        void init() {
            globalResults.clear();
            vehiclesWithFeasibleDistances.clear();

            if (maxStopId >= numRelPdLocsPerStop.size()) {
                numRelPdLocsPerStop.resize(maxStopId + 1);
            }
            for (auto &n: numRelPdLocsPerStop)
                n.store(0);
        }

        // Each thread gets an instance of a ThreadLocalFeasibleEllipticDistances at the beginning of a search. This
        // object encapsulates the local result of the thread for that search. This way, the underlying TLS structures
        // are only queried once per search.
        ThreadLocalFeasibleEllipticDistances getThreadLocalFeasibleDistances() {
            return ThreadLocalFeasibleEllipticDistances(maxStopId, indexInEntriesVector.local(), localResults.local());
        }

        // Transfers the distances computed by a single thread for a batch of K PDLocs to the global result and clears
        // the local result in the process.
        void transferThreadLocalResultToGlobalResult(const int firstPdLocIdInBatch,
                                                     const int onePastLastPdLocIdInBatch,
                                                     ThreadLocalFeasibleEllipticDistances &localResult) {
            auto &localEntries = localResult.entries;
            auto &localIndices = localResult.indexInEntriesVector;

            // Reset local index vector
            for (const auto &e: localEntries)
                localIndices[e.stopId] = INVALID_INDEX;

            // Un-batch local entries to write them results for individual PDLocs into global results vector

            // Count total number of global entries (combinations of PD-loc in batch and stop relevant for that PD-loc)
            int numNewEntriesInGlobal = 0;
            for (auto &e: localEntries) {

                assert(allSet(((e.distFromStopToPdLoc == INFTY) & (e.distFromPdLocToNextStop == INFTY)) |
                              ((e.distFromStopToPdLoc < INFTY) & (e.distFromPdLocToNextStop < INFTY))));
                // If this is a partially filled last batch, we have to make sure not to count the entries that are not
                // used (copies of index 0 in batch) as they will not be stored in the global result.
                for (int i = onePastLastPdLocIdInBatch; i < firstPdLocIdInBatch + K; ++i)
                    e.distFromStopToPdLoc[i - firstPdLocIdInBatch] = INFTY;
                LabelMask rel = e.distFromStopToPdLoc < INFTY;

                const int numRel = countSet(rel);
                assert(numRel == [&] {
                    int manualCountRel = 0;
                    for (int pdId = firstPdLocIdInBatch; pdId < onePastLastPdLocIdInBatch; ++pdId)
                        manualCountRel += rel[pdId - firstPdLocIdInBatch];
                    return manualCountRel;
                }());

                numNewEntriesInGlobal += numRel;
                numRelPdLocsPerStop[e.stopId].add_fetch(numRel, std::memory_order_relaxed);
            }

            // Allocate space for global entries
            const auto startOfRangeForBatchIt = globalResults.grow_by(numNewEntriesInGlobal);
            const int startOfRangeForBatch = static_cast<int>(startOfRangeForBatchIt - globalResults.begin());

            // Write global entries
            int nextIdx = startOfRangeForBatch;
            for (const auto &e: localEntries) {
                const auto &stopId = e.stopId;
                const DistanceLabel &batchDistTo = e.distFromStopToPdLoc;
                const DistanceLabel &batchDistFrom = e.distFromPdLocToNextStop;
                for (int pdId = firstPdLocIdInBatch; pdId < onePastLastPdLocIdInBatch; ++pdId) {
                    int idxInBatch = pdId - firstPdLocIdInBatch;
                    if (batchDistTo[idxInBatch] >= INFTY)
                        continue;
                    assert(nextIdx < startOfRangeForBatch + numNewEntriesInGlobal);
                    globalResults[nextIdx] = {stopId, pdId, batchDistTo[idxInBatch], batchDistFrom[idxInBatch]};
                    ++nextIdx;
                    vehiclesWithFeasibleDistances.insert(routeState.vehicleIdOf(stopId));
                }
            }

            // Clear local entries
            localEntries.clear();
        }

        const ConcurrentResultEntriesVector &getGlobalResults() const {
            return globalResults;
        }

        const ThreadSafeSubset& getVehiclesWithFeasibleDistances() const {
            return vehiclesWithFeasibleDistances;
        }

        int getNumRelPdLocsForStop(const int stopId) const {
            return numRelPdLocsPerStop[stopId].load(std::memory_order_seq_cst);
        }

        bool doesStopHaveRelPdLocs(const int stopId) const {
            return getNumRelPdLocsForStop(stopId) > 0;
        }

    private:

        const RouteState &routeState;
        const int &maxStopId;

        ConcurrentResultEntriesVector globalResults;
        ThreadSafeSubset vehiclesWithFeasibleDistances;

        // Thread Local Storage for local distances calculation
        tbb::enumerable_thread_specific<std::vector<int>> indexInEntriesVector;
        tbb::enumerable_thread_specific<std::vector<typename ThreadLocalFeasibleEllipticDistances::LocalResultEntry>> localResults;

        std::vector<CAtomic<int>> numRelPdLocsPerStop;

    };

}