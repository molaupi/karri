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
#include <oneapi/tbb/enumerable_thread_specific.h>

#include "LastStopBucketEntry.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"
#include "Algorithms/Buckets/CompactLastStopBucketContainer.h"
#include "Algorithms/Buckets/LastStopBucketContainer.h"
#include "Algorithms/KaRRi/Stats/LastStopBucketUpdateStats.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class BatchUpdatesUnsortedLastStopBucketsEnvironment {

    public:

        // using BucketContainer = CompactLastStopBucketContainer<LastStopEntry, CompareEntries, CompareEntries>;
        using BucketContainer = DynamicBucketContainer<LastStopBucketEntry>;
        using EntryInsertion = typename BucketContainer::EntryInsertion;
        using EntryDeletion = typename BucketContainer::EntryDeletion;
        using EntryInsertionVecT = parallel::scalable_vector<EntryInsertion>;
        using EntryDeletionVecT = parallel::scalable_vector<EntryDeletion>;

    private:

        struct StopWhenDistanceExceeded {
            explicit StopWhenDistanceExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return distToV[0] > maxDist;
            }

        private:
            const int &maxDist;
        };

        struct AddEntryInsertion {
            explicit AddEntryInsertion(const BucketContainer &bucketContainer)
                : bucketContainer(bucketContainer), curVehId(INVALID_ID), curInsertions(nullptr) {
            }

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curInsertions);
                KASSERT(curInsertions->size() > 0 && curInsertions->back() == EntryInsertion(),
                        "Invalid element to be populated at end of insertions is missing.");
                auto entry = LastStopBucketEntry(curVehId, distToV[0]);
                bucketContainer.getEntryInsertion(v, entry, curInsertions->back());
                curInsertions->emplace_back(); // Invalid last element to write to.
//                bucketContainer.insertIdle(v, entry);
                return false;
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            EntryInsertionVecT *curInsertions;
        };

        struct AddEntryDeletion {
            explicit AddEntryDeletion(const BucketContainer &bucketContainer)
                : bucketContainer(bucketContainer), curVehId(INVALID_ID), curDeletions(nullptr) {
            }

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curDeletions);
                KASSERT(curDeletions->size() > 0 && curDeletions->back() == EntryDeletion(),
                        "Invalid element to be populated at end of deletions is missing.");
                int64_t dummy = 0;
                bool removed = bucketContainer.getEntryDeletion(v, curVehId, curDeletions->back(), dummy);
                if (removed)
                    curDeletions->emplace_back(); // Invalid last element to write to.
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            EntryDeletionVecT *curDeletions;
        };

    public:

        BatchUpdatesUnsortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                                     const RouteState &routeState,
                                                     BucketContainer &bucketContainer)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.upwardGraph()),
                  routeState(routeState),
                  bucketContainer(bucketContainer),
                  addEntryInsertionsSearch(chEnv.getForwardSearch(AddEntryInsertion(bucketContainer))),
                  addEntryDeletionsSearch(
                          [&]() { return chEnv.getForwardSearch(AddEntryDeletion(bucketContainer)); }) {}


        void addIdleBucketEntryInsertions(const int vehId) {
            // No distinction between idle and non-idle
            addEntryInsertions(vehId);
        }

        void addNonIdleBucketEntryInsertions(const int vehId) {
            // No distinction between idle and non-idle
            addEntryInsertions(vehId);
        }

        void addIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            // No distinction between idle and non-idle
            addEntryDeletions(vehId, stopIndex);
        }

        void addNonIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            // No distinction between idle and non-idle
            addEntryDeletions(vehId, stopIndex);
        }

        void addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(const int, const int) {
            // No op since bucket updates are only needed if buckets are sorted
        }

        void addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(const int) {
            // No op since bucket updates are only needed if buckets are sorted
        }

        int numPendingEntryInsertions() const {

            int sum = 0;
            for (const auto &local: insertions)
                sum += local.size();

            return sum;
        }

        int numPendingEntryDeletions() const {
            int sum = 0;
            for (const auto &local: deletions)
                sum += local.size();

            return sum;
        }

        bool noPendingEntryInsertionsOrDeletions() const {
            return numPendingEntryInsertions() == 0 && numPendingEntryDeletions() == 0;
        }

        void commitEntryInsertionsAndDeletions(LastStopBucketUpdateStats& stats) {
            Timer timer;
            // Accumulate insertions and deletions from thread local vectors. Clear local vectors.
            EntryInsertionVecT globalInsertions;
            globalInsertions.reserve(numPendingEntryInsertions());
            for (auto &local: insertions) {
                globalInsertions.insert(globalInsertions.end(), local.begin(), local.end());
                local.clear();
            }
            stats.accumulateThreadLocalInsertionsTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            EntryDeletionVecT globalDeletions;
            globalDeletions.reserve(numPendingEntryDeletions());
            for (auto &local: deletions) {
                globalDeletions.insert(globalDeletions.end(), local.begin(), local.end());
                local.clear();
            }
            stats.accumulateThreadLocalDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();

            bucketContainer.batchedCommitInsertionsAndDeletions(globalInsertions, globalDeletions);
        }

        bool verifyIdleAndNonIdleBorders() const {
            return true;
        }

        // Implement the last stops at vertices interface.
        std::vector<int> vehiclesWithLastStopAt(const int vertex) const {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            const auto rank = ch.rank(vertex);
            std::vector<int> vehIds;

            for (const auto &entry: bucketContainer.getBucketOf(rank)) {
                if (entry.distToTarget == 0)
                    vehIds.push_back(entry.targetId);
            }

            return vehIds;
        }

    private:

        void addEntryInsertions(const int vehId) {
            const auto &numStops = routeState.numStopsOf(vehId);

            const auto stopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &search = addEntryInsertionsSearch.local();
            auto &entryInsertions = insertions.local();
            entryInsertions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID and the current insertions vector for the search in the callback operator.
            auto &addInsertionCallback = std::get<1>(search.getPruningCriterion().criterions);
            addInsertionCallback.curVehId = vehId;
            addInsertionCallback.curInsertions = &entryInsertions;

            // Run the search to add idle bucket entry insertions.
            search.run(ch.rank(stopVertex));

            entryInsertions.pop_back(); // Remove unused invalid last element
        }

        void addEntryDeletions(const int vehId, const int stopIndex) {
            KASSERT(stopIndex >= 0);
            KASSERT(stopIndex < routeState.numStopsOf(vehId));

            const auto stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &search = addEntryDeletionsSearch.local();
            auto &entryDeletions = deletions.local();
            entryDeletions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID the current deletions vector for the search in the callback operator.
            auto &addDeletionCallback = std::get<1>(search.getPruningCriterion().criterions);
            addDeletionCallback.curVehId = vehId;
            addDeletionCallback.curDeletions = &entryDeletions;

            search.run(ch.rank(stopVertex));

            entryDeletions.pop_back(); // Remove unused invalid last element
        }

        using AddEntryInsertionsSearch = typename CHEnvT::template UpwardSearch<AddEntryInsertion>;
        using AddEntryDeletionsSearch = typename CHEnvT::template UpwardSearch<AddEntryDeletion>;

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteState &routeState;

        BucketContainer &bucketContainer;

        tbb::enumerable_thread_specific<AddEntryInsertionsSearch> addEntryInsertionsSearch;
        tbb::enumerable_thread_specific<AddEntryDeletionsSearch> addEntryDeletionsSearch;

        tbb::enumerable_thread_specific<EntryInsertionVecT> insertions;
        tbb::enumerable_thread_specific<EntryDeletionVecT> deletions;

    };
}