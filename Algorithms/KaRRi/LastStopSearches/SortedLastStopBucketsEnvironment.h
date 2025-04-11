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
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"
#include "Algorithms/Buckets/CompactLastStopBucketContainer.h"
#include "Algorithms/KaRRi/Stats/LastStopBucketUpdateStats.h"
#include <tbb/enumerable_thread_specific.h>

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class SortedLastStopBucketsEnvironment {


        // .targetId is vehicle ID
        // .distToTarget is dist from last stop to vertex for idle vehicles or arrival time at vertex for non-idle vehicles
        using LastStopEntry = BucketEntry;

        struct CompareEntries {
            bool operator()(const LastStopEntry &e1, const LastStopEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED = true;

        using BucketContainer = CompactLastStopBucketContainer<LastStopEntry, CompareEntries, CompareEntries>;
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

        struct AddIdleEntryInsertion {
            explicit AddIdleEntryInsertion(const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curInsertions);
                KASSERT(curInsertions->size() > 0 && curInsertions->back() == EntryInsertion(),
                        "Invalid element to be populated at end of insertions is missing.");
                auto entry = LastStopEntry(curVehId, distToV[0]);
                bucketContainer.getIdleEntryInsertion(v, entry, curInsertions->back());
                curInsertions->emplace_back(); // Invalid last element to write to.
//                bucketContainer.insertIdle(v, entry);
                return false;
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            EntryInsertionVecT *curInsertions;
        };

        struct AddNonIdleEntryInsertion {
            explicit AddNonIdleEntryInsertion(const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID),
                      depTimeAtLastStopOfCurVeh(INFTY) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curInsertions);
                KASSERT(curInsertions->size() > 0 && curInsertions->back() == EntryInsertion(),
                        "Invalid element to be populated at end of insertions is missing.");
                auto entry = LastStopEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bucketContainer.getNonIdleEntryInsertion(v, entry, curInsertions->back());
                curInsertions->emplace_back(); // Invalid last element to write to.
//                bucketContainer.insertNonIdle(v, entry);
                return false;
            }

            const BucketContainer &bucketContainer;

            int curVehId;
            int depTimeAtLastStopOfCurVeh;
            EntryInsertionVecT *curInsertions;
        };

        struct AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicle {
            explicit AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicle(
                    const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID),
                      depTimeAtLastStopOfCurVeh(INFTY) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curNonIdleInsertions && curNonIdleDeletions);
                KASSERT(curNonIdleInsertions->size() > 0 && curNonIdleInsertions->back() == EntryInsertion(),
                        "Invalid element to be populated at end of insertions is missing.");
                KASSERT(curNonIdleDeletions->size() > 0 && curNonIdleDeletions->back() == EntryDeletion(),
                        "Invalid element to be populated at end of deletions is missing.");
                bool removed = bucketContainer.getNonIdleEntryDeletion(v, curVehId, curNonIdleDeletions->back());
//                bool removed = bucketContainer.removeNonIdle(v, curVehId);
                if (!removed)
                    return true; // Prune if no entry for the vehicle was found at this vertex
                curNonIdleDeletions->emplace_back(); // Invalid last element to write to next.

                auto newEntry = BucketEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bucketContainer.getNonIdleEntryInsertion(v, newEntry, curNonIdleInsertions->back());
                curNonIdleInsertions->emplace_back(); // Invalid last element to write to next.
//                bucketContainer.insertNonIdle(v, newEntry);
                return false;
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            int depTimeAtLastStopOfCurVeh;

            EntryDeletionVecT *curNonIdleDeletions;
            EntryInsertionVecT *curNonIdleInsertions;
        };

        struct AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdle {
            explicit AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID),
                      depTimeAtLastStopOfCurVeh(INFTY) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curIdleInsertions && curNonIdleDeletions);
                KASSERT(curIdleInsertions->size() > 0 && curIdleInsertions->back() == EntryInsertion(),
                        "Invalid element to be populated at end of insertions is missing.");
                KASSERT(curNonIdleDeletions->size() > 0 && curNonIdleDeletions->back() == EntryDeletion(),
                        "Invalid element to be populated at end of deletions is missing.");
                auto oldEntry = BucketEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bool removed = bucketContainer.getNonIdleEntryDeletion(v, oldEntry, curNonIdleDeletions->back());
//                bool removed = bucketContainer.removeNonIdle(v, oldEntry);
                if (!removed)
                    return true; // Prune if no entry for the vehicle was found at this vertex
                curNonIdleDeletions->emplace_back(); // Invalid last element to write to next.
                auto newEntry = BucketEntry(curVehId, distToV[0]);
                bucketContainer.getIdleEntryInsertion(v, newEntry, curIdleInsertions->back());
                curIdleInsertions->emplace_back(); // Invalid last element to write to next.
//                bucketContainer.insertIdle(v, newEntry);
                return false;
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            int depTimeAtLastStopOfCurVeh;

            EntryDeletionVecT *curNonIdleDeletions;
            EntryInsertionVecT *curIdleInsertions;
        };

        struct AddIdleEntryDeletion {
            explicit AddIdleEntryDeletion(const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curDeletions);
                KASSERT(curDeletions->size() > 0 && curDeletions->back() == EntryDeletion(),
                        "Invalid element to be populated at end of deletions is missing.");
                auto entry = BucketEntry(curVehId, distToV[0]);
                bool removed = bucketContainer.getIdleEntryDeletion(v, entry, curDeletions->back());
//                bool removed = bucketContainer.removeIdle(v, entry);
                if (removed)
                    curDeletions->emplace_back(); // Invalid last element to write to.
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            EntryDeletionVecT *curDeletions;
        };

        struct AddNonIdleEntryDeletion {
            explicit AddNonIdleEntryDeletion(const BucketContainer &bucketContainer)
                    : bucketContainer(bucketContainer), curVehId(INVALID_ID) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                KASSERT(curVehId != INVALID_ID && curDeletions);
                KASSERT(curDeletions->size() > 0 && curDeletions->back() == EntryDeletion(),
                        "Invalid element to be populated at end of deletions is missing.");
                // todo: it would be possible to use binary search here if we had the old dep time
                bool removed = bucketContainer.getNonIdleEntryDeletion(v, curVehId, curDeletions->back());
//                bool removed = bucketContainer.removeNonIdle(v, curVehId);
                if (removed)
                    curDeletions->emplace_back(); // Invalid last element to write to.
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            const BucketContainer &bucketContainer;
            int curVehId;
            EntryDeletionVecT *curDeletions;
        };

    public:

        SortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                         const RouteState &routeState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.upwardGraph()),
                  routeState(routeState),
                  bucketContainer(searchGraph.numVertices()),
                  addIdleEntryInsertionsSearch(chEnv.getForwardSearch(AddIdleEntryInsertion(bucketContainer))),
                  addNonIdleEntryInsertionsSearch(
                          [&]() { return chEnv.getForwardSearch(AddNonIdleEntryInsertion(bucketContainer)); }),
                  addEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicleSearch(
                          [&]() {
                              return chEnv.getForwardSearch(
                                      AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicle(bucketContainer));
                          }),
                  addEntryInsertionsAndDeletionsForVehicleHasBecomeIdleSearch(
                          [&]() {
                              return chEnv.getForwardSearch(
                                      AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(bucketContainer));
                          }),
                  addIdleEntryDeletionsSearch(
                          [&]() { return chEnv.getForwardSearch(AddIdleEntryDeletion(bucketContainer)); }),
                  addNonIdleEntryDeletionsSearch(
                          [&]() { return chEnv.getForwardSearch(AddNonIdleEntryDeletion(bucketContainer)); }) {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }


        void addIdleBucketEntryInsertions(const int vehId) {
            const auto &numStops = routeState.numStopsOf(vehId);

            const auto stopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &idleEntryInsertions = idleInsertions.local();
            idleEntryInsertions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID and the current insertions vector for the search in the callback operator.
            auto &addInsertionCallback = std::get<1>(addIdleEntryInsertionsSearch.getPruningCriterion().criterions);
            addInsertionCallback.curVehId = vehId;
            addInsertionCallback.curInsertions = &idleEntryInsertions;

            // Run the search to add idle bucket entry insertions.
            addIdleEntryInsertionsSearch.run(ch.rank(stopVertex));

            idleEntryInsertions.pop_back(); // Remove unused invalid last element
        }

        void addNonIdleBucketEntryInsertions(const int vehId) {
            const auto &numStops = routeState.numStopsOf(vehId);

            const auto stopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &search = addNonIdleEntryInsertionsSearch.local();
            auto &nonIdleEntryInsertions = nonIdleInsertions.local();
            nonIdleEntryInsertions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID, the departure time at the vehicle's last stop and the current insertions
            // vector for the search in the callback operator.
            auto &addInsertionCallback = std::get<1>(search.getPruningCriterion().criterions);
            addInsertionCallback.curVehId = vehId;
            addInsertionCallback.depTimeAtLastStopOfCurVeh = routeState.schedDepTimesFor(vehId)[numStops - 1];
            addInsertionCallback.curInsertions = &nonIdleEntryInsertions;

            // Run the search to add non idle bucket entry insertions.
            search.run(ch.rank(stopVertex));

            nonIdleEntryInsertions.pop_back(); // Remove unused invalid last element
        }

        void addIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            KASSERT(stopIndex >= 0);
            KASSERT(stopIndex < routeState.numStopsOf(vehId));

            const auto stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &search = addIdleEntryDeletionsSearch.local();
            auto &idleEntryDeletions = idleDeletions.local();
            idleEntryDeletions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID the current deletions vector for the search in the callback operator.
            auto &addDeletionCallback = std::get<1>(search.getPruningCriterion().criterions);
            addDeletionCallback.curVehId = vehId;
            addDeletionCallback.curDeletions = &idleEntryDeletions;

            search.run(ch.rank(stopVertex));

            idleEntryDeletions.pop_back(); // Remove unused invalid last element
        }

        void addNonIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            KASSERT(stopIndex >= 0);
            KASSERT(stopIndex < routeState.numStopsOf(vehId));

            const auto stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            auto &search = addNonIdleEntryDeletionsSearch.local();
            auto &nonIdleEntryDeletions = nonIdleDeletions.local();

            nonIdleEntryDeletions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID the current deletions vector for the search in the callback operator.
            auto &addDeletionCallback = std::get<1>(search.getPruningCriterion().criterions);
            addDeletionCallback.curVehId = vehId;
            addDeletionCallback.curDeletions = &nonIdleEntryDeletions;

            search.run(ch.rank(stopVertex));

            nonIdleEntryDeletions.pop_back(); // Remove unused invalid last element
        }

        // Update the bucket entries for a non-idle vehicle whose last stop has remained the same but the departure
        // time at that stop has changed.
        void addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(const int vehId, const int stopIndex) {

            const auto stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            const auto &numStops = routeState.numStopsOf(vehId);
            KASSERT(numStops > 1);
            auto &search = addEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicleSearch.local();
            auto &nonIdleEntryInsertions = nonIdleInsertions.local();
            auto &nonIdleEntryDeletions = nonIdleDeletions.local();
            nonIdleEntryInsertions.emplace_back(); // Invalid last element to write to.
            nonIdleEntryDeletions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID the current deletions vector for the search in the callback operator.
            auto &updateCallback = std::get<1>(search.getPruningCriterion().criterions);
            updateCallback.curVehId = vehId;
            updateCallback.depTimeAtLastStopOfCurVeh = routeState.schedDepTimesFor(vehId)[numStops - 1];
            updateCallback.curNonIdleInsertions = &nonIdleEntryInsertions;
            updateCallback.curNonIdleDeletions = &nonIdleEntryDeletions;

            search.run(ch.rank(stopVertex));

            nonIdleEntryInsertions.pop_back(); // Remove unused invalid last element
            nonIdleEntryDeletions.pop_back(); // Remove unused invalid last element
        }

        // Update bucket entries for vehicle that has become idle.
        void addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(const int vehId) {

            const auto stopLoc = routeState.stopLocationsFor(vehId)[0];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);
            KASSERT(routeState.numStopsOf(vehId) == 1);

            auto &search = addEntryInsertionsAndDeletionsForVehicleHasBecomeIdleSearch.local();
            auto &idleEntryInsertions = idleInsertions.local();
            auto &nonIdleEntryDeletions = nonIdleDeletions.local();
            idleEntryInsertions.emplace_back(); // Invalid last element to write to.
            nonIdleEntryDeletions.emplace_back(); // Invalid last element to write to.

            // Set the current vehicle ID the current deletions vector for the search in the callback operator.
            auto &updateCallback = std::get<1>(search.getPruningCriterion().criterions);
            updateCallback.curVehId = vehId;
            updateCallback.depTimeAtLastStopOfCurVeh = routeState.schedDepTimesFor(vehId)[0];
            updateCallback.curIdleInsertions = &idleEntryInsertions;
            updateCallback.curNonIdleDeletions = &nonIdleEntryDeletions;

            search.run(ch.rank(stopVertex));

            idleEntryInsertions.pop_back(); // Remove unused invalid last element
            nonIdleEntryDeletions.pop_back(); // Remove unused invalid last element
        }

        int numPendingEntryInsertions() const {

            int sum = 0;
            for (const auto &local: idleInsertions)
                sum += local.size();
            for (const auto &local: nonIdleInsertions)
                sum += local.size();

            return sum;
        }

        int numPendingEntryDeletions() const {
            int sum = 0;
            for (const auto &local: idleDeletions)
                sum += local.size();
            for (const auto &local: nonIdleDeletions)
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
            size_t numIdleInsertions = 0;
            size_t numNonIdleInsertions = 0;
            for (const auto &local: idleInsertions) {
                numIdleInsertions += local.size();
            }
            for (const auto &local: nonIdleInsertions) {
                numNonIdleInsertions += local.size();
            }
            globalInsertions.reserve(numIdleInsertions + numNonIdleInsertions);
            for (auto &local: idleInsertions) {
                globalInsertions.insert(globalInsertions.end(), local.begin(), local.end());
                local.clear();
            }
            for (auto &local: nonIdleInsertions) {
                globalInsertions.insert(globalInsertions.end(), local.begin(), local.end());
                local.clear();
            }
            stats.accumulateThreadLocalInsertionsTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            EntryDeletionVecT globalDeletions;
            size_t numIdleDeletions = 0;
            size_t numNonIdleDeletions = 0;
            for (const auto &local: idleDeletions) {
                numIdleDeletions += local.size();
            }
            for (const auto &local: nonIdleDeletions) {
                numNonIdleDeletions += local.size();
            }
            globalDeletions.reserve(numIdleDeletions + numNonIdleDeletions);
            for (auto &local: idleDeletions) {
                globalDeletions.insert(globalDeletions.end(), local.begin(), local.end());
                local.clear();
            }
            for (auto &local: nonIdleDeletions) {
                globalDeletions.insert(globalDeletions.end(), local.begin(), local.end());
                local.clear();
            }
            stats.accumulateThreadLocalDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();

            bucketContainer.batchedCommitInsertionsAndDeletions(globalInsertions, numIdleInsertions,
                                                                globalDeletions, numIdleDeletions, stats);
            KASSERT(verifyIdleAndNonIdleBorders());
        }

        bool verifyIdleAndNonIdleBorders() const {
            FORALL_VERTICES(inputGraph, v) {
                const auto rank = ch.rank(v);
                for (const auto& e : bucketContainer.getIdleBucketOf(rank)) {
                    KASSERT(routeState.numStopsOf(e.targetId) == 1);
                    if (routeState.numStopsOf(e.targetId) != 1)
                        return false;
                }
                for (const auto& e : bucketContainer.getNonIdleBucketOf(rank)) {
                    KASSERT(routeState.numStopsOf(e.targetId) > 1);
                    if (routeState.numStopsOf(e.targetId) <= 1)
                        return false;
                }
            }
            return true;
        }

        // Implement the last stops at vertices interface.
        std::vector<int> vehiclesWithLastStopAt(const int vertex) const {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            const auto rank = ch.rank(vertex);
            std::vector<int> vehIds;

            for (const auto &entry: bucketContainer.getIdleBucketOf(rank)) {
                if (entry.distToTarget == 0)
                    vehIds.push_back(entry.targetId);
            }
            for (const auto &entry: bucketContainer.getNonIdleBucketOf(rank)) {
                if (entry.distToTarget ==
                    routeState.schedDepTimesFor(entry.targetId)[routeState.numStopsOf(entry.targetId) - 1])
                    vehIds.push_back(entry.targetId);
            }

            return vehIds;
        }

    private:

        using AddIdleEntryInsertionsSearch = typename CHEnvT::template UpwardSearch<AddIdleEntryInsertion>;
        using AddNonIdleEntryInsertionsSearch = typename CHEnvT::template UpwardSearch<AddNonIdleEntryInsertion>;
        using AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdleSearch = typename CHEnvT::template UpwardSearch<AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdle>;
        using AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicleSearch = typename CHEnvT::template UpwardSearch<AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicle>;
        using AddIdleEntryDeletionsSearch = typename CHEnvT::template UpwardSearch<AddIdleEntryDeletion>;
        using AddNonIdleEntryDeletionsSearch = typename CHEnvT::template UpwardSearch<AddNonIdleEntryDeletion>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteState &routeState;

        BucketContainer bucketContainer;

//        int vehicleId;
//        int depTimeOfVehAtLastStop;
//        int maxDetourUntilEndOfServiceTime; TODO could add this back in as stopping criterion for Add searches
//        int entriesVisitedInSearch;
//        int verticesVisitedInSearch;

        // Search for only generating idle entries is only ever run sequentially upon vehicle startup.
        AddIdleEntryInsertionsSearch addIdleEntryInsertionsSearch;

        tbb::enumerable_thread_specific<AddNonIdleEntryInsertionsSearch> addNonIdleEntryInsertionsSearch;
        tbb::enumerable_thread_specific<AddEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicleSearch> addEntryInsertionsAndDeletionsForNewScheduleOfNonIdleVehicleSearch;
        tbb::enumerable_thread_specific<AddEntryInsertionsAndDeletionsForVehicleHasBecomeIdleSearch> addEntryInsertionsAndDeletionsForVehicleHasBecomeIdleSearch;
        tbb::enumerable_thread_specific<AddIdleEntryDeletionsSearch> addIdleEntryDeletionsSearch;
        tbb::enumerable_thread_specific<AddNonIdleEntryDeletionsSearch> addNonIdleEntryDeletionsSearch;


        tbb::enumerable_thread_specific<EntryInsertionVecT> idleInsertions;
        tbb::enumerable_thread_specific<EntryInsertionVecT> nonIdleInsertions;
        tbb::enumerable_thread_specific<EntryDeletionVecT> idleDeletions;
        tbb::enumerable_thread_specific<EntryDeletionVecT> nonIdleDeletions;

    };
}