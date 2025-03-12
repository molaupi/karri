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
#include <tbb/enumerable_thread_specific.h>
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/CompactSortedBucketContainer.h"
#include "BucketEntryWithLeeway.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Parallel/scalable_vector.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS>
    class EllipticBucketsEnvironment {

        using Entry = BucketEntryWithLeeway;

        struct DoesEntryHaveLargerRemainingLeeway {
            bool operator()(const Entry &e1, const Entry &e2) const {
                return e1.leeway - e1.distToTarget > e2.leeway - e2.distToTarget;
            }
        };


        struct StopWhenLeewayExceeded {
            explicit StopWhenLeewayExceeded() : currentLeeway(-1) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, const DistLabelT &distToV, const DistLabelContT &) const noexcept {
                KASSERT(currentLeeway >= 0 && currentLeeway < INFTY);
                return allSet(distToV > currentLeeway);
            }

            void setCurrentLeeway(const int newLeeway) {
                currentLeeway = newLeeway;
            }

            int currentLeeway;
        };

        struct StoreSearchSpace {
            explicit StoreSearchSpace() : searchSpace(nullptr) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                KASSERT(searchSpace);
                searchSpace->push_back(v);
                return false;
            }

            void setCurSearchSpace(parallel::scalable_vector<int>* newSearchSpace) {
                KASSERT(newSearchSpace);
                searchSpace = newSearchSpace;
            }

            parallel::scalable_vector<int> *searchSpace;
        };

    public:


        static constexpr bool SORTED_BY_REM_LEEWAY = SORTED_BUCKETS;

        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                CompactSortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
//                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;

        using EntryInsertion = typename BucketContainer::EntryInsertion;
        using EntryDeletion = typename BucketContainer::EntryDeletion;

        EllipticBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnvIn, const RouteState &routeState)
                : inputGraph(inputGraph), ch(chEnvIn.getCH()), routeState(routeState), chEnv(chEnvIn),
                  sourceBuckets(inputGraph.numVertices()), targetBuckets(inputGraph.numVertices()),
                  descendentHasEntry([&](){return BitVector(inputGraph.numVertices());}),
                  forwardSearchFromNewStop([&]() {return chEnv.getForwardTopologicalSearch(StoreSearchSpace(), StopWhenLeewayExceeded());}),
                  reverseSearchFromNewStop([&]() {return chEnv.getReverseTopologicalSearch(StoreSearchSpace(), StopWhenLeewayExceeded());}),
                  forwardSearchFromPrevStop([&](){return chEnv.getForwardSearch({}, StopWhenLeewayExceeded());}),
                  reverseSearchFromNextStop([&](){return chEnv.getReverseSearch({}, StopWhenLeewayExceeded());}),
                  deleteSearchSpace(inputGraph.numVertices()) {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBuckets;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBuckets;
        }

        size_t totalNumSourceEntries() const {
            return sourceBuckets.totalNumEntries();
        }

        size_t totalNumTargetEntries() const {
            return targetBuckets.totalNumEntries();
        }

        bool noPendingEntryInsertionsOrDeletions() const {
            return numPendingEntryInsertions() == 0 && numPendingEntryDeletions() == 0;
        }

        int numPendingEntryInsertions() const {

            int sum = 0;
            for (const auto& localInsertions : sourceInsertions)
                sum += localInsertions.size();
            for (const auto& localInsertions : targetInsertions)
                sum += localInsertions.size();

            return sum;
        }

        int numPendingEntryDeletions() const {
            return sourceDeletions.size() + targetDeletions.size();
        }

        // Adds entry insertions to source buckets for the given stop index of the given vehicle.
        // Memorizes insertions to be performed in next batch update of the buckets.
        // Safe to call in a parallel environment, insertions are stored in thread local storage.
        void addSourceBucketEntryInsertions(const int vehId, const int stopIndex
                                            //, karri::stats::UpdatePerformanceStats &stats
                                            ) {
            assert(routeState.numStopsOf(vehId) > stopIndex + 1);

            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(vehId)[stopIndex + 1],
                                        routeState.schedDepTimesFor(vehId)[stopIndex + 1]) -
                               routeState.schedDepTimesFor(vehId)[stopIndex] - InputConfig::getInstance().stopTime;

            if (leeway <= 0)
                return;

            const int newStopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));

            const int nextStopLoc = routeState.stopLocationsFor(vehId)[stopIndex + 1];
            const int nextStopRoot = ch.rank(inputGraph.edgeTail(nextStopLoc));
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);

            generateBucketEntries(stopId, leeway,
                                  newStopRoot, 0, forwardSearchFromNewStop.local(), ch.upwardGraph(),
                                  nextStopRoot, nextStopOffset, reverseSearchFromNextStop.local(), ch.downwardGraph(),
                                  sourceBuckets, sourceInsertions.local());
        }

        // Adds entry insertions to target buckets for the given stop index of the given vehicle.
        // Memorizes insertions to be performed in next batch update of the buckets.
        // Safe to call in a parallel environment, insertions are stored in thread local storage.
        void addTargetBucketEntryInsertions(const int vehId, const int stopIndex
//                                            , karri::stats::UpdatePerformanceStats &stats
                                            ) {
            assert(stopIndex > 0);

            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(vehId)[stopIndex],
                                        routeState.schedDepTimesFor(vehId)[stopIndex]) -
                               routeState.schedDepTimesFor(vehId)[stopIndex - 1] - InputConfig::getInstance().stopTime;
            if (leeway <= 0)
                return;

            const int newStopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeTail(newStopLoc));
            const int newStopOffset = inputGraph.travelTime(newStopLoc);

            const int prevStopLoc = routeState.stopLocationsFor(vehId)[stopIndex - 1];
            const int prevStopRoot = ch.rank(inputGraph.edgeHead(prevStopLoc));

            generateBucketEntries(stopId, leeway,
                                  newStopRoot, newStopOffset, reverseSearchFromNewStop.local(), ch.downwardGraph(),
                                  prevStopRoot, 0, forwardSearchFromPrevStop.local(), ch.upwardGraph(),
                                  targetBuckets, targetInsertions.local());
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const int vehId, karri::stats::UpdatePerformanceStats &stats) {
            const auto numStops = routeState.numStopsOf(vehId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateSourceLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeState.vehicleIdOf(e.targetId) != vehId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeState.leewayOfLegStartingAt(e.targetId);
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            for (int idx = 0; idx < numStops - 1; ++idx) {
                const int root = ch.rank(inputGraph.edgeHead(stopLocations[idx]));
                deleteSearchSpace.insert(root);
            }
            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
                const auto v = *iter;
                if (sourceBuckets.updateAllEntries(v, updateSourceLeeway, numEntriesScanned)) {
                    FORALL_INCIDENT_EDGES(ch.upwardGraph(), v, e) {
                        const auto w = ch.upwardGraph().edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_update_time += time;
            stats.elliptic_update_numVerticesVisited += numVerticesVisited;
            stats.elliptic_update_numEntriesScanned += numEntriesScanned;
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const int vehId, karri::stats::UpdatePerformanceStats &stats) {
            const auto numStops = routeState.numStopsOf(vehId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateTargetLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeState.vehicleIdOf(e.targetId) != vehId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeState.leewayOfLegStartingAt(routeState.idOfPreviousStopOf(e.targetId));
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            for (int idx = 1; idx < numStops; ++idx) {
                const int root = ch.rank(inputGraph.edgeTail(stopLocations[idx]));
                deleteSearchSpace.insert(root);
            }
            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
                const auto v = *iter;
                if (targetBuckets.updateAllEntries(v, updateTargetLeeway, numEntriesScanned)) {
                    FORALL_INCIDENT_EDGES(ch.downwardGraph(), v, e) {
                        const auto w = ch.downwardGraph().edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_update_time += time;
            stats.elliptic_update_numVerticesVisited += numVerticesVisited;
            stats.elliptic_update_numEntriesScanned += numEntriesScanned;
        }


        void addSourceBucketEntryDeletions(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeHead(stopLoc));
            deleteBucketEntries(stopId, root, ch.upwardGraph(), sourceBuckets, sourceDeletions);
        }

        void addTargetBucketEntryDeletions(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeTail(stopLoc));
            deleteBucketEntries(stopId, root, ch.downwardGraph(), targetBuckets, targetDeletions);
        }

        void commitEntryInsertions() {
            commitSourceBucketEntryInsertions();
            commitTargetBucketEntryInsertions();
        }

        void commitSourceBucketEntryInsertions() {

            // Accumulate insertions from thread local insertions. Clear local insertions.
            std::vector<EntryInsertion> global;
            for (auto& local : sourceInsertions) {
                global.insert(global.end(), local.begin(), local.end());
                local.clear();
            }

            sourceBuckets.batchedCommitInsertions(global);
        }

        void commitTargetBucketEntryInsertions() {

            // Accumulate insertions from thread local insertions. Clear local insertions.
            std::vector<EntryInsertion> global;
            for (auto& local : targetInsertions) {
                global.insert(global.end(), local.begin(), local.end());
                local.clear();
            }

            targetBuckets.batchedCommitInsertions(global);
        }

        void commitEntryDeletions() {
            commitSourceBucketEntryDeletions();
            commitTargetBucketEntryDeletions();
        }

        void commitSourceBucketEntryDeletions() {
            sourceBuckets.batchedCommitDeletions(sourceDeletions);
            sourceDeletions.clear();
        }

        void commitTargetBucketEntryDeletions() {
            targetBuckets.batchedCommitDeletions(targetDeletions);
            targetDeletions.clear();
        }

    private:


        // Searches for inserting new stops: Topo search from/to new stop, regular CH searches to/from neighboring stops.
        using SearchFromNewStop = typename CHEnvT::template TopologicalUpwardSearch<
                StoreSearchSpace, StopWhenLeewayExceeded>;
        using SearchFromNeighbor = typename CHEnvT::template UpwardSearch<
                dij::NoCriterion, StopWhenLeewayExceeded>;

        template<typename EntryInsertionsVecT>
        void generateBucketEntries(const int stopId, const int leeway,
                                   const int newStopRoot, const int newStopOffSet, SearchFromNewStop &searchFromNewStop,
                                   const CH::SearchGraph &newStopGraph,
                                   const int neighborRoot, const int neighborOffset,
                                   SearchFromNeighbor &searchFromNeighbor,
                                   const CH::SearchGraph &neighborGraph,
                                   const BucketContainer &buckets,
                                   EntryInsertionsVecT& entryInsertions
                                   //,
//                                   karri::stats::UpdatePerformanceStats &stats
                                   ) {
//            int64_t numEntriesGenerated = 0;
//            Timer timer;

            auto& localDescHasEntry = descendentHasEntry.local();
            KASSERT(localDescHasEntry.cardinality() == 0);

            // Run topological search from new stop and memorize search space:
            parallel::scalable_vector<int> searchSpace;
            std::get<0>(searchFromNewStop.getPruningCriterion().criterions).setCurrentLeeway(leeway);
            std::get<1>(searchFromNewStop.getPruningCriterion().criterions).setCurSearchSpace(&searchSpace);
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.getStoppingCriterion().setCurrentLeeway(leeway);
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

            entryInsertions.emplace_back(); // Invalid last element to write to.

            for (auto it = searchSpace.crbegin(); it < searchSpace.crend(); ++it) {
                const auto v = *it;

                // Try to find witness for shortest path that has a higher ranked vertex:
                int minDistViaHigherPath = INFTY;
                FORALL_INCIDENT_EDGES(neighborGraph, v, e) {
                    const int higherVertex = neighborGraph.edgeHead(e);
                    const int distViaHigherVertex =
                            searchFromNewStop.getDistance(higherVertex) + neighborGraph.traversalCost(e);
                    minDistViaHigherPath = std::min(minDistViaHigherPath, distViaHigherVertex);
                }
                const bool betterHigherPathExists = minDistViaHigherPath <= searchFromNewStop.getDistance(v);
                searchFromNewStop.distanceLabels[v].min(minDistViaHigherPath);


                // Propagate distances to neighbor down into search space:
                FORALL_INCIDENT_EDGES(newStopGraph, v, e) {
                    const int higherVertex = newStopGraph.edgeHead(e);
                    const int distViaHigherVertex =
                            newStopGraph.traversalCost(e) + searchFromNeighbor.getDistance(higherVertex);
                    searchFromNeighbor.distanceLabels[v].min(distViaHigherVertex);
                }

                // Check if vertex is in ellipse using distances to neighbor:
                const bool inEllipse = searchFromNewStop.getDistance(v) + searchFromNeighbor.getDistance(v) <= leeway;
                if ((!betterHigherPathExists && inEllipse) || localDescHasEntry[v]) {
//                    buckets.insert(v, {stopId, searchFromNewStop.getDistance(v), leeway});
                    buckets.getEntryInsertion(v, {stopId, searchFromNewStop.getDistance(v), leeway}, entryInsertions.back());
                    entryInsertions.emplace_back(); // Invalid last element to write to.
//                    ++numEntriesGenerated;

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries that can be
                    // used in delete operations.
                    localDescHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    localDescHasEntry[v] = false;
                }
            }

            entryInsertions.pop_back(); // Remove unused invalid last element

//            const auto time = timer.elapsed<std::chrono::nanoseconds>();
//            stats.elliptic_generate_time += time;
//            stats.elliptic_generate_numVerticesInSearchSpace += searchSpace.size();
//            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph, const BucketContainer &buckets, std::vector<EntryDeletion>& entryDeletions) {
//            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            int64_t numEntriesScanned = 0;
//            Timer timer;
            entryDeletions.emplace_back(); // Invalid last element to write to

            deleteSearchSpace.clear();
            deleteSearchSpace.insert(root);

            for (auto it = deleteSearchSpace.begin(); it < deleteSearchSpace.end(); ++it) {
                const auto &v = *it;
//                if (buckets.remove(v, stopId)) {
                if (buckets.getEntryDeletion(v, stopId, entryDeletions.back(), numEntriesScanned)) {
                    entryDeletions.emplace_back(); // New invalid last element, previous one has been populated
                    FORALL_INCIDENT_EDGES(graph, v, e) {
                        const auto w = graph.edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
//                ++numVerticesVisited;
            }

            entryDeletions.pop_back(); // Remove unused invalid last element


            //            const auto time = timer.elapsed<std::chrono::nanoseconds>();
//            stats.elliptic_delete_time += time;
//            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
//            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        const CHEnvT& chEnv;

        BucketContainer sourceBuckets;
        BucketContainer targetBuckets;

        tbb::enumerable_thread_specific<BitVector> descendentHasEntry;

        tbb::enumerable_thread_specific<SearchFromNewStop> forwardSearchFromNewStop;
        tbb::enumerable_thread_specific<SearchFromNewStop> reverseSearchFromNewStop;
        tbb::enumerable_thread_specific<SearchFromNeighbor> forwardSearchFromPrevStop;
        tbb::enumerable_thread_specific<SearchFromNeighbor> reverseSearchFromNextStop;

        Subset deleteSearchSpace;

        tbb::enumerable_thread_specific<parallel::scalable_vector<EntryInsertion>> sourceInsertions;
        tbb::enumerable_thread_specific<parallel::scalable_vector<EntryInsertion>> targetInsertions;


        std::vector<EntryDeletion> sourceDeletions;
        std::vector<EntryDeletion> targetDeletions;

    };
}