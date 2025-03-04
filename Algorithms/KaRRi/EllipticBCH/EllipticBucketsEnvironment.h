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
            explicit StopWhenLeewayExceeded(const int &currentLeeway) : currentLeeway(currentLeeway) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, const DistLabelT &distToV, const DistLabelContT &) const noexcept {
                return allSet(distToV > currentLeeway);
            }

            const int &currentLeeway;
        };

        struct StoreSearchSpace {
            explicit StoreSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                searchSpace.push_back(v);
                return false;
            }

            std::vector<int> &searchSpace;
        };

    public:


        static constexpr bool SORTED_BY_REM_LEEWAY = SORTED_BUCKETS;

        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                CompactSortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
//                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;

        EllipticBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteState &routeState)
                : inputGraph(inputGraph), ch(chEnv.getCH()), routeState(routeState),
                  sourceBuckets(inputGraph.numVertices()), targetBuckets(inputGraph.numVertices()),
                  forwardSearchFromNewStop(
                          chEnv.getForwardTopologicalSearch(StoreSearchSpace(searchSpace),
                                                            StopWhenLeewayExceeded(currentLeeway))),
                  reverseSearchFromNewStop(
                          chEnv.getReverseTopologicalSearch(StoreSearchSpace(searchSpace),
                                                            StopWhenLeewayExceeded(currentLeeway))),
                  forwardSearchFromPrevStop(chEnv.getForwardSearch({}, StopWhenLeewayExceeded(currentLeeway))),
                  reverseSearchFromNextStop(chEnv.getReverseSearch({}, StopWhenLeewayExceeded(currentLeeway))),
                  currentLeeway(INFTY),
                  searchSpace(),
                  descendentHasEntry(inputGraph.numVertices()),
                  deleteSearchSpace(inputGraph.numVertices()) {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBuckets;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBuckets;
        }

        void generateSourceBucketEntries(const Vehicle &veh, const int stopIndex, karri::stats::UpdatePerformanceStats &stats) {
            assert(routeState.numStopsOf(veh.vehicleId) > stopIndex + 1);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex + 1],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex + 1]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex] - InputConfig::getInstance().stopTime;

            if (leeway <= 0)
                return;

            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));

            const int nextStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex + 1];
            const int nextStopRoot = ch.rank(inputGraph.edgeTail(nextStopLoc));
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);

            generateBucketEntries(stopId, leeway,
                                  newStopRoot, 0, forwardSearchFromNewStop, ch.upwardGraph(),
                                  nextStopRoot, nextStopOffset, reverseSearchFromNextStop, ch.downwardGraph(),
                                  sourceBuckets, stats);
        }

        void generateTargetBucketEntries(const Vehicle &veh, const int stopIndex, karri::stats::UpdatePerformanceStats &stats) {
            assert(stopIndex > 0);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex - 1] - InputConfig::getInstance().stopTime;
            if (leeway <= 0)
                return;

            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeTail(newStopLoc));
            const int newStopOffset = inputGraph.travelTime(newStopLoc);

            const int prevStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex - 1];
            const int prevStopRoot = ch.rank(inputGraph.edgeHead(prevStopLoc));

            generateBucketEntries(stopId, leeway,
                                  newStopRoot, newStopOffset, reverseSearchFromNewStop, ch.downwardGraph(),
                                  prevStopRoot, 0, forwardSearchFromPrevStop, ch.upwardGraph(),
                                  targetBuckets, stats);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle& veh, karri::stats::UpdatePerformanceStats &stats) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateSourceLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeState.vehicleIdOf(e.targetId) != veh.vehicleId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeState.leewayOfLegStartingAt(e.targetId);
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
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

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle& veh, karri::stats::UpdatePerformanceStats &stats) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateTargetLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeState.vehicleIdOf(e.targetId) != veh.vehicleId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeState.leewayOfLegStartingAt(routeState.idOfPreviousStopOf(e.targetId));
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
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

        void deleteSourceBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeHead(stopLoc));
            deleteBucketEntries(stopId, root, ch.upwardGraph(), sourceBuckets);
        }

        void deleteTargetBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeTail(stopLoc));
            deleteBucketEntries(stopId, root, ch.downwardGraph(), targetBuckets);
        }

    private:


        // Searches for inserting new stops: Topo search from/to new stop, regular CH searches to/from neighboring stops.
        using SearchFromNewStop = typename CHEnvT::template TopologicalUpwardSearch<
                StoreSearchSpace, StopWhenLeewayExceeded>;
        using SearchFromNeighbor = typename CHEnvT::template UpwardSearch<
                dij::NoCriterion, StopWhenLeewayExceeded>;

        void generateBucketEntries(const int stopId, const int leeway,
                                   const int newStopRoot, const int newStopOffSet, SearchFromNewStop &searchFromNewStop,
                                   const CH::SearchGraph &newStopGraph,
                                   const int neighborRoot, const int neighborOffset,
                                   SearchFromNeighbor &searchFromNeighbor,
                                   const CH::SearchGraph &neighborGraph,
                                   BucketContainer &buckets,
                                   karri::stats::UpdatePerformanceStats &stats) {
            int64_t numEntriesGenerated = 0;
            Timer timer;

            // Run topological search from new stop and memorize search space:
            searchSpace.clear();
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

            std::vector<typename BucketContainer::EntryInsertion> entryInsertions;
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
                if ((!betterHigherPathExists && inEllipse) || descendentHasEntry[v]) {
//                    buckets.insert(v, {stopId, searchFromNewStop.getDistance(v), leeway});
                    buckets.getEntryInsertion(v, {stopId, searchFromNewStop.getDistance(v), leeway}, entryInsertions.back());
                    entryInsertions.emplace_back(); // Invalid last element to write to.
                    ++numEntriesGenerated;

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries that can be
                    // used in delete operations.
                    descendentHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    descendentHasEntry[v] = false;
                }
            }

            entryInsertions.pop_back(); // Remove unused invalid last element
            KASSERT(entryInsertions.size() == numEntriesGenerated);
            // TODO: accumulate insertions for entire batch of requests and run this once.
            buckets.batchedCommitInsertions(entryInsertions); // Commit entry insertions


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += searchSpace.size();
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph, BucketContainer &buckets) {
//            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            int64_t numEntriesScanned = 0;
//            Timer timer;
            std::vector<typename BucketContainer::EntryDeletion> entryDeletions;
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
            std::vector<typename BucketContainer::EntryInsertion> placeholder;
            buckets.batchedCommitInsertionsAndDeletions(placeholder, entryDeletions); // Commit entry deletions


            //            const auto time = timer.elapsed<std::chrono::nanoseconds>();
//            stats.elliptic_delete_time += time;
//            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
//            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        BucketContainer sourceBuckets;
        BucketContainer targetBuckets;

        SearchFromNewStop forwardSearchFromNewStop;
        SearchFromNewStop reverseSearchFromNewStop;
        SearchFromNeighbor forwardSearchFromPrevStop;
        SearchFromNeighbor reverseSearchFromNextStop;
        int currentLeeway;

        std::vector<int> searchSpace;
        BitVector descendentHasEntry;

        Subset deleteSearchSpace;
    };
}