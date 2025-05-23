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

        // Top PEAK_SIZE vertices make up peak for which we store up-down-buckets instead of only up- or down-buckets.
        static constexpr int PEAK_SIZE = 2500;

    public:


        static constexpr bool SORTED_BY_REM_LEEWAY = SORTED_BUCKETS;

        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;

        EllipticBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteState &routeState,
                                   karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph), ch(chEnv.getCH()), routeState(routeState),
                  revUpGraph(ch.upwardGraph().getReverseGraph()),
                  revDownGraph(ch.downwardGraph().getReverseGraph()),
                  sourceBuckets(inputGraph.numVertices()), targetBuckets(inputGraph.numVertices()),
                  sourcePeakBuckets(inputGraph.numVertices()), targetPeakBuckets(inputGraph.numVertices()),
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
                  deleteSearchSpace(inputGraph.numVertices()),
                  stats(stats) {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBuckets;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBuckets;
        }

        const BucketContainer &getSourcePeakBuckets() const {
            return sourcePeakBuckets;
        }

        const BucketContainer &getTargetPeakBuckets() const {
            return targetPeakBuckets;
        }

        void generateSourceBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(routeState.numStopsOf(veh.vehicleId) > stopIndex + 1);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex + 1],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex + 1]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex] -
                               InputConfig::getInstance().stopTime;

            if (leeway <= 0)
                return;

            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));

            const int nextStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex + 1];
            const int nextStopRoot = ch.rank(inputGraph.edgeTail(nextStopLoc));
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);

            generateBucketEntries(stopId, leeway,
                                  newStopRoot, 0, forwardSearchFromNewStop, ch.upwardGraph(), revUpGraph,
                                  nextStopRoot, nextStopOffset, reverseSearchFromNextStop, ch.downwardGraph(),
                                  revDownGraph, sourceBuckets, sourcePeakBuckets);
        }

        void generateTargetBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex > 0);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex - 1] -
                               InputConfig::getInstance().stopTime;
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
                                  revDownGraph, prevStopRoot, 0, forwardSearchFromPrevStop, ch.upwardGraph(),
                                  revUpGraph, targetBuckets, targetPeakBuckets);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle &veh) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateSourceLeeway = [&](BucketEntryWithLeeway &e) {
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
                if (sourceBuckets.updateAllEntries(v, updateSourceLeeway)) {
                    FORALL_INCIDENT_EDGES(ch.upwardGraph(), v, e) {
                        const auto w = ch.upwardGraph().edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
                numEntriesScanned += sourceBuckets.getNumEntriesVisitedInLastUpdateOrRemove();
            }

//            // Update entries in peak buckets:
//            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
//                const auto v = *iter;
//                if (sourcePeakBuckets.updateAllEntries(v, updateSourceLeeway)) {
//                    FORALL_INCIDENT_EDGES(revDownGraph, v, e) {
//                        const auto w = revDownGraph.edgeHead(e);
//                        deleteSearchSpace.insert(w);
//                    }
//                }
//                ++numVerticesVisited;
//                numEntriesScanned += sourcePeakBuckets.getNumEntriesVisitedInLastUpdateOrRemove();
//            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_update_time += time;
            stats.elliptic_update_numVerticesVisited += numVerticesVisited;
            stats.elliptic_update_numEntriesScanned += numEntriesScanned;
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle &veh) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateTargetLeeway = [&](BucketEntryWithLeeway &e) {
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
                if (targetBuckets.updateAllEntries(v, updateTargetLeeway)) {
                    FORALL_INCIDENT_EDGES(ch.downwardGraph(), v, e) {
                        const auto w = ch.downwardGraph().edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
                numEntriesScanned += targetBuckets.getNumEntriesVisitedInLastUpdateOrRemove();
            }

//            // Update entries in peak buckets:
//            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
//                const auto v = *iter;
//                if (targetPeakBuckets.updateAllEntries(v, updateTargetLeeway)) {
//                    FORALL_INCIDENT_EDGES(revUpGraph, v, e) {
//                        const auto w = revUpGraph.edgeHead(e);
//                        deleteSearchSpace.insert(w);
//                    }
//                }
//                ++numVerticesVisited;
//                numEntriesScanned += targetPeakBuckets.getNumEntriesVisitedInLastUpdateOrRemove();
//            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_update_time += time;
            stats.elliptic_update_numVerticesVisited += numVerticesVisited;
            stats.elliptic_update_numEntriesScanned += numEntriesScanned;
        }

        void deleteSourceBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeHead(stopLoc));
            deleteBucketEntries(stopId, root, ch.upwardGraph(), revDownGraph, sourceBuckets, sourcePeakBuckets);
        }

        void deleteTargetBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeTail(stopLoc));
            deleteBucketEntries(stopId, root, ch.downwardGraph(), revUpGraph, targetBuckets, targetPeakBuckets);
        }

        static bool isInPeak(const int v, const int numVertices) {
            return v >= numVertices - PEAK_SIZE;
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
                                   const CH::SearchGraph &revNewStopGraph,
                                   const int neighborRoot, const int neighborOffset,
                                   SearchFromNeighbor &searchFromNeighbor,
                                   const CH::SearchGraph &neighborGraph,
                                   const CH::SearchGraph &revNeighborGraph,
                                   BucketContainer &buckets,
                                   BucketContainer &peakBuckets) {
            int64_t numEntriesGenerated = 0;
            Timer timer;

            // Run topological search from new stop and memorize search space:
            searchSpace.clear();
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

            int highestWithEntry = -1;
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
                    buckets.insert(v, {stopId, searchFromNewStop.getDistance(v), leeway});
                    ++numEntriesGenerated;
                    highestWithEntry = std::max(highestWithEntry, v);

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries that can be
                    // used in delete operations.
                    descendentHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    descendentHasEntry[v] = false;
                }
            }

            // Propagate distances from new stop and distance to neighbor to all vertices in the peak and
            // add entries to peak buckets:
            int numPeakEntriesGenerated = 0;
            for (int v = highestWithEntry; v >= inputGraph.numVertices() - PEAK_SIZE; --v) {

                // Distances at v are final, so we can check if v is in the ellipse and needs a peak bucket entry.
                const int distFromNewStop = searchFromNewStop.getDistance(v);
                const int distToNeighbor = searchFromNeighbor.getDistance(v);
                const bool inEllipse = distFromNewStop + distToNeighbor <= leeway;
                KASSERT(v != highestWithEntry || inEllipse);

                if (!inEllipse)
                    continue;

                peakBuckets.insert(v, {stopId, searchFromNewStop.getDistance(v), leeway});
                ++numPeakEntriesGenerated;

                // Propagate distances from new stop along downward edges in neighbor graph:
                FORALL_INCIDENT_EDGES(revNeighborGraph, v, e) {
                    const int lowerVertex = revNeighborGraph.edgeHead(e);
                    const int distViaV = distFromNewStop + revNeighborGraph.traversalCost(e);
                    searchFromNewStop.distanceLabels[lowerVertex].min(distViaV);
                }

                // Propagate distances to neighbor along downward edges in new stop graph:
                FORALL_INCIDENT_EDGES(revNewStopGraph, v, e) {
                    const int lowerVertex = revNewStopGraph.edgeHead(e);
                    const int distViaV = distToNeighbor + revNewStopGraph.traversalCost(e);
                    searchFromNeighbor.distanceLabels[lowerVertex].min(distViaV);
                }
            }
            unused(numPeakEntriesGenerated);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += searchSpace.size();
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph,
                            const CH::SearchGraph &revOppositeGraph,
                            BucketContainer &buckets, BucketContainer &peakBuckets) {
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            deleteSearchSpace.clear();
            deleteSearchSpace.insert(root);
            for (auto it = deleteSearchSpace.begin(); it < deleteSearchSpace.end(); ++it) {
                const auto &v = *it;
                if (buckets.remove(v, stopId)) {
                    FORALL_INCIDENT_EDGES(graph, v, e) {
                        const auto w = graph.edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
                numEntriesScanned += buckets.getNumEntriesVisitedInLastUpdateOrRemove();
            }

            // Remove entries from peak buckets:
            for (auto it = deleteSearchSpace.begin(); it < deleteSearchSpace.end(); ++it) {
                const auto &v = *it;
                if (peakBuckets.remove(v, stopId)) {
                    FORALL_INCIDENT_EDGES(revOppositeGraph, v, e) {
                        const auto w = revOppositeGraph.edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_delete_time += time;
            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        CH::SearchGraph revUpGraph; // Reverse of ch.upwardGraph(), i.e. reverse upward edges
        CH::SearchGraph revDownGraph; // Reverse of ch.downwardGraph(), i.e. actual downward edges

        BucketContainer sourceBuckets;
        BucketContainer targetBuckets;

        // Up-down buckets for small subset of vertices at the top of the CH (peak vertices).
        BucketContainer sourcePeakBuckets;
        BucketContainer targetPeakBuckets;

        SearchFromNewStop forwardSearchFromNewStop;
        SearchFromNewStop reverseSearchFromNewStop;
        SearchFromNeighbor forwardSearchFromPrevStop;
        SearchFromNeighbor reverseSearchFromNextStop;
        int currentLeeway;


        std::vector<int> searchSpace;
        BitVector descendentHasEntry;

        Subset deleteSearchSpace;

        karri::stats::UpdatePerformanceStats &stats;
    };
}