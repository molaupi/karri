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

    // Does not actually generate bucket entries only at stop within ellipse but anywhere within travel time radius
    // of leeway around stop.
    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS>
    class RadiusEllipticBucketsEnvironment {

        using Entry = BucketEntryWithLeeway;

        // Buckets are sorted by remaining leeway. Allows early stop of bucket scan.
        struct DoesEntryHaveLargerRemainingLeeway {
            bool operator()(const Entry &e1, const Entry &e2) const {
                return e1.leeway - e1.travelTimeToTarget > e2.leeway - e2.travelTimeToTarget;
            }
        };

    public:
        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;

    private:

        struct PruneIfTravelTimeExceedsLeeway {
            explicit PruneIfTravelTimeExceedsLeeway(StampedDistanceLabelContainer<int> &travelTimes,
                                                    const int &currentLeeway)
                    : travelTimes(travelTimes), currentLeeway(currentLeeway) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                return travelTimes[v] > currentLeeway;
            }

            StampedDistanceLabelContainer<int> &travelTimes;
            const int &currentLeeway;
        };

        struct GenerateEntry {
            explicit GenerateEntry(BucketContainer &bucketContainer, StampedDistanceLabelContainer<int> &travelTimes,
                                   int &curStopId, int &curLeeway, int &verticesVisited)
                    : bucketContainer(bucketContainer), travelTimes(travelTimes), curStopId(curStopId),
                      curLeeway(curLeeway), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntryWithLeeway(curStopId, distToV[0], travelTimes[v], curLeeway);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            StampedDistanceLabelContainer<int> &travelTimes;
            int &curStopId;
            int &curLeeway;
            int &verticesVisited;
        };

        struct UpdateTravelTimeCallback {

            UpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                     StampedDistanceLabelContainer<int> &travelTimes) : searchGraph(searchGraph),
                                                                                        travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                if (improved[0])
                    travelTimes[w] = travelTimes[v] + searchGraph.travelTime(e);
            }

            const CH::SearchGraph &searchGraph;
            StampedDistanceLabelContainer<int> &travelTimes;
        };

        using GenerateBucketEntriesSearch = typename CHEnvT::template UpwardSearch<dij::CompoundCriterion<PruneIfTravelTimeExceedsLeeway, GenerateEntry>, dij::NoCriterion, UpdateTravelTimeCallback>;


    public:

        static constexpr bool SORTED_BY_REM_LEEWAY = SORTED_BUCKETS;


        RadiusEllipticBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                         const RouteState &routeState,
                                         karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph), ch(chEnv.getCH()), routeState(routeState),
                  sourceBuckets(inputGraph.numVertices()), targetBuckets(inputGraph.numVertices()),
                  travelTimes(inputGraph.numVertices()),
                  forwardSearchFromNewStop(
                          chEnv.getForwardSearch({PruneIfTravelTimeExceedsLeeway(travelTimes, currentLeeway),
                                                  GenerateEntry(sourceBuckets, travelTimes, curStopId, currentLeeway, numVerticesVisited)},
                                                 {}, UpdateTravelTimeCallback(ch.upwardGraph(), travelTimes))),
                  reverseSearchFromNewStop(
                          chEnv.getReverseSearch({PruneIfTravelTimeExceedsLeeway(travelTimes, currentLeeway),
                                                  GenerateEntry(targetBuckets, travelTimes, curStopId, currentLeeway, numVerticesVisited)},
                                                 {}, UpdateTravelTimeCallback(ch.downwardGraph(), travelTimes))),
                  curStopId(INVALID_ID),
                  currentLeeway(INFTY),
                  numVerticesVisited(0),
                  deleteSearchSpace(inputGraph.numVertices()),
                  stats(stats) {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBuckets;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBuckets;
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

            curStopId = stopId;
            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));
            generateBucketEntries(newStopRoot, 0, 0, forwardSearchFromNewStop);
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

            curStopId = stopId;
            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeTail(newStopLoc));
            const int newStopCostOffset = inputGraph.traversalCost(newStopLoc);
            const int newStopTravelTimeOffset = inputGraph.travelTime(newStopLoc);
            generateBucketEntries(newStopRoot, newStopCostOffset, newStopTravelTimeOffset, reverseSearchFromNewStop);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle &veh) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numEntriesScanned = 0;
            numVerticesVisited = 0;
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
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_update_time += time;
            stats.elliptic_update_numVerticesVisited += numVerticesVisited;
            stats.elliptic_update_numEntriesScanned += numEntriesScanned;
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle &veh) {
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numEntriesScanned = 0;
            numVerticesVisited = 0;
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

        void generateBucketEntries(const int newStopRoot,
                                   const int newStopCostOffset,
                                   const int newStopTravelTimeOffset,
                                   GenerateBucketEntriesSearch &search) {
            int64_t numEntriesGenerated = 0;
            Timer timer;

            numVerticesVisited = 0;
            travelTimes.init();
            travelTimes[newStopRoot] = newStopTravelTimeOffset;

            search.runWithOffset(newStopRoot, newStopCostOffset);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += numVerticesVisited;
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph, BucketContainer &buckets) {
            int64_t numEntriesScanned = 0;
            numVerticesVisited = 0;
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
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_delete_time += time;
            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        BucketContainer sourceBuckets;
        BucketContainer targetBuckets;

        StampedDistanceLabelContainer<int> travelTimes;
        GenerateBucketEntriesSearch forwardSearchFromNewStop;
        GenerateBucketEntriesSearch reverseSearchFromNewStop;
        int curStopId;
        int currentLeeway;

        int numVerticesVisited;

        BitVector descendentHasEntry;

        Subset deleteSearchSpace;

        karri::stats::UpdatePerformanceStats &stats;
    };
}