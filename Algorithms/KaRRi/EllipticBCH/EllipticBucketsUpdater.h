//
// Created by tim on 12.12.23.
//

#pragma once

#include <type_traits>
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "BucketEntryWithLeeway.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS>
    class EllipticBucketsUpdater {

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
                return currentLeeway < distToV;
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

        using BucketContainer = std::conditional_t<SORTED_BUCKETS, //TODO: Buckets rausziehen
                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;

        using BucketType = RouteStateDataType;

        EllipticBucketsUpdater(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                               const InputConfig &inputConfig, karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph), ch(chEnv.getCH()), inputConfig(inputConfig),
                  variableSourceBuckets(inputGraph.numVertices()), variableTargetBuckets(inputGraph.numVertices()),
                  fixedSourceBuckets(inputGraph.numVertices()), fixedTargetBuckets(inputGraph.numVertices()),
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


        const BucketContainer &getSourceBuckets(const BucketType type) const {
            return (type == BucketType::VARIABLE) ? variableSourceBuckets : fixedSourceBuckets;
        }

        const BucketContainer &getTargetBuckets(BucketType type) const {
            return (type == BucketType::VARIABLE) ? variableTargetBuckets : fixedTargetBuckets;
        }

        void generateSourceBucketEntries(const Vehicle &veh, const int stopIndex, RouteStateData &routeStateData) {
            assert(routeStateData.numStopsOf(veh.vehicleId) > stopIndex + 1);

            const int stopId = routeStateData.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeStateData.maxArrTimesFor(veh.vehicleId)[stopIndex + 1],
                                        routeStateData.schedDepTimesFor(veh.vehicleId)[stopIndex + 1]) -
                               routeStateData.schedDepTimesFor(veh.vehicleId)[stopIndex] - inputConfig.stopTime;
            currentLeeway = leeway;

            const int newStopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));

            const int nextStopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex + 1];
            const int nextStopRoot = ch.rank(inputGraph.edgeTail(nextStopLoc));
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);

            if (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) {
                generateBucketEntries(stopId, leeway,
                                      newStopRoot, 0, forwardSearchFromNewStop, ch.upwardGraph(),
                                      nextStopRoot, nextStopOffset, reverseSearchFromNextStop, ch.downwardGraph(),
                                      variableSourceBuckets);
                return;
            }
            generateBucketEntries(stopId, leeway,
                                  newStopRoot, 0, forwardSearchFromNewStop, ch.upwardGraph(),
                                  nextStopRoot, nextStopOffset, reverseSearchFromNextStop, ch.downwardGraph(),
                                  fixedSourceBuckets);

        }

        void generateTargetBucketEntries(const Vehicle &veh, const int stopIndex, RouteStateData &routeStateData) {
            assert(stopIndex > 0);

            const int stopId = routeStateData.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeStateData.maxArrTimesFor(veh.vehicleId)[stopIndex],
                                        routeStateData.schedDepTimesFor(veh.vehicleId)[stopIndex]) -
                               routeStateData.schedDepTimesFor(veh.vehicleId)[stopIndex - 1] - inputConfig.stopTime;
            currentLeeway = leeway;

            const int newStopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeTail(newStopLoc));
            const int newStopOffset = inputGraph.travelTime(newStopLoc);

            const int prevStopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex - 1];
            const int prevStopRoot = ch.rank(inputGraph.edgeHead(prevStopLoc));

            if (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) {
                generateBucketEntries(stopId, leeway,
                                      newStopRoot, newStopOffset, reverseSearchFromNewStop, ch.downwardGraph(),
                                      prevStopRoot, 0, forwardSearchFromPrevStop, ch.upwardGraph(),
                                      variableTargetBuckets);
                return;
            }
            generateBucketEntries(stopId, leeway,
                                  newStopRoot, newStopOffset, reverseSearchFromNewStop, ch.downwardGraph(),
                                  prevStopRoot, 0, forwardSearchFromPrevStop, ch.upwardGraph(),
                                  fixedTargetBuckets);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle& veh, RouteStateData &routeStateData) {
            const auto numStops = routeStateData.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateSourceLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeStateData.vehicleIdOf(e.targetId) != veh.vehicleId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeStateData.leewayOfLegStartingAt(e.targetId);
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeStateData.stopLocationsFor(veh.vehicleId);
            for (int idx = 0; idx < numStops - 1; ++idx) {
                const int root = ch.rank(inputGraph.edgeHead(stopLocations[idx]));
                deleteSearchSpace.insert(root);
            }

            auto &sourceBuckets = (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) ? variableSourceBuckets : fixedSourceBuckets;

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

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle& veh, RouteStateData &routeStateData) {
            const auto numStops = routeStateData.numStopsOf(veh.vehicleId);
            if (numStops <= 1)
                return;
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            auto updateTargetLeeway = [&](BucketEntryWithLeeway& e) {
                if (routeStateData.vehicleIdOf(e.targetId) != veh.vehicleId)
                    return false;
                const auto oldLeeway = e.leeway;
                e.leeway = routeStateData.leewayOfLegStartingAt(routeStateData.idOfPreviousStopOf(e.targetId));
                return e.leeway != oldLeeway;
            };
            deleteSearchSpace.clear();
            const auto stopLocations = routeStateData.stopLocationsFor(veh.vehicleId);
            for (int idx = 1; idx < numStops; ++idx) {
                const int root = ch.rank(inputGraph.edgeTail(stopLocations[idx]));
                deleteSearchSpace.insert(root);
            }

            auto &targetBuckets = (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) ? variableTargetBuckets : fixedTargetBuckets;

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

        void deleteSourceBucketEntries(const Vehicle &veh, const int stopIndex, RouteStateData &routeStateData) {
            const int stopId = routeStateData.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeHead(stopLoc));
            if (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) {
                deleteBucketEntries(stopId, root, ch.upwardGraph(), variableSourceBuckets);
                return;
            }
            deleteBucketEntries(stopId, root, ch.upwardGraph(), fixedSourceBuckets);
        }

        void deleteTargetBucketEntries(const Vehicle &veh, const int stopIndex, RouteStateData &routeStateData) {
            const int stopId = routeStateData.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = ch.rank(inputGraph.edgeTail(stopLoc));

            if (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) {
                deleteBucketEntries(stopId, root, ch.downwardGraph(), variableTargetBuckets);
                return;
            }
            deleteBucketEntries(stopId, root, ch.downwardGraph(), fixedTargetBuckets);
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
                                   BucketContainer &buckets) {
            int64_t numEntriesGenerated = 0;
            Timer timer;

            // Run topological search from new stop and memorize search space:
            searchSpace.clear();
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

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

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries that can be
                    // used in delete operations.
                    descendentHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    descendentHasEntry[v] = false;
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += searchSpace.size();
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph, BucketContainer &buckets) {
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
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_delete_time += time;
            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const InputConfig &inputConfig;

        BucketContainer variableSourceBuckets;
        BucketContainer variableTargetBuckets;

        BucketContainer  fixedSourceBuckets;
        BucketContainer fixedTargetBuckets;

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
